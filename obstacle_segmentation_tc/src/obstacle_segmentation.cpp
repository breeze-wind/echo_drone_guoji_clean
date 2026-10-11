#include "obstacle_segmentation/obstacle_segmentation.hpp"

#include <cmath>

ObstacleSegmentationNode::ObstacleSegmentationNode(std::string name, const rclcpp::NodeOptions& options)
    : Node(name, options)
{
    RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
    // 声明参数
    this->declare_parameter("input_cloud_topic", "input_cloud");
    this->declare_parameter("output_cloud_topic", "output_cloud");
    this->declare_parameter("output_frame", "map");
    this->declare_parameter("base_frame", "base_link");
    this->declare_parameter("leaf_size", 0.1);
    this->declare_parameter("point_num_for_normal", 50);
    this->declare_parameter("angle_threshold", 0.1);
    this->declare_parameter("obstacle_x_min", -10.0);
    this->declare_parameter("obstacle_x_max", 10.0);
    this->declare_parameter("obstacle_y_min", -10.0);
    this->declare_parameter("obstacle_y_max", 10.0);
    this->declare_parameter("obstacle_z_min", 0.0);
    this->declare_parameter("obstacle_z_max", 2.0);
    this->declare_parameter("obstacle_range_min", 0.5);
    this->declare_parameter("obstacle_range_max", 2.0);
    this->declare_parameter("body_min_x", -0.3);
    this->declare_parameter("body_max_x", 0.3);
    this->declare_parameter("body_min_y", -0.2);
    this->declare_parameter("body_max_y", 0.2);
    this->declare_parameter("use_downsample", true);

    RCLCPP_INFO(this->get_logger(), "点云分割节点初始化");
    this->get_parameter("input_cloud_topic", input_cloud_topic_);
    this->get_parameter("output_cloud_topic", output_cloud_topic_);
    this->get_parameter("output_frame", output_frame_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("leaf_size", leaf_size_);
    this->get_parameter("point_num_for_normal", point_num_for_normal_);
    this->get_parameter("angle_threshold", angle_threshold_);
    this->get_parameter("obstacle_x_min", obstacle_x_min_);
    this->get_parameter("obstacle_x_max", obstacle_x_max_);
    this->get_parameter("obstacle_y_min", obstacle_y_min_);
    this->get_parameter("obstacle_y_max", obstacle_y_max_);
    this->get_parameter("obstacle_z_min", obstacle_z_min_);
    this->get_parameter("obstacle_z_max", obstacle_z_max_);
    this->get_parameter("obstacle_range_min", obstacle_range_min_);
    this->get_parameter("obstacle_range_max", obstacle_range_max_);
    this->get_parameter("use_downsample", use_downsample_);

    // 设置滤波器的体素大小
    pass_through_filter_x_.setFilterFieldName("x");
    pass_through_filter_x_.setFilterLimits(obstacle_x_min_, obstacle_x_max_);
    pass_through_filter_x_.setFilterLimitsNegative(false);
    pass_through_filter_y_.setFilterFieldName("y");
    pass_through_filter_y_.setFilterLimits(obstacle_y_min_, obstacle_y_max_);
    pass_through_filter_y_.setFilterLimitsNegative(false);
    pass_through_filter_z_.setFilterFieldName("z");
    pass_through_filter_z_.setFilterLimits(obstacle_z_min_, obstacle_z_max_);
    pass_through_filter_z_.setFilterLimitsNegative(false);
    voxfilter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);

    obstacle_height_ = 2.0;

    pcl_cnt = 0;

    if_need_clear = false;

    output_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();

    blank_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    segement_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    tfbuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tfbuffer_);
    // 初始化pub和sub
    output_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, 10);
    input_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_cloud_topic_, 10, std::bind(&ObstacleSegmentationNode::cloudCallback, this, std::placeholders::_1));
    clear_state_sub_ = this->create_subscription<std_msgs::msg::Bool>("/robot/clear_state",
            10, std::bind(&ObstacleSegmentationNode::ClearStateCallback, this, std::placeholders::_1));
    obstacle_height_sub_ = this->create_subscription<std_msgs::msg::Float64>("/robot/obstacle_height", 10,
        std::bind(&ObstacleSegmentationNode::ObstacleHeightCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "点云分割节点初始化完成");
}

void ObstacleSegmentationNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (msg->data.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "接收到的点云数据为空.");
        segement_cloud->clear();
        pcl_cnt = 0;
        return;
    }
    if (msg->header.frame_id.empty())
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Input cloud has no frame_id; dropping it rather than guessing a transform");
        segement_cloud->clear();
        pcl_cnt = 0;
        return;
    }

    geometry_msgs::msg::TransformStamped cloud_to_map;
    try
    {
        cloud_to_map = tfbuffer_->lookupTransform(
            output_frame_, msg->header.frame_id, rclcpp::Time(msg->header.stamp),
            rclcpp::Duration::from_seconds(0.15));
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "No %s <- %s TF at cloud stamp; dropping cloud: %s",
            output_frame_.c_str(), msg->header.frame_id.c_str(), ex.what());
        segement_cloud->clear();
        pcl_cnt = 0;
        return;
    }

    // 将点云转换为pcl格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    const Eigen::Matrix4f cloud_to_map_matrix =
        tf2::transformToEigen(cloud_to_map).matrix().cast<float>();
    pcl::transformPointCloud(*cloud, *cloud, cloud_to_map_matrix);
    const float sensor_z_in_map = static_cast<float>(cloud_to_map.transform.translation.z);

    pcl_cnt++;

    for (const auto& point : cloud->points)
    {
        if (!std::isfinite(point.x) ||
            !std::isfinite(point.y) ||
            !std::isfinite(point.z))
        {
            continue;
        }
        if (point.z < 0.2 || point.z > obstacle_height_)
        {
            continue;
        }
        if (point.z - sensor_z_in_map < 0.1)
        {
            if (point.z - sensor_z_in_map > -0.2)
            {
                segement_cloud->points.push_back(point);
            }
        }
    }

    for (auto& point : segement_cloud->points)
    {
        point.z = 0.0;
    }

    if (pcl_cnt > 2)
    {
        // if(1)
        // {
        //     for (int j = 0; j < 1000; ++j)
        //     {
        //         pcl::PointXYZ point;
        //         point.x = -1.0;
        //         point.y = -2.5 / 500 + 8.0 / 1000 * j;
        //         point.z = 0.0;
        //         segement_cloud->points.push_back(point);
        //     }
        //     for (int j = 0; j < 1000; ++j)
        //     {
        //         pcl::PointXYZ point;
        //         point.x = 6.7;
        //         point.y = -2.5 / 500 + 8.0 / 1000 * j;
        //         point.z = 0.0;
        //         segement_cloud->points.push_back(point);
        //     }
        //     for (int j = 0; j < 1000; ++j)
        //     {
        //         pcl::PointXYZ point;
        //         point.x = -1.0 / 500 + 6.7 / 1000 * j;
        //         point.y = 8.0;
        //         point.z = 0.0;
        //         segement_cloud->points.push_back(point);
        //     }
        //     for (int j = 0; j < 1000; ++j)
        //     {
        //         pcl::PointXYZ point;
        //         point.x = -1.0 / 500 + 6.7 / 1000 * j;
        //         point.y = -2.5;
        //         point.z = 0.0;
        //         segement_cloud->points.push_back(point);
        //     }
        // }

        segement_cloud->width = segement_cloud->points.size();
        segement_cloud->height = 1;
        segement_cloud->is_dense = true;

        pcl::toROSMsg(*segement_cloud, *output_cloud);

        output_cloud->header.frame_id = output_frame_;
        output_cloud->header.stamp = msg->header.stamp;
        output_cloud_pub_->publish(*output_cloud);
        segement_cloud->points.clear();
        pcl_cnt = 0;
    }

    // RCLCPP_INFO(this->get_logger(), "障碍物点云数据正在发布");
}

void ObstacleSegmentationNode::ClearStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if_need_clear = msg->data;
}

void ObstacleSegmentationNode::ObstacleHeightCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    obstacle_height_ = msg->data;
}
