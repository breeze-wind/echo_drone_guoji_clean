//
// Created by elsa on 25-2-18.
//

#ifndef OBSTACLE_SEGMENTATION_NODE_HPP
#define OBSTACLE_SEGMENTATION_NODE_HPP

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/radius_outlier_removal.h>      //半径滤波器头文件

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class ObstacleSegmentationNode : public rclcpp::Node
{
public:
    explicit ObstacleSegmentationNode(std::string name, const rclcpp::NodeOptions &options);

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    /// 接收从point-lio节点传来的当前位姿
    void CurrentPoseCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg);

    void ClearStateCallback(const std_msgs::msg::Bool::SharedPtr msg);

    void ObstacleHeightCallback(const std_msgs::msg::Float64::SharedPtr msg);

    double odom_array[7];

private:
    // 创建滤波器对象
    pcl::PassThrough<pcl::PointXYZ> pass_through_filter_x_;
    pcl::PassThrough<pcl::PointXYZ> pass_through_filter_y_;
    pcl::PassThrough<pcl::PointXYZ> pass_through_filter_z_;

    pcl::VoxelGrid<pcl::PointXYZ> voxfilter;

    std::string input_cloud_topic_;
    std::string output_cloud_topic_;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    float leaf_size_;          // 体素滤波器的体素大小
    int point_num_for_normal_; // 用于计算法向量的点数
    float angle_threshold_;    // 法向量与地面的夹角阈值
    float obstacle_x_min_;     // 障碍物点云范围(livox坐标系)
    float obstacle_x_max_;
    float obstacle_y_min_;
    float obstacle_y_max_;
    float obstacle_z_min_;
    float obstacle_z_max_;
    float obstacle_range_min_; // 障碍物点云范围(livox坐标系)
    float obstacle_range_max_;
    bool use_downsample_;
    bool if_need_clear;
    int cout;
    std::string base_frame_;
    double current_z_;
    double obstacle_height_;

    int pcl_cnt;

    sensor_msgs::msg::PointCloud2::SharedPtr output_cloud;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr blank_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr segement_cloud;

    std::unique_ptr<tf2_ros::Buffer> tfbuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    /// 接收当前位姿
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr current_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_cloud_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr obstacle_height_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr clear_state_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_cloud_pub_;
};

#endif //OBSTACLE_SEGMENTATION_NODE_HPP
