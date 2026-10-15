//
// Created by elsa on 25-2-18.
//

#ifndef OBSTACLE_SEGMENTATION_NODE_HPP
#define OBSTACLE_SEGMENTATION_NODE_HPP

#include <chrono>
#include <deque>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <utility>
#include <unordered_map>
#include <cstdint>

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
#include "sensor_msgs/msg/laser_scan.hpp"

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

    void ClearStateCallback(const std_msgs::msg::Bool::SharedPtr msg);

    void ObstacleHeightCallback(const std_msgs::msg::Float64::SharedPtr msg);

private:
    using SteadyTime = std::chrono::steady_clock::time_point;
    struct PendingCloud
    {
        sensor_msgs::msg::PointCloud2::SharedPtr msg;
        SteadyTime queued_at;
    };
    struct ObstacleCellState
    {
        SteadyTime last_seen;
        SteadyTime clear_until;
    };

    // 创建滤波器对象
    pcl::PassThrough<pcl::PointXYZ> pass_through_filter_x_;
    pcl::PassThrough<pcl::PointXYZ> pass_through_filter_y_;
    pcl::PassThrough<pcl::PointXYZ> pass_through_filter_z_;

    pcl::VoxelGrid<pcl::PointXYZ> voxfilter;

    std::string input_cloud_topic_;
    std::string output_cloud_topic_;
    std::string clearing_scan_topic_;
    std::string output_frame_;

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
    double obstacle_above_sensor_max_m_;
    double body_min_x_;
    double body_max_x_;
    double body_min_y_;
    double body_max_y_;
    double body_min_z_;
    double body_max_z_;
    float obstacle_range_min_; // 障碍物点云范围(livox坐标系)
    float obstacle_range_max_;
    bool use_downsample_;
    bool if_need_clear;
    int cout;
    std::string base_frame_;
    double obstacle_height_;
    double obstacle_cell_stale_timeout_sec_;
    double clear_ray_max_range_;
    double clear_grid_resolution_;
    double tf_wait_timeout_sec_;
    double clearing_max_cloud_age_sec_;
    int max_pending_clouds_;
    std::deque<PendingCloud> pending_clouds_;
    rclcpp::TimerBase::SharedPtr pending_cloud_timer_;
    int64_t last_enqueued_stamp_ns_{0};
    int64_t last_processed_stamp_ns_{0};
    uint64_t tf_timeout_drop_count_{0};
    uint64_t queue_overflow_drop_count_{0};
    uint64_t tf_resolved_count_{0};
    double tf_wait_total_sec_{0.0};
    double tf_wait_max_sec_{0.0};
    SteadyTime last_queue_stats_time_{};
    SteadyTime last_clearing_maintenance_time_{};
    std::unordered_map<int64_t, ObstacleCellState> obstacle_cell_states_;

    int pcl_cnt;

    void processPendingClouds();
    void processCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                      const geometry_msgs::msg::TransformStamped& cloud_to_map);

    sensor_msgs::msg::PointCloud2::SharedPtr output_cloud;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr blank_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr segement_cloud;

    std::unique_ptr<tf2_ros::Buffer> tfbuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_cloud_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr obstacle_height_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr clear_state_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr clearing_cloud_pub_;
};

#endif //OBSTACLE_SEGMENTATION_NODE_HPP
