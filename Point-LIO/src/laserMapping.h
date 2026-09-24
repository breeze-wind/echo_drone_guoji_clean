#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include<iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "IMU_Processing.hpp"
#include "parameters.h"
#include "Estimator.h"

#include"rclcpp/rclcpp.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

#include"nav_msgs/msg/odometry.hpp"
#include"nav_msgs/msg/path.hpp"

#include"visualization_msgs/msg/marker.hpp"

#include"sensor_msgs/msg/point_cloud2.hpp"

#include<tf2/transform_datatypes.h>
#include<tf2_ros/transform_broadcaster.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include"geometry_msgs/msg/vector3.hpp"

#include"livox_ros_driver2/msg/custom_msg.h"

#include <std_msgs/msg/bool.hpp>

#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

class LaserMappingNode : public rclcpp::Node
{
public:
    /**
    * @brief 构造函数
    */
    explicit LaserMappingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
    * @brief 析构函数
    */
    ~LaserMappingNode();

    /**
    * @brief 抛出异常
    */
    void SigHandle(int sig);

private:
    /**
    * @brief 从yaml文件中读取要用到的ros参数
    */
    void readParameters();

    /**
    * @brief 加载先验地图
    * @param filename 先验地图.pcd文件的路径
    */
    PointCloudXYZI::Ptr loadPointcloudFromPcd(const std::string &filename);

    /**
    * @brief 从ros时间戳中获取秒
    */
    double get_time_sec(const builtin_interfaces::msg::Time &time);

    /**
    * @brief 将当前时间转换为ros时间戳
    */
    rclcpp::Time get_ros_time(double timestamp);

    /**
    * @brief 将点云从雷达坐标系根据外参Lidar_T/R_wrt_IMU转换到imu坐标系
    */
    void pointBodyLidarToIMU(PointType const * const pi, PointType * const po);

    /**
    * @brief 获取ikdtree中已经被删除的点
    */
    void points_cache_collect();

    /**
    * @brief 将距离当前雷达远的之前录入的点云从ikdtree中删除
    *
    */
    void lasermap_fov_segment();

    /**
    * @brief 根据选项，将点云存入lidar_buffer中。可选1：将一帧点云按时间分开
    * @brief 2：将连续帧点云存入lidar_buffer中
    * @brief 3：将每帧点云存入lidar_buffer中
    *
    * @param msg
    */
    void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    /**
    * @brief 根据选项，将点云存入lidar_buffer中。可选1：将一帧点云按时间分开
    * @brief 2：将连续帧点云存入lidar_buffer中
    * @brief 3：将每帧点云存入lidar_buffer中
    *
    * @param msg
    */
    void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);

    /**
    * @brief 将imu数据加入队列
    *
    * @param msg_in
    */
    void imu_cbk(const sensor_msgs::msg::Imu::SharedPtr msg_in);

    /**
    * @brief 将lidar_buffer中的最早一份点云传给Measures，并根据条件初始化Measures中的imu
    *
    * @param meas
    * @return true 更新成功
    * @return false 更新失败,可能是lidar_buffer为空,可能imu数据还没更新
    */
    bool sync_packages(MeasureGroup &meas);

    /**
     * @brief ikdtree地图增量更新（feats_down_world点云）
     *
    */
    void map_incremental();

    /**
     * @brief 发布初始ikdtree地图
     *
     * @param pubLaserCloudFullRes Publisher
    */
    void publish_init_kdtree(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes);

    /**
     * @brief 发布这一帧下采样后的世界坐标系点云
     *
     * @param pubLaserCloudFullRes Publisher
    */
    void publish_frame_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes);

    /**
    * @brief 发布这一帧未处理过的原始车体坐标系点云
    *
    * @param pubLaserCloudFull_body Publisher
    */
    void publish_frame_body(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body);

    /**
    * @brief pubLaserCloudObstacle_body发布这一帧的障碍物点云
    *
    * @param pubLaserCloudObstacle_body Publisher
    */
    void publish_obstacle_frame_body(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudObstacle_body);

    /**
    * @brief 将这一帧的车体坐标系点云加到之前的所有点云中，发布所有点云的叠加
    *
    * @param pubLaserCloudMap Publisher
    */
    void publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap);

    /**
    * @brief 根据当前的Kf_output更新当前位姿
    *
    * @tparam T
    * @param out
    */
    template<typename T>
    void set_posestamp(T & out);

    /**
    * @brief 发布set_posestamp函数更新后的当前位姿（坐标系变换）到robot_frame坐标系
    *
    * @param pubOdomAftMapped
    * @param tf_br
    */
    void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped,
        std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_br);
    /**
    * @brief pubPath
    *
    *
    */
    void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath);
    /**
    * @brief pubLaserCloudObstacle_body发布这一帧的障碍物点云
    *
    */
    void map_publish_callback();
    /**
    * @brief 计时器回调函数，控制程序以一定的频率运行
    */
    void timer_callback();

    //初始化发布者和订阅者
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::SubscriptionOptions sub_option;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc; //订阅PointCloud2类型的点云消息
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_; //订阅CustomMsg类型的点云消息
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu; //订阅imu消息
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull; //发布下采样后世界坐标系下的点云 /cloud_registered
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body; //发布这一帧机体坐标系下的点云 /cloud_registered_body
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudObstacle; //发布这一帧的障碍物点云 /cloud_obstacle_new
    //rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap; //发布从ikdtree中获取的初始地图点云 /Laser_map
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped; //发布odomAftMapped到odom_topic
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr plane_pub;
    /// 发送当前位姿
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr current_pose_pub_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::unique_ptr<tf2_ros::TransformBroadcaster> static_broadcaster_;

    int frame_num = 0;
    double aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_propag = 0;
    std::time_t startTime, endTime;
    rclcpp::TimerBase::SharedPtr timer_;
    Eigen::Matrix<double, 24, 24> Q_input;
    Eigen::Matrix<double, 30, 30> Q_output;

    const float MOV_THRESHOLD = 1.5f;
    mutex mtx_buffer;
    condition_variable sig_buffer;
    int feats_down_size = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
    int frame_ct = 0;
    double time_update_last = 0.0, time_current = 0.0, time_predict_last_const = 0.0, t_last = 0.0;
    shared_ptr<ImuProcess> p_imu;
    bool init_map = false, flg_first_scan = true;
    PointCloudXYZI::Ptr  ptr_con;
    double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot11[MAXN];
    double match_time = 0, solve_time = 0, propag_time = 0, update_time = 0;
    bool   lidar_pushed = false, flg_reset = false, flg_exit = false;
    vector<BoxPointType> cub_needrm;

    deque<PointCloudXYZI::Ptr>  lidar_buffer;
    deque<double>               time_buffer;
    deque<sensor_msgs::msg::Imu::SharedPtr> imu_deque;
    PointCloudXYZI::Ptr feats_undistort; //从meas中直接获取的最原始点云NO.1
    PointCloudXYZI::Ptr feats_down_body_space; //
    PointCloudXYZI::Ptr init_feats_world; //不断收集feats_down_world用于初始化地图（ikdtree）的点，世界系 NO.2
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterMap;
    V3D euler_cur;
    MeasureGroup Measures;  //雷达传回的各种最原始的数据
    sensor_msgs::msg::Imu imu_last, imu_next;
    sensor_msgs::msg::Imu::SharedPtr imu_last_ptr;
    nav_msgs::msg::Path path;
    nav_msgs::msg::Odometry odomAftMapped;
    geometry_msgs::msg::PoseStamped msg_body_pose;
    double stamp_;
    sensor_msgs::msg::PointCloud2 pcd_map_;
    PointCloudXYZI::Ptr cloud;
    pcl::PCLPointCloud2 cloudBlob;
    int points_cache_size = 0;
    BoxPointType LocalMap_Points;
    bool Localmap_Initialized = false;
    int process_increments = 0;
    PointCloudXYZI::Ptr pcl_wait_pub; //所有点云相加（可选是否下采样）
    PointCloudXYZI::Ptr pcl_wait_save;
    bool is_first_frame__ = true;
    bool is_first_kf_ = true;
    int sleep_time = 0;
};
