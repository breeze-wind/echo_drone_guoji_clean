//
// Created by elsa on 25-4-2.
//

#ifndef PCD2PGM_HPP
#define PCD2PGM_HPP

#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/filters/conditional_removal.h>         //条件滤波器头文件
#include <pcl/filters/passthrough.h>                 //直通滤波器头文件
#include <pcl/filters/radius_outlier_removal.h>      //半径滤波器头文件
#include <pcl/filters/statistical_outlier_removal.h> //统计滤波器头文件
#include <pcl/filters/voxel_grid.h>                  //体素滤波器头文件
#include <pcl/features/normal_3d.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/opencv.hpp>

class Pcd2Pgm : public rclcpp::Node
{
public:
    explicit Pcd2Pgm(std::string name);

private:
    std::string pcd_path, pcd_name, map_path, map_name, yaml_name;
    double thre_z_min, thre_z_max, thre_x_min, thre_x_max, thre_y_min, thre_y_max, thre_radius, map_resolution, free_thresh, occupied_thresh, leaf_size, angle_threshold,height_threshold, standard_deviation_multiplier;
    int thre_point_count, point_num_for_normal, point_num_for_statistical;
    std::string method;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

    nav_msgs::msg::OccupancyGrid msg;

    void loadPcd();

    void convertPointToMap();

    // 直通滤波器对点云进行过滤，获取设定高度范围内的数据
    void passThroughFilter();

    // 半径滤波
    void radiusOutlierFilter();

    // 体素滤波
    void voxelGridFilter();

    // 统计滤波
    void statisticalOutlierFilter();

    // 通过判断有没有点将点转换为地图
    // 需要通过调整阈值将天花板和地面排除
    void convertPointToMapByPoint();

    // 通过判断法向量的大小将点转换为地图
    // 不需要将天花板和地面排除，z的范围可适当扩大
    void convertPointToMapByNormal();

    void convertPointToMapByHeight();

    // 将占据地图保存为图片和yaml文件
    void saveMap();

};

#endif //PCD2PGM_HPP
