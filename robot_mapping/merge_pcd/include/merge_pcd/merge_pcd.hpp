//
// Created by elsa on 25-4-2.
//

#ifndef MERGE_PCD_HPP
#define MERGE_PCD_HPP

#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <filesystem>

class MergePCD : public rclcpp::Node
{
public:
    explicit MergePCD();
    void merge_pcd();

private:
    std::string pcd_folder_;
};

#endif //MERGE_PCD_HPP
