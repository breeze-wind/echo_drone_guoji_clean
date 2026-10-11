//
// Created by elsa on 25-4-2.
//

#include "merge_pcd/merge_pcd.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <vector>

MergePCD::MergePCD() : Node("merge_pcd_node")
{
    RCLCPP_INFO(this->get_logger(), "merge_pcd_node start");

    this->declare_parameter("pcd_folder", "/home/tc/Desktop/auto_sentry2025/src/auto_sentry2025/Point-LIO/PCD/merge");
    this->declare_parameter("output_file", "");

    this->get_parameter("pcd_folder", pcd_folder_);

    merge_pcd();
}

void MergePCD::merge_pcd()
{
    if (!std::filesystem::exists(pcd_folder_) || !std::filesystem::is_directory(pcd_folder_))
    {
        RCLCPP_ERROR(this->get_logger(), "PCD input directory does not exist: %s", pcd_folder_.c_str());
        return;
    }

    // Sort inputs for repeatable merge order and never re-ingest previous products.
    std::vector<std::filesystem::path> input_files;
    for (const auto &entry : std::filesystem::directory_iterator(pcd_folder_))
    {
        const auto filename = entry.path().filename().string();
        if (entry.is_regular_file() && entry.path().extension() == ".pcd" &&
            filename.find("_merged.pcd") == std::string::npos &&
            filename.find("merged_for_map_") != 0)
        {
            input_files.push_back(entry.path());
        }
    }
    std::sort(input_files.begin(), input_files.end());
    if (input_files.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "No raw .pcd files found in %s", pcd_folder_.c_str());
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &file : input_files)
    {
        RCLCPP_INFO(this->get_logger(), "Loading %s", file.c_str());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file.string(), *cloud_temp) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read file %s", file.c_str());
            return;
        }
        *cloud += *cloud_temp;
    }

    std::string output_file;
    this->get_parameter("output_file", output_file);
    if (output_file.empty())
    {
        const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::tm local_time{};
        localtime_r(&now, &local_time);
        std::ostringstream filename;
        filename << std::put_time(&local_time, "%Y_%m_%d_%H_%M_%S") << "_merged.pcd";
        output_file = (std::filesystem::path(pcd_folder_) / filename.str()).string();
    }

    if (pcl::io::savePCDFileASCII(output_file, *cloud) != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to save merged PCD: %s", output_file.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Merged %zu source PCDs (%zu points); saved %s",
                input_files.size(), cloud->size(), output_file.c_str());
}
