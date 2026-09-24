//
// Created by elsa on 25-4-2.
//

#include "pcd2pgm/pcd2pgm.hpp"

Pcd2Pgm::Pcd2Pgm(std::string name) : Node(name)
{
    RCLCPP_INFO(this->get_logger(), "%s 节点已启动", name.c_str());
    this->declare_parameter("pcd_path", "~");
    this->declare_parameter("pcd_name", "scans.pcd");
    this->declare_parameter("map_path", "~");
    this->declare_parameter("map_name", "map.png");
    this->declare_parameter("yaml_name", "map.yaml");
    this->declare_parameter("thre_z_min", 0.2);
    this->declare_parameter("thre_z_max", 1.0);
    this->declare_parameter("thre_radius", 0.25);
    this->declare_parameter("thres_point_count", 5);
    this->declare_parameter("map_resolution", 0.05);
    this->declare_parameter("thre_x_min", -10.0);
    this->declare_parameter("thre_x_max", 10.0);
    this->declare_parameter("thre_y_min", -10.0);
    this->declare_parameter("thre_y_max", 10.0);
    this->declare_parameter("free_thresh", 0.196);
    this->declare_parameter("occupied_thresh", 0.65);
    this->declare_parameter("point_num_for_normal", 10);
    this->declare_parameter("method", "point");
    this->declare_parameter("leaf_size", 0.02);
    this->declare_parameter("angle_threshold", 0.1);
    this->declare_parameter("height_threshold", 0.1);
    this->declare_parameter("point_num_for_statistical", 100);
    this->declare_parameter("standard_deviation_multiplier", 1.0);

    this->get_parameter("pcd_path", pcd_path);
    this->get_parameter("pcd_name", pcd_name);
    this->get_parameter("map_path", map_path);
    this->get_parameter("map_name", map_name);
    this->get_parameter("yaml_name", yaml_name);
    this->get_parameter("thre_z_min", thre_z_min);
    this->get_parameter("thre_z_max", thre_z_max);
    this->get_parameter("thre_radius", thre_radius);
    this->get_parameter("thres_point_count", thre_point_count);
    this->get_parameter("map_resolution", map_resolution);
    this->get_parameter("thre_x_min", thre_x_min);
    this->get_parameter("thre_x_max", thre_x_max);
    this->get_parameter("thre_y_min", thre_y_min);
    this->get_parameter("thre_y_max", thre_y_max);
    this->get_parameter("free_thresh", free_thresh);
    this->get_parameter("occupied_thresh", occupied_thresh);
    this->get_parameter("point_num_for_normal", point_num_for_normal);
    this->get_parameter("method", method);
    this->get_parameter("leaf_size", leaf_size);
    this->get_parameter("angle_threshold", angle_threshold);
    this->get_parameter("height_threshold", height_threshold);
    this->get_parameter("point_num_for_statistical", point_num_for_normal);
    this->get_parameter("standard_deviation_multiplier", angle_threshold);

    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // 加载pcd文件
    loadPcd();
    // 将点云转换为地图
    convertPointToMap();
    // 保存地图
    saveMap();
}

void Pcd2Pgm::loadPcd()
{
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path + "/" + pcd_name, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Loaded %d data points from test_pcd.pcd with the following fields: %s",
                cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str());
}

void Pcd2Pgm::convertPointToMap()
{
    if (method == "point")
    {
        RCLCPP_INFO(get_logger(), "choose method:point");
        convertPointToMapByPoint();
    }
    else if (method == "normal")
    {
        RCLCPP_INFO(get_logger(), "choose method:normal");
        
        convertPointToMapByNormal();
    }
    else if (method == "height")
    {
        RCLCPP_INFO(get_logger(), "choose method:height");
        
        convertPointToMapByHeight();
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "未知的转换方法");
    }
}

void Pcd2Pgm::passThroughFilter()
{
    RCLCPP_INFO(this->get_logger(), "直通滤波开始");
    // 创建滤波器对象
    pcl::PassThrough<pcl::PointXYZ> passthroughz;
    // 输入点云
    passthroughz.setInputCloud(cloud);
    // 设置对z轴进行操作
    passthroughz.setFilterFieldName("z");
    // 设置滤波范围
    passthroughz.setFilterLimits(thre_z_min, thre_z_max);
    // true表示保留滤波范围外，false表示保留范围内
    passthroughz.setFilterLimitsNegative(false);
    // 执行滤波并存储
    passthroughz.filter(*cloud);

    // 创建滤波器对象
    pcl::PassThrough<pcl::PointXYZ> passthroughx;
    // 输入点云
    passthroughx.setInputCloud(cloud);
    // 设置对x轴进行操作
    passthroughx.setFilterFieldName("x");
    // 设置滤波范围
    passthroughx.setFilterLimits(thre_x_min, thre_x_max);
    // true表示保留滤波范围外，false表示保留范围内
    passthroughx.setFilterLimitsNegative(false);
    // 执行滤波并存储
    passthroughx.filter(*cloud);

    // 创建滤波器对象
    pcl::PassThrough<pcl::PointXYZ> passthroughy;
    // 输入点云
    passthroughy.setInputCloud(cloud);
    // 设置对y轴进行操作
    passthroughy.setFilterFieldName("y");
    // 设置滤波范围
    passthroughy.setFilterLimits(thre_y_min, thre_y_max);
    // true表示保留滤波范围外，false表示保留范围内
    passthroughy.setFilterLimitsNegative(false);
    // 执行滤波并存储
    passthroughy.filter(*cloud);
    // test 保存滤波后的点云到文件
    pcl::io::savePCDFile<pcl::PointXYZ>(pcd_path + "/map_passthrough_filter.pcd", *cloud);
    RCLCPP_INFO(this->get_logger(), "直通滤波结束");
}

void Pcd2Pgm::radiusOutlierFilter()
{
    RCLCPP_INFO(this->get_logger(), "半径滤波开始");
    // 创建滤波器
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;
    // 设置输入点云
    radiusoutlier.setInputCloud(cloud);
    // 设置半径,在该范围内找临近点
    radiusoutlier.setRadiusSearch(thre_radius);
    // 设置查询点的邻域点集数，小于该阈值的删除
    radiusoutlier.setMinNeighborsInRadius(thre_point_count);
    radiusoutlier.filter(*cloud);
    // test 保存滤波后的点云到文件
    pcl::io::savePCDFile<pcl::PointXYZ>(pcd_path + "/map_radius_filter.pcd", *cloud);
    RCLCPP_INFO(this->get_logger(), "半径滤波结束");
}

void Pcd2Pgm::voxelGridFilter()
{
    RCLCPP_INFO(this->get_logger(), "体素滤波开始");
    pcl::VoxelGrid<pcl::PointXYZ> voxfilter;
    voxfilter.setInputCloud(cloud);
    // 设置滤波器的体素大小
    voxfilter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxfilter.filter(*cloud);
    // test 保存滤波后的点云到文件
    pcl::io::savePCDFile<pcl::PointXYZ>(pcd_path + "/map_voxel_filter.pcd", *cloud);
    RCLCPP_INFO(this->get_logger(), "体素滤波结束");
}

void Pcd2Pgm::statisticalOutlierFilter()
{
    RCLCPP_INFO(this->get_logger(), "统计滤波开始");
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statisfilter;
    statisfilter.setInputCloud(cloud);
    // 设置在进行统计时考虑查询点临近点集数
    statisfilter.setMeanK(point_num_for_statistical);
    // 设置标准差倍数
    statisfilter.setStddevMulThresh(standard_deviation_multiplier);
    statisfilter.filter(*cloud);
    // test 保存滤波后的点云到文件
    pcl::io::savePCDFile<pcl::PointXYZ>(pcd_path + "/map_statistical_filter.pcd", *cloud);
    RCLCPP_INFO(this->get_logger(), "统计滤波结束");
}

void Pcd2Pgm::convertPointToMapByPoint()
{
    // 先进行直通滤波，限制点云范围
    passThroughFilter();
    // 再进行半径滤波，去除离群点
    radiusOutlierFilter();
    // 再进行统计滤波，去除离群点
    //statisticalOutlierFilter();

    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map";

    msg.info.map_load_time = this->get_clock()->now();
    msg.info.resolution = map_resolution;

    double x_min, x_max, y_min, y_max;
    // 确定x和y的最大最小值
    for (int i = 0; i < cloud->points.size() - 1; i++)
    {
        if (i == 0)
        {
            x_min = x_max = cloud->points[i].x;
            y_min = y_max = cloud->points[i].y;
        }

        double x = cloud->points[i].x;
        double y = cloud->points[i].y;

        if (x < x_min)
            x_min = x;
        if (x > x_max)
            x_max = x;

        if (y < y_min)
            y_min = y;
        if (y > y_max)
            y_max = y;
    }
    // origin的确定
    msg.info.origin.position.x = x_min;
    msg.info.origin.position.y = y_min;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.x = 0.0;
    msg.info.origin.orientation.y = 0.0;
    msg.info.origin.orientation.z = 0.0;
    msg.info.origin.orientation.w = 1.0;
    // 设置栅格地图大小
    msg.info.width = int((x_max - x_min) / map_resolution);
    msg.info.height = int((y_max - y_min) / map_resolution);
    // 实际地图中某点坐标为(x,y)，对应栅格地图中坐标为[x*map.info.width+y]
    msg.data.resize(msg.info.width * msg.info.height);
    msg.data.assign(msg.info.width * msg.info.height, 0);
    for (int iter = 0; iter < cloud->points.size(); iter++)
    {
        int i = int((cloud->points[iter].x - x_min) / map_resolution);
        if (i < 0 || i >= msg.info.width)
            continue;

        int j = int((cloud->points[iter].y - y_min) / map_resolution);
        if (j < 0 || j >= msg.info.height - 1)
            continue;
        // 栅格地图的占有概率[0,100]，这里设置为占据
        msg.data[i + j * msg.info.width] = 100;
    }
    // 创建法向量估计对象
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    // 创建一个空的kdtree对象，并把它传递给法向量估计对象
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    // 输出数据集
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setKSearch(point_num_for_normal); // 使用最近的point_num_for_normal_个点计算法向量
    ne.compute(*cloud_normals); // 计算法向量

    for (int iter = 0; iter < cloud->points.size(); iter++)
    {
        float gradient = acos(
            sqrt(pow(cloud_normals->points[iter].normal_x, 2) + pow(cloud_normals->points[iter].normal_y, 2)) / sqrt(
                pow(cloud_normals->points[iter].normal_x, 2) + pow(cloud_normals->points[iter].normal_y, 2) + pow(
                    cloud_normals->points[iter].normal_z, 2)));
        // 如果法向量与地面的夹角大于角度阈值
        if (gradient < angle_threshold)
        {
            int i = int((cloud->points[iter].x - x_min) / map_resolution);
            if (i < 0 || i >= msg.info.width)
                continue;

            int j = int((cloud->points[iter].y - y_min) / map_resolution);
            if (j < 0 || j >= msg.info.height - 1)
                continue;
            // 栅格地图的占有概率[0,100]，这里设置为占据
            msg.data[i + j * msg.info.width] = 100;
        }
        else
        {
            int i = int((cloud->points[iter].x - x_min) / map_resolution);
            if (i < 0 || i >= msg.info.width)
                continue;

            int j = int((cloud->points[iter].y - y_min) / map_resolution);
            if (j < 0 || j >= msg.info.height - 1)
                continue;
            msg.data[i + j * msg.info.width] = 0;
        }
    }
    //  string all_points_dir(string(string(ROOT_DIR) + "PCD/" + pcd_file_name + "_") + to_string(pcd_index) + string(".pcd"));
    //         pcl::PCDWriter pcd_writer;
    //         RCLCPP_INFO(this->get_logger(), "current scan saved to /PCD/%s", all_points_dir.c_str());
    //         pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
}

void Pcd2Pgm::convertPointToMapByNormal()
{
    // 先进行直通滤波，限制点云范围
    passThroughFilter();
    // 再进行半径滤波，去除离群点
    radiusOutlierFilter();
    // 再进行统计滤波，去除离群点
    statisticalOutlierFilter();

    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map";

    msg.info.map_load_time = this->get_clock()->now();
    msg.info.resolution = map_resolution;

    double x_min, x_max, y_min, y_max;

    // 确定x和y的最大最小值
    for (int i = 0; i < cloud->points.size() - 1; i++)
    {
        if (i == 0)
        {
            x_min = x_max = cloud->points[i].x;
            y_min = y_max = cloud->points[i].y;
        }

        double x = cloud->points[i].x;
        double y = cloud->points[i].y;

        if (x < x_min)
            x_min = x;
        if (x > x_max)
            x_max = x;

        if (y < y_min)
            y_min = y;
        if (y > y_max)
            y_max = y;
    }
    // origin的确定
    msg.info.origin.position.x = x_min;
    msg.info.origin.position.y = y_min;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.x = 0.0;
    msg.info.origin.orientation.y = 0.0;
    msg.info.origin.orientation.z = 0.0;
    msg.info.origin.orientation.w = 1.0;
    // 设置栅格地图大小
    msg.info.width = int((x_max - x_min) / map_resolution);
    msg.info.height = int((y_max - y_min) / map_resolution);
    // 实际地图中某点坐标为(x,y)，对应栅格地图中坐标为[x*map.info.width+y]
    msg.data.resize(msg.info.width * msg.info.height);
    msg.data.assign(msg.info.width * msg.info.height, 0);

    // 创建法向量估计对象
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    // 创建一个空的kdtree对象，并把它传递给法向量估计对象
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    // 输出数据集
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setKSearch(point_num_for_normal); // 使用最近的point_num_for_normal_个点计算法向量
    ne.compute(*cloud_normals); // 计算法向量

    for (int iter = 0; iter < cloud->points.size(); iter++)
    {
        float gradient = acos(
            sqrt(pow(cloud_normals->points[iter].normal_x, 2) + pow(cloud_normals->points[iter].normal_y, 2)) / sqrt(
                pow(cloud_normals->points[iter].normal_x, 2) + pow(cloud_normals->points[iter].normal_y, 2) + pow(
                    cloud_normals->points[iter].normal_z, 2)));
        // 如果法向量与地面的夹角大于角度阈值
        if (gradient < angle_threshold)
        {
            int i = int((cloud->points[iter].x - x_min) / map_resolution);
            if (i < 0 || i >= msg.info.width)
                continue;

            int j = int((cloud->points[iter].y - y_min) / map_resolution);
            if (j < 0 || j >= msg.info.height - 1)
                continue;
            // 栅格地图的占有概率[0,100]，这里设置为占据
            msg.data[i + j * msg.info.width] = 100;
        }
    }
}

void Pcd2Pgm::convertPointToMapByHeight()
{
    // 先进行直通滤波，限制点云范围
    passThroughFilter();
    // 再进行半径滤波，去除离群点
    // radiusOutlierFilter();
    // 再进行统计滤波，去除离群点
    //statisticalOutlierFilter();

    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map";

    msg.info.map_load_time = this->get_clock()->now();
    msg.info.resolution = map_resolution;

    double x_min, x_max, y_min, y_max;

    // 确定x和y的最大最小值
    for (int i = 0; i < cloud->points.size() - 1; i++)
    {
        if (i == 0)
        {
            x_min = x_max = cloud->points[i].x;
            y_min = y_max = cloud->points[i].y;
        }

        double x = cloud->points[i].x;
        double y = cloud->points[i].y;

        if (x < x_min)
            x_min = x;
        if (x > x_max)
            x_max = x;

        if (y < y_min)
            y_min = y;
        if (y > y_max)
            y_max = y;
    }
    // origin的确定
    msg.info.origin.position.x = x_min;
    msg.info.origin.position.y = y_min;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.x = 0.0;
    msg.info.origin.orientation.y = 0.0;
    msg.info.origin.orientation.z = 0.0;
    msg.info.origin.orientation.w = 1.0;
    // 设置栅格地图大小
    msg.info.width = int((x_max - x_min) / map_resolution);
    msg.info.height = int((y_max - y_min) / map_resolution);
    // 实际地图中某点坐标为(x,y)，对应栅格地图中坐标为[x*map.info.width+y]
    msg.data.resize(msg.info.width * msg.info.height);
    msg.data.assign(msg.info.width * msg.info.height, 0);

    // 新建一个矩阵，用于存储每个栅格的最大高度
    std::vector<std::vector<double>> height_map(msg.info.width, std::vector<double>(msg.info.height, 0.0));
    for (int iter = 0; iter < cloud->points.size(); iter++)
    {
        int i = int((cloud->points[iter].x - x_min) / map_resolution);
        if (i < 0 || i >= msg.info.width)
            continue;

        int j = int((cloud->points[iter].y - y_min) / map_resolution);
        if (j < 0 || j >= msg.info.height - 1)
            continue;
        // 如果当前点的高度大于该栅格的最大高度，则更新该栅格的最大高度
        if (cloud->points[iter].z > height_map[i][j])
        {
            height_map[i][j] = cloud->points[iter].z;
        }
    }
    // 遍历所有栅格，如果与临近栅格的高度差大于阈值，则认为是障碍物
    for (int i = 0; i < msg.info.width; i++)
    {
        for (int j = 0; j < msg.info.height; j++)
        {
            if (i > 0 && j > 0 && i < msg.info.width - 1 && j < msg.info.height - 1)
            {
                if (fabs(height_map[i][j] - height_map[i - 1][j]) > height_threshold ||
                    fabs(height_map[i][j] - height_map[i + 1][j]) > height_threshold ||
                    fabs(height_map[i][j] - height_map[i][j - 1]) > height_threshold || fabs(
                        height_map[i][j] - height_map[i][j + 1]) > height_threshold)
                {
                    msg.data[i + j * msg.info.width] = 100;
                }
            }
        }
    }
}

void Pcd2Pgm::saveMap()
{
    RCLCPP_INFO(this->get_logger(), "开始保存地图...");

    // 检查 msg.info.height 和 msg.info.width 是否有效
    if (msg.info.height <= 0 || msg.info.width <= 0)
    {
        RCLCPP_ERROR(this->get_logger(), "无效的地图尺寸：height=%d, width=%d", msg.info.height, msg.info.width);
        return;
    }

    // 创建 Mat 对象
    cv::Mat map(msg.info.height, msg.info.width, CV_8UC1);

    // 填充 Mat 对象
    for (int i = 0; i < msg.info.height; i++)
    {
        for (int j = 0; j < msg.info.width; j++)
        {
            map.at<uchar>(msg.info.height - i - 1, j) = (100 - msg.data[i * msg.info.width + j]) * 2.55;
        }
    }

    // 检查 Mat 对象是否为空
    if (map.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "生成的地图图像为空");
        return;
    }

    // 保存图像
    if (!cv::imwrite(map_path + "/" + map_name, map))
    {
        RCLCPP_ERROR(this->get_logger(), "无法保存地图图像到 %s", (map_path + "/" + map_name).c_str());
        return;
    }

    // 创建 YAML 文件
    std::ofstream yaml_file;
    yaml_file.open(map_path + "/" + yaml_name);
    if (!yaml_file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "无法创建 YAML 文件 %s", (map_path + "/" + yaml_name).c_str());
        return;
    }

    yaml_file << "image: " << map_name << std::endl;
    yaml_file << "mode: trinary" << std::endl;
    yaml_file << "resolution: " << map_resolution << std::endl;
    yaml_file << "origin: [" << msg.info.origin.position.x << ", " << msg.info.origin.position.y << ", " << msg.info.
        origin.position.z << "]" << std::endl;
    yaml_file << "negate: 0" << std::endl;
    yaml_file << "occupied_thresh: " << occupied_thresh << std::endl;
    yaml_file << "free_thresh: " << free_thresh << std::endl;
    yaml_file.close();

    RCLCPP_INFO(this->get_logger(), "地图保存成功");
}
