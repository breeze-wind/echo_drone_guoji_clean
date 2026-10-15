#include "obstacle_segmentation/obstacle_segmentation.hpp"

#include <cmath>
#include <limits>
#include <algorithm>
#include <stdexcept>

namespace
{
using CellKey = int64_t;

CellKey makeCellKey(int x, int y)
{
    const auto ux = static_cast<uint32_t>(x);
    const auto uy = static_cast<uint32_t>(y);
    return static_cast<CellKey>((static_cast<uint64_t>(ux) << 32) | uy);
}

template <typename CellPredicate>
bool rayHasMatchingCell(double x0, double y0, double x1, double y1,
                       double resolution, CellPredicate&& matches)
{
    int x = static_cast<int>(std::floor(x0 / resolution));
    int y = static_cast<int>(std::floor(y0 / resolution));
    const int end_x = static_cast<int>(std::floor(x1 / resolution));
    const int end_y = static_cast<int>(std::floor(y1 / resolution));
    const int dx = std::abs(end_x - x);
    const int sx = x < end_x ? 1 : -1;
    const int dy = -std::abs(end_y - y);
    const int sy = y < end_y ? 1 : -1;
    int error = dx + dy;
    while (x != end_x || y != end_y)
    {
        if (matches(makeCellKey(x, y)))
        {
            return true;
        }
        const int twice_error = 2 * error;
        if (twice_error >= dy)
        {
            error += dy;
            x += sx;
        }
        if (twice_error <= dx)
        {
            error += dx;
            y += sy;
        }
    }
    return false;
}

std::pair<int, int> cellFromKey(CellKey key)
{
    const auto packed = static_cast<uint64_t>(key);
    const int x = static_cast<int32_t>(packed >> 32);
    const int y = static_cast<int32_t>(static_cast<uint32_t>(packed));
    return {x, y};
}
}  // namespace

ObstacleSegmentationNode::ObstacleSegmentationNode(std::string name, const rclcpp::NodeOptions& options)
    : Node(name, options)
{
    RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
    // 声明参数
    this->declare_parameter("input_cloud_topic", "input_cloud");
    this->declare_parameter("output_cloud_topic", "output_cloud");
    this->declare_parameter("clearing_scan_topic", "/scan_obstacle_clear");
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
    this->declare_parameter("obstacle_above_sensor_max_m", 0.4);
    this->declare_parameter("obstacle_range_min", 0.5);
    this->declare_parameter("obstacle_range_max", 2.0);
    this->declare_parameter("body_min_x", -0.3);
    this->declare_parameter("body_max_x", 0.3);
    this->declare_parameter("body_min_y", -0.25);
    this->declare_parameter("body_max_y", 0.25);
    this->declare_parameter("body_min_z", -0.25);
    this->declare_parameter("body_max_z", 0.15);
    this->declare_parameter("use_downsample", true);
    this->declare_parameter("obstacle_cell_stale_timeout_sec", 1.0);
    this->declare_parameter("clear_ray_max_range", 6.0);
    this->declare_parameter("clear_grid_resolution", 0.05);
    this->declare_parameter("tf_wait_timeout_sec", 3.0);
    this->declare_parameter("clearing_max_cloud_age_sec", 0.5);
    this->declare_parameter("max_pending_clouds", 40);

    RCLCPP_INFO(this->get_logger(), "点云分割节点初始化");
    this->get_parameter("input_cloud_topic", input_cloud_topic_);
    this->get_parameter("output_cloud_topic", output_cloud_topic_);
    this->get_parameter("clearing_scan_topic", clearing_scan_topic_);
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
    this->get_parameter("obstacle_above_sensor_max_m", obstacle_above_sensor_max_m_);
    this->get_parameter("obstacle_range_min", obstacle_range_min_);
    this->get_parameter("obstacle_range_max", obstacle_range_max_);
    this->get_parameter("body_min_x", body_min_x_);
    this->get_parameter("body_max_x", body_max_x_);
    this->get_parameter("body_min_y", body_min_y_);
    this->get_parameter("body_max_y", body_max_y_);
    this->get_parameter("body_min_z", body_min_z_);
    this->get_parameter("body_max_z", body_max_z_);
    this->get_parameter("use_downsample", use_downsample_);
    this->get_parameter("obstacle_cell_stale_timeout_sec", obstacle_cell_stale_timeout_sec_);
    this->get_parameter("clear_ray_max_range", clear_ray_max_range_);
    this->get_parameter("clear_grid_resolution", clear_grid_resolution_);
    this->get_parameter("tf_wait_timeout_sec", tf_wait_timeout_sec_);
    this->get_parameter("clearing_max_cloud_age_sec", clearing_max_cloud_age_sec_);
    this->get_parameter("max_pending_clouds", max_pending_clouds_);
    tf_wait_timeout_sec_ = std::max(0.0, tf_wait_timeout_sec_);
    clearing_max_cloud_age_sec_ = std::max(0.0, clearing_max_cloud_age_sec_);
    max_pending_clouds_ = std::max(1, max_pending_clouds_);
    obstacle_above_sensor_max_m_ = std::max(0.0, obstacle_above_sensor_max_m_);
    if (body_min_x_ > body_max_x_ || body_min_y_ > body_max_y_ || body_min_z_ > body_max_z_)
    {
        throw std::runtime_error("Invalid self-filter bounds: each body_min_* must be <= body_max_*");
    }
    last_queue_stats_time_ = std::chrono::steady_clock::now();

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
    clearing_cloud_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(clearing_scan_topic_, 10);
    input_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_cloud_topic_, 10, std::bind(&ObstacleSegmentationNode::cloudCallback, this, std::placeholders::_1));
    clear_state_sub_ = this->create_subscription<std_msgs::msg::Bool>("/robot/clear_state",
            10, std::bind(&ObstacleSegmentationNode::ClearStateCallback, this, std::placeholders::_1));
    obstacle_height_sub_ = this->create_subscription<std_msgs::msg::Float64>("/robot/obstacle_height", 10,
        std::bind(&ObstacleSegmentationNode::ObstacleHeightCallback, this, std::placeholders::_1));
    pending_cloud_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&ObstacleSegmentationNode::processPendingClouds, this));
    RCLCPP_INFO(this->get_logger(), "点云分割节点初始化完成");
    RCLCPP_INFO(this->get_logger(),
        "Obstacle cell expiry enabled: clear cells absent from obstacle cloud for %.1f s; range <= %.1f m",
        obstacle_cell_stale_timeout_sec_, clear_ray_max_range_);
    RCLCPP_INFO(this->get_logger(),
        "Timestamp-aligned cloud queue: max_wait=%.2fs max_queue=%d; stale scans do not emit clearing rays after %.2fs",
        tf_wait_timeout_sec_, max_pending_clouds_, clearing_max_cloud_age_sec_);
    RCLCPP_INFO(this->get_logger(),
        "Ignore obstacle returns at or above sensor height + %.2f m",
        obstacle_above_sensor_max_m_);
    RCLCPP_INFO(this->get_logger(),
        "Self-point exclusion enabled in input cloud frame: x=[%.2f, %.2f], y=[%.2f, %.2f], z=[%.2f, %.2f] m",
        body_min_x_, body_max_x_, body_min_y_, body_max_y_, body_min_z_, body_max_z_);
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

    const int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
    if (stamp_ns <= last_enqueued_stamp_ns_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Dropping out-of-order cloud stamp=%ld; last enqueued stamp=%ld",
            static_cast<long>(stamp_ns), static_cast<long>(last_enqueued_stamp_ns_));
        return;
    }
    last_enqueued_stamp_ns_ = stamp_ns;

    if (static_cast<int>(pending_clouds_.size()) >= max_pending_clouds_)
    {
        pending_clouds_.pop_front();
        ++queue_overflow_drop_count_;
        // Never combine data across a queue gap; that would republish old hits
        // as if they belonged to a newer scan.
        segement_cloud->clear();
        pcl_cnt = 0;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Pending TF queue full; dropped oldest cloud (overflow drops=%lu, queue limit=%d)",
            static_cast<unsigned long>(queue_overflow_drop_count_), max_pending_clouds_);
    }
    pending_clouds_.push_back(PendingCloud{msg, std::chrono::steady_clock::now()});
}

void ObstacleSegmentationNode::processPendingClouds()
{
    // Never burst-process a backlog. One scan per timer tick keeps CPU use
    // bounded and lets the input queue's drop-oldest policy shed stale work.
    while (!pending_clouds_.empty())
    {
        const auto& pending = pending_clouds_.front();
        geometry_msgs::msg::TransformStamped cloud_to_map;
        try
        {
            cloud_to_map = tfbuffer_->lookupTransform(
                output_frame_, pending.msg->header.frame_id,
                rclcpp::Time(pending.msg->header.stamp), rclcpp::Duration::from_seconds(0.0));
        }
        catch (const tf2::TransformException& ex)
        {
            const double wait_sec = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - pending.queued_at).count();
            if (wait_sec < tf_wait_timeout_sec_)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Waiting for TF %s <- %s at queued cloud stamp; pending=%zu wait=%.2fs: %s",
                    output_frame_.c_str(), pending.msg->header.frame_id.c_str(),
                    pending_clouds_.size(), wait_sec, ex.what());
                break;  // Preserve timestamp order; do not process newer scans first.
            }

            pending_clouds_.pop_front();
            ++tf_timeout_drop_count_;
            segement_cloud->clear();
            pcl_cnt = 0;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Dropping cloud after TF wait timeout %.2fs (timeout drops=%lu, pending=%zu); last error: %s",
                wait_sec, static_cast<unsigned long>(tf_timeout_drop_count_),
                pending_clouds_.size(), ex.what());
            continue;
        }

        const auto msg = pending.msg;
        const double wait_sec = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - pending.queued_at).count();
        const int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
        pending_clouds_.pop_front();
        if (stamp_ns <= last_processed_stamp_ns_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Dropping cloud processed out of order (stamp=%ld last=%ld)",
                static_cast<long>(stamp_ns), static_cast<long>(last_processed_stamp_ns_));
            segement_cloud->clear();
            pcl_cnt = 0;
            continue;
        }
        last_processed_stamp_ns_ = stamp_ns;
        ++tf_resolved_count_;
        tf_wait_total_sec_ += wait_sec;
        tf_wait_max_sec_ = std::max(tf_wait_max_sec_, wait_sec);
        processCloud(msg, cloud_to_map);
        break;
    }

    const auto stats_now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(stats_now - last_queue_stats_time_).count() >= 5.0)
    {
        const double average_wait = tf_resolved_count_ > 0
            ? tf_wait_total_sec_ / static_cast<double>(tf_resolved_count_) : 0.0;
        RCLCPP_INFO(this->get_logger(),
            "TF queue stats (5s): processed=%lu avg_wait=%.3fs max_wait=%.3fs pending=%zu timeout_drops=%lu overflow_drops=%lu",
            static_cast<unsigned long>(tf_resolved_count_), average_wait, tf_wait_max_sec_,
            pending_clouds_.size(), static_cast<unsigned long>(tf_timeout_drop_count_),
            static_cast<unsigned long>(queue_overflow_drop_count_));
        tf_resolved_count_ = 0;
        tf_wait_total_sec_ = 0.0;
        tf_wait_max_sec_ = 0.0;
        last_queue_stats_time_ = stats_now;
    }
}

void ObstacleSegmentationNode::processCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
    const geometry_msgs::msg::TransformStamped& cloud_to_map)
{

    // 将点云转换为pcl格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // /cloud_registered_body is expressed in Point-LIO's body/IMU frame.
    // Remove only the compact volume occupied by the airframe before applying
    // the dynamic body-to-map transform; nearby external obstacles outside
    // this 3-D box remain available to the costmap.
    const size_t input_point_count = cloud->points.size();
    const auto inside_self_volume = [this](const pcl::PointXYZ& point)
    {
        return
            point.x >= body_min_x_ && point.x <= body_max_x_ &&
            point.y >= body_min_y_ && point.y <= body_max_y_ &&
            point.z >= body_min_z_ && point.z <= body_max_z_;
    };
    cloud->points.erase(
        std::remove_if(cloud->points.begin(), cloud->points.end(), inside_self_volume),
        cloud->points.end());
    const size_t self_point_count = input_point_count - cloud->points.size();
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Self-point filter: removed %zu/%zu points inside body box in frame '%s'",
        self_point_count, input_point_count, msg->header.frame_id.c_str());

    const Eigen::Matrix4f cloud_to_map_matrix =
        tf2::transformToEigen(cloud_to_map).matrix().cast<float>();
    const Eigen::Matrix3f map_to_cloud_rotation =
        cloud_to_map_matrix.block<3, 3>(0, 0).transpose();
    pcl::transformPointCloud(*cloud, *cloud, cloud_to_map_matrix);
    // Track the last time each obstacle-map cell appeared in the exact cloud
    // that feeds /cloud_obstacle. Cells unseen for the configured timeout are
    // sent as clearing-ray endpoints to Nav2.
    const auto now = std::chrono::steady_clock::now();
    const int64_t now_ros_ns = this->now().nanoseconds();
    const int64_t cloud_stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
    const double cloud_age_sec = std::max(
        0.0, static_cast<double>(now_ros_ns - cloud_stamp_ns) * 1e-9);
    const auto observation_time = now - std::chrono::duration_cast<SteadyTime::duration>(
        std::chrono::duration<double>(cloud_age_sec));
    constexpr auto kClearingMaintenancePeriod = std::chrono::milliseconds(200);
    const bool run_clearing_maintenance = last_clearing_maintenance_time_ == SteadyTime{} ||
        now - last_clearing_maintenance_time_ >= kClearingMaintenancePeriod;
    if (run_clearing_maintenance)
    {
        last_clearing_maintenance_time_ = now;
    }
    const double sensor_x = cloud_to_map.transform.translation.x;
    const double sensor_y = cloud_to_map.transform.translation.y;
    const double sensor_z = cloud_to_map.transform.translation.z;
    const double max_obstacle_z = std::min<double>(obstacle_z_max_, obstacle_height_);
    const double overhead_cutoff_z = sensor_z + obstacle_above_sensor_max_m_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr frame_obstacles(new pcl::PointCloud<pcl::PointXYZ>);

    pcl_cnt++;

    for (const auto& point : cloud->points)
    {
        if (!std::isfinite(point.x) ||
            !std::isfinite(point.y) ||
            !std::isfinite(point.z))
        {
            continue;
        }
        if (point.z < obstacle_z_min_ || point.z > max_obstacle_z ||
            point.z >= overhead_cutoff_z)
        {
            continue;
        }
        segement_cloud->points.push_back(point);
        frame_obstacles->points.push_back(point);
    }

    constexpr int kClearingScanBins = 720;
    constexpr double kPi = 3.14159265358979323846;
    const float angle_min = static_cast<float>(-kPi);
    const float angle_increment = static_cast<float>(2.0 * kPi / kClearingScanBins);
    std::vector<float> nearest_current_range;
    if (run_clearing_maintenance)
    {
        nearest_current_range.assign(
            kClearingScanBins, std::numeric_limits<float>::infinity());
    }

    // Refresh last-seen times for cells present in this scan and remember
    // current returns so a stale-cell ray cannot clear through a nearer hit.
    for (const auto& point : frame_obstacles->points)
    {
        const Eigen::Vector3f delta_map(point.x - sensor_x, point.y - sensor_y, 0.0F);
        const Eigen::Vector3f delta_sensor =
            map_to_cloud_rotation * delta_map;
        const double range = std::hypot(delta_sensor.x(), delta_sensor.y());
        // Cells outside clearing range can never produce a clearing ray, so
        // keeping them in the historical cell table only wastes scan time.
        if (range > clear_ray_max_range_)
        {
            continue;
        }
        const CellKey key = makeCellKey(
            static_cast<int>(std::floor(point.x / clear_grid_resolution_)),
            static_cast<int>(std::floor(point.y / clear_grid_resolution_)));
        auto& state = obstacle_cell_states_[key];
        // Age obstacle cells from the scan's observation time, not its delayed
        // processing time, so queued scans cannot resurrect stale obstacles.
        state.last_seen = observation_time;
        state.clear_until = SteadyTime{};

        if (!run_clearing_maintenance)
        {
            continue;
        }
        double angle = std::atan2(delta_sensor.y(), delta_sensor.x());
        if (angle >= kPi)
        {
            angle -= 2.0 * kPi;
        }
        const int bin = static_cast<int>(std::floor((angle - angle_min) / angle_increment));
        if (range >= 0.1 && range <= clear_ray_max_range_ &&
            bin >= 0 && bin < kClearingScanBins)
        {
            nearest_current_range[bin] = std::min(
                nearest_current_range[bin], static_cast<float>(range));
        }
    }

    if (run_clearing_maintenance)
    {
    sensor_msgs::msg::LaserScan clearing_scan;
    clearing_scan.header = msg->header;
    clearing_scan.angle_min = angle_min;
    clearing_scan.angle_max = static_cast<float>(kPi);
    clearing_scan.angle_increment = angle_increment;
    clearing_scan.time_increment = 0.0F;
    clearing_scan.scan_time = 0.2F;
    clearing_scan.range_min = 0.1F;
    clearing_scan.range_max = static_cast<float>(clear_ray_max_range_);
    clearing_scan.ranges.assign(kClearingScanBins, std::numeric_limits<float>::infinity());
    for (auto state = obstacle_cell_states_.begin(); state != obstacle_cell_states_.end();)
    {
        const double unseen_sec = std::chrono::duration<double>(now - state->second.last_seen).count();
        if (unseen_sec < obstacle_cell_stale_timeout_sec_)
        {
            ++state;
            continue;
        }

        const auto cell = cellFromKey(state->first);
        const double cell_x = (cell.first + 0.5) * clear_grid_resolution_;
        const double cell_y = (cell.second + 0.5) * clear_grid_resolution_;
        const Eigen::Vector3f delta_map(cell_x - sensor_x, cell_y - sensor_y, 0.0F);
        const Eigen::Vector3f delta_sensor = map_to_cloud_rotation * delta_map;
        const double range = std::hypot(delta_sensor.x(), delta_sensor.y());
        double angle = std::atan2(delta_sensor.y(), delta_sensor.x());
        if (angle >= kPi)
        {
            angle -= 2.0 * kPi;
        }
        const int bin = static_cast<int>(std::floor((angle - angle_min) / angle_increment));
        const bool in_range = range >= clearing_scan.range_min && range <= clearing_scan.range_max;
        const bool not_occluded = bin >= 0 && bin < kClearingScanBins &&
            nearest_current_range[bin] + clear_grid_resolution_ >= range;
        bool path_has_fresh_obstacle = false;
        if (in_range && not_occluded)
        {
            path_has_fresh_obstacle = rayHasMatchingCell(
                sensor_x, sensor_y, cell_x, cell_y, clear_grid_resolution_,
                [&](CellKey path_cell)
                {
                    const auto path_state = obstacle_cell_states_.find(path_cell);
                    return path_state != obstacle_cell_states_.end() &&
                        std::chrono::duration<double>(now - path_state->second.last_seen).count() <
                            obstacle_cell_stale_timeout_sec_;
                });
        }
        if (in_range && not_occluded && !path_has_fresh_obstacle)
        {
            // A one-second repeat window makes the clear command robust to
            // Nav2's lower costmap update rate and avoids retaining stale cells.
            if (state->second.clear_until == SteadyTime{})
            {
                state->second.clear_until = now + std::chrono::seconds(1);
            }
            if (now <= state->second.clear_until)
            {
                float& ray_range = clearing_scan.ranges[bin];
                if (!std::isfinite(ray_range) || range > ray_range)
                {
                    ray_range = static_cast<float>(range);
                }
            }
        }
        if ((state->second.clear_until != SteadyTime{} && now > state->second.clear_until) ||
            unseen_sec > 60.0)
        {
            state = obstacle_cell_states_.erase(state);
            continue;
        }
        ++state;
    }
    if (cloud_age_sec <= clearing_max_cloud_age_sec_)
    {
        clearing_cloud_pub_->publish(clearing_scan);
    }
    else
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Suppressing clearing rays from stale cloud (age=%.2fs > %.2fs); obstacle marking remains enabled",
            cloud_age_sec, clearing_max_cloud_age_sec_);
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
