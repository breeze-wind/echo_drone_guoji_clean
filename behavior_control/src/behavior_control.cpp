//
// 由 elsa 于 25-7-4 创建。
//

#include "behavior_control/behavior_control.hpp"

#include <cmath>

namespace
{
// 110 作为台架测试截断步骤：确认达到起飞高度后，只持续发布圆周目标，
// 不再进入旧的搜索、投掷或穿门流程。
constexpr int kTakeoffCircleStep = 110;
constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kMinCircleRadius = 0.05;
constexpr double kMinCircleSpeed = 0.01;
}

BehaviorControl::BehaviorControl(std::string name) : Node("behavior_control")
{
    current_step = 1;
    RCLCPP_INFO(this->get_logger(), "%s node create", name.c_str());
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    /* 读yaml文件参数 */
    this->declare_parameter<std::vector<double>>("tank_position", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("tent_position", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("car_position", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("pillbox_position", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("bridge_position", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("passing_door_src_1", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("passing_door_src_2", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("passing_door_des", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("random_target_search_1", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("random_target_search_2", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("random_target_search_3", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("random_target_init_search_1", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("random_target_init_search_2", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("prev_random_target", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("prev_random_tank_target", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<std::string>>("target_sequence", std::vector<std::string>{"tent", "car", "pillbox", "tank"});
    this->declare_parameter("cruise_height", 0.6);
    this->declare_parameter("detection_height", 1.5);
    this->declare_parameter("H_detection_height", 1.5);
    this->declare_parameter("passing_door_height", 0.4);
    this->declare_parameter("eject_height", 0.25);
    this->declare_parameter("dynamic_eject_height", 0.5);
    this->declare_parameter("if_hit_tank", false);
    this->declare_parameter("if_hit_car", false);
    this->declare_parameter("if_hit_pillbox", false);
    this->declare_parameter("if_hit_tent", false);
    this->declare_parameter("if_hit_bridge", false);
    this->declare_parameter("if_passing_door", false);
    this->declare_parameter("if_need_passing_all", false);
    this->declare_parameter("offset_x_1", 0.1);
    this->declare_parameter("offset_y_1", 0.1);
    this->declare_parameter("offset_x_2", 0.1);
    this->declare_parameter("offset_y_2", 0.1);
    this->declare_parameter("offset_x_3", 0.1);
    this->declare_parameter("offset_y_3", 0.1);
    this->declare_parameter<int>("/servo/servo", 0);
    this->declare_parameter("max_vel_x_passing", 0.6);
    this->declare_parameter("max_vel_y_passing", 1.5);
    this->declare_parameter("max_vel_x_backwards_passing", 0.4);
    this->declare_parameter("max_vel_theta_passing", 0.25);
    this->declare_parameter("acc_lim_x_passing", 1.5);
    this->declare_parameter("acc_lim_y_passing", 0.4);
    this->declare_parameter("acc_lim_theta_passing", 0.25);
    this->declare_parameter("max_global_plan_lookahead_dist", 1.25);
    this->declare_parameter("weight_inflation", 1.0);
    this->declare_parameter("robot_radius", 0.05);
    this->declare_parameter("takeoff_circle_enabled", true);
    this->declare_parameter("takeoff_circle_radius", 0.5);
    this->declare_parameter("takeoff_circle_speed", 0.2);
    this->declare_parameter("takeoff_circle_direction", 1.0);

    this->get_parameter<std::vector<double>>("tank_position", tank_);
    this->get_parameter<std::vector<double>>("tent_position", tent_);
    this->get_parameter<std::vector<double>>("car_position", car_);
    this->get_parameter<std::vector<double>>("pillbox_position", pillbox_);
    this->get_parameter<std::vector<double>>("bridge_position", bridge_);
    this->get_parameter<std::vector<double>>("passing_door_src_1", passing_door_src_1_);
    this->get_parameter<std::vector<double>>("passing_door_src_2", passing_door_src_2_);
    this->get_parameter<std::vector<double>>("passing_door_des", passing_door_des_);
    this->get_parameter<std::vector<std::string>>("target_sequence", target_sequence_);
    this->get_parameter<std::vector<double>>("random_target_search_1", random_target_search_1_);
    this->get_parameter<std::vector<double>>("random_target_search_2", random_target_search_2_);
    this->get_parameter<std::vector<double>>("random_target_search_3", random_target_search_3_);
    this->get_parameter<std::vector<double>>("random_target_init_search_1", random_target_init_search_1_);
    this->get_parameter<std::vector<double>>("random_target_init_search_2", random_target_init_search_2_);
    this->get_parameter<std::vector<double>>("prev_random_target", prev_random_target_);
    this->get_parameter<std::vector<double>>("prev_random_tank_target", prev_random_tank_target_);
    this->get_parameter("cruise_height", cruise_height_);
    this->get_parameter("detection_height", detection_height_);
    this->get_parameter("H_detection_height", H_detection_height_);
    this->get_parameter("passing_door_height", passing_door_height_);
    this->get_parameter("eject_height", eject_height_);
    this->get_parameter("dynamic_eject_height", dynamic_eject_height_);
    this->get_parameter("if_hit_tank", if_hit_tank_);
    this->get_parameter("if_hit_car", if_hit_car_);
    this->get_parameter("if_hit_pillbox", if_hit_pillbox_);
    this->get_parameter("if_hit_tent", if_hit_tent_);
    this->get_parameter("if_hit_bridge", if_hit_bridge_);
    this->get_parameter("if_passing_door", if_passing_door_);
    this->get_parameter("if_need_passing_all", if_need_passing_all_);
    this->get_parameter("offset_x_1", offset_x_1_);
    this->get_parameter("offset_y_1", offset_y_1_);
    this->get_parameter("offset_x_2", offset_x_2_);
    this->get_parameter("offset_y_2", offset_y_2_);
    this->get_parameter("offset_x_3", offset_x_3_);
    this->get_parameter("offset_y_3", offset_y_3_);
    this->get_parameter("max_vel_x_passing", max_vel_x_passing);
    this->get_parameter("max_vel_y_passing", max_vel_y_passing);
    this->get_parameter("max_vel_x_backwards_passing", max_vel_x_backwards_passing);
    this->get_parameter("max_vel_theta_passing", max_vel_theta_passing);
    this->get_parameter("acc_lim_x_passing", acc_lim_x_passing);
    this->get_parameter("acc_lim_y_passing", acc_lim_y_passing);
    this->get_parameter("acc_lim_theta_passing", acc_lim_theta_passing);
    this->get_parameter("max_global_plan_lookahead_dist", max_global_plan_lookahead_dist);
    this->get_parameter("weight_inflation", weight_inflation);
    this->get_parameter("robot_radius", robot_radius);
    this->get_parameter("takeoff_circle_enabled", takeoff_circle_enabled_);
    this->get_parameter("takeoff_circle_radius", takeoff_circle_radius_);
    this->get_parameter("takeoff_circle_speed", takeoff_circle_speed_);
    this->get_parameter("takeoff_circle_direction", takeoff_circle_direction_);
    takeoff_circle_direction_ = takeoff_circle_direction_ >= 0.0 ? 1.0 : -1.0;

    if (!if_hit_tank_)
    {
        tank_[0] = 0.0;
        tank_[1] = 0.0;
    }

    random_target_.resize(2);
    random_tank_target_.resize(2);
    detected_target_.resize(2);

    RCLCPP_INFO(this->get_logger(), "tank_position: %lf, %lf", tank_[0], tank_[1]);
    RCLCPP_INFO(this->get_logger(), "tent_position: %lf, %lf", tent_[0], tent_[1]);
    RCLCPP_INFO(this->get_logger(), "car_position: %lf, %lf", car_[0], car_[1]);
    RCLCPP_INFO(this->get_logger(), "pillbox_position: %lf, %lf", pillbox_[0], pillbox_[1]);
    RCLCPP_INFO(this->get_logger(), "bridge_position: %lf, %lf", bridge_[0], bridge_[1]);

    // target_positions_["tank"] = tank_;
    target_positions_["tent"] = tent_;
    target_positions_["car"] = car_;
    target_positions_["pillbox"] = pillbox_;
    target_positions_["bridge"] = bridge_;
    if_hit_target_["tank"] = if_hit_tank_;
    if_hit_target_["tent"] = if_hit_tent_;
    if_hit_target_["car"] = if_hit_car_;
    if_hit_target_["pillbox"] = if_hit_pillbox_;
    if_hit_target_["bridge"] = if_hit_bridge_;

    current_passing_door_ = false;
    if_turning = false;
    if_find_random_target_ = false;
    if_find_random_tank_target_ = false;

    current_x_ = 0.0;
    current_y_ = 0.0;
    current_z_ = 0.0;
    dynamic_detection_height_ = detection_height_ + 0.35;

    eject_cnt = 0;
    detection_cnt = 0;
    turning_cnt = 0;
    passing_cnt_1_ = 0;
    passing_cnt_2_ = 0;
    eject_cnt_threshold_ = 4.0 / 0.25; //等待投掷时间
    dynamic_eject_cnt_threshold_ = 1.5 / 0.25; //动态靶等待投掷时间
    detection_cnt_threshold_ = 2.5 / 0.25; //等待识别时间
    dynamic_detection_cnt_threshold_ = 5.0 / 0.25; //在初始起飞后寻找随机靶的等待时间
    turning_cnt_threshold_ = 5.0 / 0.25; //等待转向时间

    obstacle_height_ = 2.0;

    servo_index_ = 0;
    last_servo_index_ = 0;
    current_nav_mode = 0;
    last_nav_mode = 0;
    eject_last_x = 0.0;
    eject_last_y = 0.0;
    arming_state = false;
    if_landing = false;
    if_nav = false;

    is_tank_or_bridge_ = false;
    takeoff_circle_started_ = false;
    takeoff_circle_center_x_ = 0.0;
    takeoff_circle_center_y_ = 0.0;

    current_target_position_.transform.rotation.x = 0.0;
    current_target_position_.transform.rotation.y = 0.0;
    current_target_position_.transform.rotation.z = 0.0;
    current_target_position_.transform.rotation.w = 1.0;

    map_frame_ = "map";
    camera_frame_ = "camera_link";
    target_frame_ = "target_position";

    livox_to_camera = tf_buffer_->lookupTransform("livox", "camera_link", rclcpp::Time(),
                                    rclcpp::Duration::from_seconds(0.5));
    // RCLCPP_INFO(this->get_logger(), "-----------------+++++++++++++++livox_to_camera x y z: %lf, %lf, %lf",
    //     livox_to_camera.transform.translation.x, livox_to_camera.transform.translation.y, livox_to_camera.transform.translation.z);
    livox_to_camera_affine = tf2::transformToEigen(livox_to_camera);
    Eigen::Affine3d map_to_livox_affine = Eigen::Affine3d::Identity(); //map->雷达
    Eigen::Affine3d map_to_camera_affine = Eigen::Affine3d::Identity();

    camera_pt_.header.frame_id = "camera_link";
    camera_pt_.point.z = 0.0;

    if(!if_passing_door_)
    {
        passing_door_src_1_[0] = 0.0;
        passing_door_src_1_[1] = 0.0;
        passing_door_des_[0] = 0.0;
        passing_door_des_[1] = 0.0;
        passing_door_height_ = cruise_height_;
    }

    /* 计时器，pubsub初始化 */
    step_period_ms = std::chrono::milliseconds(static_cast<int64_t>(250));
    mission_period_ms = std::chrono::milliseconds(static_cast<int64_t>(100));

    navigate_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    if (!this->navigate_to_pose_client_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
    }
    navigate_to_pose_action_ = nav2_msgs::action::NavigateToPose::Goal();
    navigate_to_pose_action_.pose.pose.position.x = 0.0;
    navigate_to_pose_action_.pose.pose.position.y = 0.0;
    navigate_to_pose_action_.pose.pose.position.z = 0.0;
    navigate_to_pose_action_.pose.pose.orientation.x = 0.0;
    navigate_to_pose_action_.pose.pose.orientation.y = 0.0;
    navigate_to_pose_action_.pose.pose.orientation.z = 0.0;
    navigate_to_pose_action_.pose.pose.orientation.w = 1.0;
    navigate_to_pose_goal_ = nav2_msgs::action::NavigateToPose::Goal();

    target_pose_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("/robot/target_pose", 10);
    goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    nav_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robot/nav_state", 10);
    passing_door_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robot/passing_door_state", 10);
    turning_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robot/turning_state", 10);
    obstacle_height_pub_ = this->create_publisher<std_msgs::msg::Float64>("/robot/obstacle_height", 10);
    clear_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robot/clear_state", 10);
    camera_choose_pub_ = this->create_publisher<std_msgs::msg::Bool>("/camera/choose", 10);
    arm_state_sub_ = this->create_subscription<std_msgs::msg::Bool>("/robot/arm_state", 10,
        std::bind(&BehaviorControl::ArmStateCallback, this, std::placeholders::_1));
    current_pose_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>("/robot/current_pose",
            10, std::bind(&BehaviorControl::CurrentPoseCallback, this, std::placeholders::_1));
    image_location_sub_ = this->create_subscription<robot_interfaces::msg::ImageLocation>("/robot/image_location",
            10, std::bind(&BehaviorControl::ImageLocationCallback, this, std::placeholders::_1));
    usbcamera_info_sub_ = this->create_subscription<robot_interfaces::msg::ImageLocation>("/robot/usb_camera",
            10, std::bind(&BehaviorControl::USBCameraInfoCallback, this, std::placeholders::_1));

    servo_parameter_client_ = this->create_client<rcl_interfaces::srv::SetParameters>("/servo_node/set_parameters");
    controller_server_parameter_client_ = this->create_client<rcl_interfaces::srv::SetParameters>("/controller_server/set_parameters");
    local_costmap_parameter_client_ = this->create_client<rcl_interfaces::srv::SetParameters>("/local_costmap/local_costmap/set_parameters");
    //等待服务可用
    while (!servo_parameter_client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "servo_parameter service not available, waiting...");
    }
    while (!controller_server_parameter_client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "controller_server_parameter service not available, waiting...");
    }
    while (!local_costmap_parameter_client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "local_costmap_parameter service not available, waiting...");
    }

    step_timer_ = this->create_wall_timer(step_period_ms, std::bind(&BehaviorControl::step_timer_callback, this));
    mission_timer_ = this->create_wall_timer(mission_period_ms, std::bind(&BehaviorControl::mission_timer_callback, this));
}

void BehaviorControl::CurrentPoseCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
    current_x_ = msg->transform.translation.x;
    current_y_ = msg->transform.translation.y;
    current_z_ = msg->transform.translation.z + 0.39;

    map_to_livox.header.stamp = msg->header.stamp;
    map_to_livox.header.frame_id = "map";
    map_to_livox.child_frame_id = "livox";
    map_to_livox.transform.translation.x = current_x_;
    map_to_livox.transform.translation.y = current_y_;
    map_to_livox.transform.translation.z = current_z_;
    map_to_livox.transform.rotation.x = msg->transform.rotation.x;
    map_to_livox.transform.rotation.y = msg->transform.rotation.y;
    map_to_livox.transform.rotation.z = msg->transform.rotation.z;
    map_to_livox.transform.rotation.w = msg->transform.rotation.w;

    map_to_livox_affine = tf2::transformToEigen(map_to_livox);
    map_to_camera_affine = map_to_livox_affine * livox_to_camera_affine;

    map_to_camera.header.stamp = msg->header.stamp;
    map_to_camera.header.frame_id = "map";
    map_to_camera.child_frame_id = "camera_link";
    map_to_camera.transform.translation.x = map_to_camera_affine.translation().x();
    map_to_camera.transform.translation.y = map_to_camera_affine.translation().y();
    map_to_camera.transform.translation.z = map_to_camera_affine.translation().z();
    Eigen::Matrix3d rotation_matrix = map_to_camera_affine.rotation();
    Eigen::Quaterniond base_quat_result(rotation_matrix);
    map_to_camera.transform.rotation.x = base_quat_result.x();
    map_to_camera.transform.rotation.y = base_quat_result.y();
    map_to_camera.transform.rotation.z = base_quat_result.z();
    map_to_camera.transform.rotation.w = base_quat_result.w();
}

void BehaviorControl::USBCameraInfoCallback(const robot_interfaces::msg::ImageLocation::SharedPtr msg) //usb相机发过来的是livox系
{
    if(current_step == 112 || current_step == 114)
    {
        if(msg->id == 6)
        {
            if_find_random_target_ = true;
            geometry_msgs::msg::PointStamped camera_point, world_point;
            camera_point.header.frame_id = "livox";
            camera_point.header.stamp = this->now();
            camera_point.point.x = msg->image_x;
            camera_point.point.y = msg->image_y;
            camera_point.point.z = 0.0;
            try
            {
                tf2::doTransform(camera_point, world_point, map_to_livox);

                random_target_[0] = world_point.point.x;
                random_target_[1] = world_point.point.y;
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "TF transform failed in USBCameraInfoCallback: %s", ex.what());
            }
        }
        if(msg->id == 5)
        {
            RCLCPP_INFO(this->get_logger(), "USB Camera detected!!!!!");
            if_find_random_tank_target_ = true;
            geometry_msgs::msg::PointStamped camera_point, world_point;
            camera_point.header.frame_id = "livox";
            camera_point.header.stamp = this->now();
            camera_point.point.x = msg->image_x;
            camera_point.point.y = msg->image_y;
            camera_point.point.z = 0.0;
            try
            {
                tf2::doTransform(camera_point, world_point, map_to_livox);

                random_tank_target_[0] = world_point.point.x;
                random_tank_target_[1] = world_point.point.y;
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "TF transform failed in USBCameraInfoCallback: %s", ex.what());
            }

            //
            for (int i = 0; i < target_positions_.size(); i++)
            {
                RCLCPP_INFO(this->get_logger(), "1111111111111111111111111");
                if ((random_tank_target_[0] - target_positions_[target_sequence_[i]][0])*(random_tank_target_[0] - target_positions_[target_sequence_[i]][0]) +
                    (random_tank_target_[1] - target_positions_[target_sequence_[i]][1])*(random_tank_target_[1] - target_positions_[target_sequence_[i]][1]) <= 1.0)
                {
                    RCLCPP_INFO(this->get_logger(), "tank与其他目标点重合!!!!!!!!");
                    if_find_random_tank_target_ = false;
                }
            }
        }
    }
}

void BehaviorControl::start_takeoff_circle_if_needed()
{
    if(takeoff_circle_started_)
        return;

    // 只在首次进入时锁定圆心，使 Point-LIO 后续轻微抖动也不会改变操作员
    // 看到的圆周轨迹；第一个目标点落在当前位置。
    takeoff_circle_start_time_ = this->now();
    takeoff_circle_center_x_ = current_x_ - takeoff_circle_radius_;
    takeoff_circle_center_y_ = current_y_;
    takeoff_circle_started_ = true;

    RCLCPP_INFO(this->get_logger(),
        "起飞确认，开始匀速圆周运动: radius=%lf, speed=%lf, direction=%lf, center=(%lf, %lf)",
        takeoff_circle_radius_, takeoff_circle_speed_, takeoff_circle_direction_,
        takeoff_circle_center_x_, takeoff_circle_center_y_);
}

void BehaviorControl::publish_takeoff_circle_target()
{
    // 圆周参数非法时退化为原地悬停目标，不进入后续旧任务流程。
    if(takeoff_circle_radius_ < kMinCircleRadius || takeoff_circle_speed_ < kMinCircleSpeed)
    {
        current_target_position_.transform.translation.x = current_x_;
        current_target_position_.transform.translation.y = current_y_;
        current_target_position_.transform.translation.z = cruise_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        return;
    }

    start_takeoff_circle_if_needed();

    // 用 omega = v / r 把固定线速度转换成角度推进量。
    double elapsed = (this->now() - takeoff_circle_start_time_).seconds();
    double angle_abs = std::fmod(takeoff_circle_speed_ / takeoff_circle_radius_ * elapsed, kTwoPi);

    double angle = takeoff_circle_direction_ * angle_abs;
    current_target_position_.header.stamp = this->now();
    current_target_position_.header.frame_id = map_frame_;
    current_target_position_.child_frame_id = target_frame_;
    current_target_position_.transform.translation.x =
        takeoff_circle_center_x_ + takeoff_circle_radius_ * std::cos(angle);
    current_target_position_.transform.translation.y =
        takeoff_circle_center_y_ + takeoff_circle_radius_ * std::sin(angle);
    current_target_position_.transform.translation.z = cruise_height_;
    current_target_position_.transform.rotation.x = 0.0;
    current_target_position_.transform.rotation.y = 0.0;
    current_target_position_.transform.rotation.z = 0.0;
    current_target_position_.transform.rotation.w = 1.0;
    target_pose_pub_->publish(current_target_position_);

    if_nav = false;
    current_passing_door_ = false;
    if_turning = false;
}

void BehaviorControl::ImageLocationCallback(const robot_interfaces::msg::ImageLocation::SharedPtr msg) //d435发过来是camera_link系
{
    detected_target_id_ = msg->id;
    if(detected_target_id_ == 6) //十字随机目标
    {
        if (!if_find_random_target_)
        {
            if_find_random_target_ = true;
            detected_target_[0] = msg->image_x;
            detected_target_[1] = msg->image_y;

            geometry_msgs::msg::PointStamped camera_point, world_point;
            camera_point.header.frame_id = "camera_link";
            camera_point.header.stamp = this->now();
            camera_point.point.x = msg->image_x;
            camera_point.point.y = msg->image_y;
            camera_point.point.z = 0.0;
            try
            {
                tf2::doTransform(camera_point, world_point, map_to_camera);

                random_target_[0] = world_point.point.x;
                random_target_[1] = world_point.point.y;
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "TF transform failed in ImageLocationCallback: %s", ex.what());
            }
        }
    }
    else if(detected_target_id_ == 5)  //tank随机目标
    {
        RCLCPP_INFO(this->get_logger(), "!!!!!!!!Detected tank!!!!!");
        if (!if_find_random_tank_target_)
        {
            RCLCPP_INFO(this->get_logger(), "----------------------");
            if_find_random_tank_target_ = true;
            detected_target_[0] = msg->image_x;
            detected_target_[1] = msg->image_y;

            geometry_msgs::msg::PointStamped camera_point, world_point;
            camera_point.header.frame_id = "camera_link";
            camera_point.header.stamp = this->now();
            camera_point.point.x = msg->image_x;
            camera_point.point.y = msg->image_y;
            camera_point.point.z = 0.0;
            try
            {
                tf2::doTransform(camera_point, world_point, map_to_camera);

                random_tank_target_[0] = world_point.point.x;
                random_tank_target_[1] = world_point.point.y;
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "TF transform failed in ImageLocationCallback: %s", ex.what());
            }
            //
            for (int i = 0; i < target_positions_.size(); i++)
            {
                if ((random_tank_target_[0] - target_positions_[target_sequence_[i]][0])*(random_tank_target_[0] - target_positions_[target_sequence_[i]][0]) +
                    (random_tank_target_[1] - target_positions_[target_sequence_[i]][1])*(random_tank_target_[1] - target_positions_[target_sequence_[i]][1]) <= 1.0)
                {
                    RCLCPP_INFO(this->get_logger(), "??????目标点重合");
                    if_find_random_tank_target_ = false;
                }
            }
        }
    }
    else
    {
        if(detected_target_id_ == 2)  //排除tank和bridge误识别情况
        {
            // 先转坐标系
            geometry_msgs::msg::PointStamped camera_point, world_point;
            double curr_detected_target[2];
            camera_point.header.frame_id = "camera_link";
            camera_point.header.stamp = this->now();
            camera_point.point.x = msg->image_x;
            camera_point.point.y = msg->image_y;
            camera_point.point.z = 0.0;
            try
            {
                tf2::doTransform(camera_point, world_point, map_to_camera);

                curr_detected_target[0] = world_point.point.x;
                curr_detected_target[1] = world_point.point.y;
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "TF transform failed in ImageLocationCallback: %s", ex.what());
            }

            if((curr_detected_target[0] - bridge_[0]) * (curr_detected_target[0] - bridge_[0]) +
                (curr_detected_target[1] - bridge_[1]) * (curr_detected_target[1] - bridge_[1]) >= 1.0) //实际是tank
            {
                if_find_random_tank_target_ = true;
                random_tank_target_[0] = curr_detected_target[0];
                random_tank_target_[1] = curr_detected_target[1];
            }
        }
        detected_target_[0] = msg->image_x;
        detected_target_[1] = msg->image_y;
    }
}

void BehaviorControl::step_timer_callback()
{
    RCLCPP_INFO(this->get_logger(), "---------->current step: %d", current_step);
    if(current_step == 0) //等待飞控解锁
    {
        if(arming_state)
            current_step = 1;
    }
    else if(current_step == 300)
    {
    RCLCPP_INFO(this->get_logger(),"------------------模拟拉高");
        if(fabs(current_z_ - cruise_height_) <= 0.05)
        {
            RCLCPP_INFO(this->get_logger(),"------------------拉高结束，高度：%f",current_z_);
        }
    }
    else if(current_step == 1) //等待起飞至巡航高度
    {
        if(fabs(current_z_ - cruise_height_) <= 0.05)
        {
            // 绕圈调试已关闭（试飞前）：确认起飞后直接进入原任务流程。
            // 注意：恢复后会先进入 111/112 随机靶搜索，再进 21 定点导航。
            // if(takeoff_circle_enabled_)
            //     current_step = kTakeoffCircleStep;
            // else
            current_step = 111;  //current_step=61;
        }
    }
    else if(current_step == kTakeoffCircleStep) //起飞后持续执行匀速圆周运动
    {
        // 不再推进 current_step，确保不会误入后续投掷/穿门流程。
        start_takeoff_circle_if_needed();
    }
    //在起飞点左右两侧移动，寻找随机靶
    else if(current_step == 111)
    {
        if(fabs(current_x_ - random_target_init_search_1_[0]) <= 0.2)
            if(fabs(current_y_ - random_target_init_search_1_[1]) <= 0.2)
            {
                current_step = 112;
            }
    }
    else if(current_step == 112)
    {
        if(if_find_random_target_ && if_find_random_tank_target_) //找到随机靶
        {
            detection_cnt = 0;
            current_step = 21;
            for(int i = 0; i < 3; i++)
            {
                std_msgs::msg::Bool camera_choose_msg;
                camera_choose_msg.data = true;
                camera_choose_pub_->publish(camera_choose_msg);
                RCLCPP_INFO(this->get_logger(), ">>>>>>>>>>>>>>>>>>>使用d435<<<<<<<<<<<<<<<<<<<");
            }
        }
        detection_cnt++;
        if(detection_cnt >= dynamic_detection_cnt_threshold_)
        {
            detection_cnt = 0;
            current_step = 113;
        }
    }
    else if(current_step == 113)
    {
        if(fabs(current_x_ - random_target_init_search_2_[0]) <= 0.2)
            if(fabs(current_y_ - random_target_init_search_2_[1]) <= 0.2)
            {
                current_step = 114;
            }
    }
    else if(current_step == 114)
    {
        if(if_find_random_target_ && if_find_random_tank_target_) //找到随机靶
        {
            detection_cnt = 0;
            current_step = 21;
            for(int i = 0; i < 3; i++)
            {
                std_msgs::msg::Bool camera_choose_msg;
                camera_choose_msg.data = true;
                camera_choose_pub_->publish(camera_choose_msg);
                RCLCPP_INFO(this->get_logger(), ">>>>>>>>>>>>>>>>>>>使用d435<<<<<<<<<<<<<<<<<<<");
            }
        }
        detection_cnt++;
        if(detection_cnt >= dynamic_detection_cnt_threshold_)
        {
            detection_cnt = 0;
            current_step = 21;
            for(int i = 0; i < 3; i++)
            {
                std_msgs::msg::Bool camera_choose_msg;
                camera_choose_msg.data = true;
                camera_choose_pub_->publish(camera_choose_msg);
                RCLCPP_INFO(this->get_logger(), ">>>>>>>>>>>>>>>>>>>使用d435<<<<<<<<<<<<<<<<<<<");
            }
        }
    }
    //进入第一个目标点循环
    else if(current_step == 21) //是否到目标点附近
    {
        if(fabs(current_x_ - target_positions_[target_sequence_[0]][0]) < 0.25)
            if(fabs(current_y_ - target_positions_[target_sequence_[0]][1]) < 0.25)
            {
                if(if_hit_target_[target_sequence_[0]]) //进行投掷
                    current_step = 22;
                else //不投掷
                    current_step = 31;
            }
    }
    else if(current_step == 22) //拉高
    {
        if (fabs(current_z_ - detection_height_) < 0.2)
        {
            current_step = 23;
        }
    }
    else if(current_step == 23) //进行识别
    {
        detection_cnt++;
        if(detection_cnt >= detection_cnt_threshold_)
        {
            detection_cnt = 0;
            current_step = 24;
            camera_pt_.point.x = detected_target_[0];
            camera_pt_.point.y = detected_target_[1];
        }
    }
    else if(current_step == 24) //下降投掷
    {
        eject_cnt++;
        if(eject_cnt >= eject_cnt_threshold_) //计时
        {
            eject_cnt = 0;
            if(if_need_passing_all_) //需要遍历静态靶
                current_step = 31;
            else
                current_step = 91; //不需要遍历静态靶
        }
    }
    //进入第二个目标点循环
    else if(current_step == 31) //是否到目标点附近
    {
        if(fabs(current_x_ - target_positions_[target_sequence_[1]][0]) < 0.25)
            if(fabs(current_y_ - target_positions_[target_sequence_[1]][1]) < 0.25)
            {
                if(if_hit_target_[target_sequence_[1]]) //进行投掷
                    current_step = 32;
                else //不投掷
                    current_step = 41;
            }
    }
    else if(current_step == 32) //拉高
    {
        if (fabs(current_z_ - detection_height_) < 0.2)
        {
            current_step = 33;
        }
    }
    else if(current_step == 33) //进行识别
    {
        detection_cnt++;
        if(detection_cnt >= detection_cnt_threshold_)
        {
            detection_cnt = 0;
            current_step = 34;
            camera_pt_.point.x = detected_target_[0];
            camera_pt_.point.y = detected_target_[1];
        }
    }
    else if(current_step == 34) //下降投掷
    {
        eject_cnt++;
        if(eject_cnt >= eject_cnt_threshold_) //计时
        {
            eject_cnt = 0;
            current_step = 41;
        }
    }
    //进入第三个目标点循环
    else if(current_step == 41) //是否到目标点附近
    {
        if(fabs(current_x_ - target_positions_[target_sequence_[2]][0]) < 0.25)
            if(fabs(current_y_ - target_positions_[target_sequence_[2]][1]) < 0.25)
            {
                if(if_hit_target_[target_sequence_[2]]) //进行投掷
                    current_step = 42;
                else //不投掷
                    current_step = 51;
            }
    }
    else if(current_step == 42) //拉高
    {
        if (fabs(current_z_ - detection_height_) < 0.2)
        {
            current_step = 43;
        }
    }
    else if(current_step == 43) //进行识别
    {
        detection_cnt++;
        if(detection_cnt >= detection_cnt_threshold_)
        {
            detection_cnt = 0;
            current_step = 44;
            camera_pt_.point.x = detected_target_[0];
            camera_pt_.point.y = detected_target_[1];
        }
    }
    else if(current_step == 44) //下降投掷
    {
        eject_cnt++;
        if(eject_cnt >= eject_cnt_threshold_) //计时
        {
            eject_cnt = 0;
            current_step = 51;
        }
    }
    //进入第四个目标点循环
    else if(current_step == 51) //是否到目标点附近
    {
        if(fabs(current_x_ - target_positions_[target_sequence_[3]][0]) < 0.25)
            if(fabs(current_y_ - target_positions_[target_sequence_[3]][1]) < 0.25)
            {
                if(if_hit_target_[target_sequence_[3]]) //进行投掷
                    current_step = 52;
                else //不投掷
                    current_step = 61;
            }
    }
    else if(current_step == 52) //拉高
    {
        if (fabs(current_z_ - detection_height_) < 0.2)
        {
            current_step = 53;
        }
    }
    else if(current_step == 53) //进行识别
    {
        detection_cnt++;
        if(detection_cnt >= detection_cnt_threshold_)
        {
            detection_cnt = 0;
            current_step = 54;
            camera_pt_.point.x = detected_target_[0];
            camera_pt_.point.y = detected_target_[1];
        }
    }
    else if(current_step == 54) //下降投掷
    {
        eject_cnt++;
        if(eject_cnt >= eject_cnt_threshold_) //计时
        {
            eject_cnt = 0;
            current_step = 91;
        }
    }
    //进入搜索随机靶任务
    else if(current_step == 91) //搜索第一个点
    {
        if(if_find_random_target_ && if_find_random_tank_target_)
        {
            current_step = 61; //找到2个随机靶
            RCLCPP_INFO(this->get_logger(), "----------找到2个随机靶--------");
            return;
        }
        if(fabs(current_x_ - random_target_search_1_[0]) < 0.15)
            if(fabs(current_y_ - random_target_search_1_[1]) < 0.15)
            {
                current_step = 92;
            }
    }
    else if(current_step == 92) //搜索第二个点
    {
        if(if_find_random_target_ && if_find_random_tank_target_)
        {
            current_step = 61; //找到2个随机靶
            return;
        }
        if(fabs(current_x_ - random_target_search_2_[0]) < 0.15)
            if(fabs(current_y_ - random_target_search_2_[1]) < 0.15)
            {
                current_step = 93;
            }
    }
    else if(current_step == 93) //搜索第三个点
    {
        if(fabs(current_x_ - random_target_search_3_[0]) < 0.15)
            if(fabs(current_y_ - random_target_search_3_[1]) < 0.15)
            {
                if(!if_find_random_tank_target_)
                {
                    random_tank_target_[0] = prev_random_tank_target_[0];
                    random_tank_target_[1] = prev_random_tank_target_[1];
                }
                current_step = 61;
                RCLCPP_INFO(this->get_logger(), "random_target: %lf, %lf", random_target_[0], random_target_[1]);
                RCLCPP_INFO(this->get_logger(), "random_tank_target: %lf, %lf", random_tank_target_[0], random_tank_target_[1]);
            }
    }
    //进入动态目标点循环（逻辑待改）
    else if (current_step == 61)
    {
        if(!if_find_random_tank_target_)
        {
            random_tank_target_[0] = prev_random_tank_target_[0];
            random_tank_target_[1] = prev_random_tank_target_[1];
        }
        if (fabs(current_x_ - random_tank_target_[0]) < 0.2) //是否到目标点附近
            if (fabs(current_y_ - random_tank_target_[1]) < 0.2)
                current_step = 62;
    }
    else if(current_step == 62)  //拉高
    {
        if (fabs(current_z_ - detection_height_) < 0.2)
        {
            current_step = 63;
        }
    }
    else if(current_step == 63) //进行识别
    {
        detection_cnt++;
        if(detection_cnt >= detection_cnt_threshold_)
        {
            detection_cnt = 0;
            current_step = 64;
            if(if_find_random_tank_target_)
            {
                stored_point[0] = random_tank_target_[0];
                stored_point[1] = random_tank_target_[1];
            }
            else
            {
                stored_point[0] = detected_target_[0];
                stored_point[1] = detected_target_[1];
            }
        }
    }
    else if(current_step == 64) //下降投掷
    {
        eject_cnt++;
        if(eject_cnt >= eject_cnt_threshold_) //计时
        {
            eject_cnt = 0;
            current_step = 101; //进入随机靶投掷任务
        }
    }
    //进入随机靶投掷任务
    else if(current_step == 101) //是否到目标点附近
    {
        if(!if_find_random_tank_target_)
        {
            random_target_[0] = prev_random_target_[0];
            random_target_[1] = prev_random_target_[1];
        }
        if(fabs(current_x_ - random_target_[0]) < 0.2)
            if(fabs(current_y_ - random_target_[1]) < 0.2)
            {
                current_step = 102;
            }
    }
    else if(current_step == 102) //拉高
    {
        if (fabs(current_z_ - detection_height_) < 0.2)
        {
            current_step = 103;
        }
    }
    else if(current_step == 103) //进行识别
    {
        detection_cnt++;
        if(detection_cnt >= detection_cnt_threshold_)
        {
            detection_cnt = 0;
            current_step = 104;
            if(if_find_random_target_)
            {
                stored_point[0] = random_target_[0];
                stored_point[1] = random_target_[1];
            }
            else
            {
                stored_point[0] = detected_target_[0];
                stored_point[1] = detected_target_[1];
            }
        }
    }
    else if(current_step == 104) //下降投掷
    {
        eject_cnt++;
        if(eject_cnt >= eject_cnt_threshold_) //计时
        {
            eject_cnt = 0;
            current_step = 71; //进入穿门任务
        }
    }
    //进入穿门任务
    else if(current_step == 71) //去穿门起点
    {
        if(fabs(current_x_ - passing_door_src_1_[0]) < 0.15)
            if(fabs(current_y_ - passing_door_src_1_[1]) < 0.15)
            {
                if(!if_passing_door_) //不穿门回起点
                    current_step = 75;
                else
                    current_step = 72;
            }
    }
    else if(current_step == 72) //原地转向90度
    {
        turning_cnt++;
        if(turning_cnt >= 23)
        {
            std_msgs::msg::Bool clear_state_msg;
            clear_state_msg.data = true;
            clear_state_pub_->publish(clear_state_msg);
        }
        if (turning_cnt >= turning_cnt_threshold_)
        {
            std_msgs::msg::Bool clear_state_msg;
            clear_state_msg.data = false;
            clear_state_pub_->publish(clear_state_msg);

            turning_cnt = 0;
            current_step = 74;
        }
    }
    else if(current_step == 73) //导航至穿门中间点
    {
        if(fabs(current_x_ - passing_door_src_2_[0]) < 0.1)
            if(fabs(current_y_ - passing_door_src_2_[1]) < 0.1)
            {
                current_step = 74;
            }
    }
    else if(current_step == 74) //导航至穿门终点
    {
        if(fabs(current_x_ - passing_door_des_[0]) < 0.12)
            if(fabs(current_y_ - passing_door_des_[1]) < 0.12)
            {
                current_step = 82; //进入降落穿门后终点状态
            }
    }

    else if(current_step == 75) //不穿门直接回起点
    {
        if(fabs(current_x_) < 0.1)
            if(fabs(current_y_) < 0.1)
            {
                current_step = 81; //进入降落起点状态
            }
    }
}

void BehaviorControl::mission_timer_callback()
{
    RCLCPP_INFO(this->get_logger(), "+++++++++++++++++tank position: %lf, %lf+++++++++++++++++",
        random_tank_target_[0], random_tank_target_[1]);
    RCLCPP_INFO(this->get_logger(), "+++++++++++++++if_find_random_tank_target: %d+++++++++++++++++",if_find_random_tank_target_);
    if(current_step == 0)
    {
        current_target_position_.transform.translation.x = 0.0;
        current_target_position_.transform.translation.y = 0.0;
        current_target_position_.transform.translation.z = 0.0;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        current_passing_door_ = false;
        RCLCPP_INFO(this->get_logger(), "等待飞控解锁...");
        obstacle_height_ = 2.0;
    }
    else if(current_step == 300)
    {
        current_target_position_.transform.translation.x = 0.0;
        current_target_position_.transform.translation.y = 0.0;
        current_target_position_.transform.translation.z = cruise_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        RCLCPP_INFO(this->get_logger(), "等待起飞至巡航高度...");
    }
    else if(current_step == 1)
    {
        current_target_position_.transform.translation.x = 0.0;
        current_target_position_.transform.translation.y = 0.0;
        current_target_position_.transform.translation.z = cruise_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        RCLCPP_INFO(this->get_logger(), "等待起飞至巡航高度...");
    }
    else if(current_step == kTakeoffCircleStep)
    {
        // 圆周目标走 /robot/target_pose，再由 flight_control/mavros_adapter
        // 转成 MAVROS setpoint_position/local。
        publish_takeoff_circle_target();
    }
    else if(current_step == 111)
    {
        current_target_position_.transform.translation.x = random_target_init_search_1_[0];
        current_target_position_.transform.translation.y = random_target_init_search_1_[1];
        current_target_position_.transform.translation.z = detection_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;

        // rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
        // action_goal.pose.header.frame_id = "map";
        // action_goal.pose.pose.position.x = random_target_init_search_1_[0];
        // action_goal.pose.pose.position.y = random_target_init_search_1_[1];
        // action_goal.pose.pose.position.z = cruise_height_;
        // action_goal.pose.pose.orientation.x = 0.0;
        // action_goal.pose.pose.orientation.y = 0.0;
        // action_goal.pose.pose.orientation.z = 0.0;
        // action_goal.pose.pose.orientation.w = 1.0;
        // navigate_to_pose_client_->async_send_goal(action_goal);
        // if_nav = true;
    }
    else if(current_step == 112)
    {
        current_target_position_.transform.translation.x = random_target_init_search_1_[0];
        current_target_position_.transform.translation.y = random_target_init_search_1_[1];
        current_target_position_.transform.translation.z = detection_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        RCLCPP_INFO(this->get_logger(), "第一个随机靶搜索点");
    }
    else if(current_step == 113)
    {
        current_target_position_.transform.translation.x = random_target_init_search_2_[0];
        current_target_position_.transform.translation.y = random_target_init_search_2_[1];
        current_target_position_.transform.translation.z = detection_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        // rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
        // action_goal.pose.header.frame_id = "map";
        // action_goal.pose.pose.position.x = random_target_init_search_2_[0];
        // action_goal.pose.pose.position.y = random_target_init_search_2_[1];
        // action_goal.pose.pose.position.z = cruise_height_;
        // action_goal.pose.pose.orientation.x = 0.0;
        // action_goal.pose.pose.orientation.y = 0.0;
        // action_goal.pose.pose.orientation.z = 0.0;
        // action_goal.pose.pose.orientation.w = 1.0;
        // navigate_to_pose_client_->async_send_goal(action_goal);
        // if_nav = true;
    }
    else if(current_step == 114)
    {
        current_target_position_.transform.translation.x = random_target_init_search_2_[0];
        current_target_position_.transform.translation.y = random_target_init_search_2_[1];
        current_target_position_.transform.translation.z = detection_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        RCLCPP_INFO(this->get_logger(), "第二个随机靶搜索点");
    }
    else if(current_step == 21)
    {
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
		action_goal.pose.header.frame_id = "map";
        action_goal.pose.pose.position.x = target_positions_[target_sequence_[0]][0];
        action_goal.pose.pose.position.y = target_positions_[target_sequence_[0]][1];
        action_goal.pose.pose.position.z = cruise_height_;
        action_goal.pose.pose.orientation.x = 0.0;
        action_goal.pose.pose.orientation.y = 0.0;
        action_goal.pose.pose.orientation.z = 0.0;
        action_goal.pose.pose.orientation.w = 1.0;
        navigate_to_pose_client_->async_send_goal(action_goal);
        if_nav = true;
        RCLCPP_INFO(this->get_logger(), "第一个目标点: %lf, %lf", target_positions_[target_sequence_[0]][0],
                            target_positions_[target_sequence_[0]][1]);
    }
    else if(current_step == 22)
    {
        current_target_position_.transform.translation.x = target_positions_[target_sequence_[0]][0];
        current_target_position_.transform.translation.y = target_positions_[target_sequence_[0]][1];
        current_target_position_.transform.translation.z = detection_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        RCLCPP_INFO(this->get_logger(), "拉高中...current_target_z: %f", current_target_position_.transform.translation.z);
    }
    else if(current_step == 23)  // 进行识别
	{
        camera_pt_.header.stamp = this->now();
        camera_pt_.point.x = detected_target_[0];
        camera_pt_.point.y = detected_target_[1];

        try
        {
            tf2::doTransform(camera_pt_, world_pt_, map_to_camera);

            current_target_position_.transform.translation.x = world_pt_.point.x + offset_x_1_;
            current_target_position_.transform.translation.y = world_pt_.point.y + offset_y_1_;
            current_target_position_.transform.translation.z = detection_height_;

            target_pose_pub_->publish(current_target_position_);
            RCLCPP_INFO(this->get_logger(), "识别中... (相机发来map坐标: %.2f, %.2f, %.2f)",
                        world_pt_.point.x, world_pt_.point.y, detection_height_);
            RCLCPP_INFO(this->get_logger(), "识别中... (预设map坐标: %.2f, %.2f, %.2f)",
                        target_positions_[target_sequence_[0]][0], target_positions_[target_sequence_[0]][1], detection_height_);
            RCLCPP_INFO(this->get_logger(), "识别中... (飞机自身坐标: %.2f, %.2f)",
                                current_x_, current_y_);
            RCLCPP_INFO(this->get_logger(), "识别中... (转换前坐标: %.2f, %.2f)",
                        camera_pt_.point.x, camera_pt_.point.y);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF transform failed in step 23: %s", ex.what());
        }

        if_nav = false;
    }
    else if(current_step == 24)  // 下降投掷
    {
        camera_pt_.header.stamp = this->now();
        try
        {
            tf2::doTransform(camera_pt_, world_pt_, map_to_camera);

            current_target_position_.transform.translation.x = world_pt_.point.x + offset_x_1_;
            current_target_position_.transform.translation.y = world_pt_.point.y + offset_y_1_;

            if (current_z_ - eject_height_ >= 0.6)
                current_target_position_.transform.translation.z = current_z_ - 0.6;
            else
                current_target_position_.transform.translation.z = eject_height_;

            target_pose_pub_->publish(current_target_position_);
            RCLCPP_INFO(this->get_logger(), "下降投掷目标(map): %.2f, %.2f, %.2f",
                        world_pt_.point.x, world_pt_.point.y, current_target_position_.transform.translation.z);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF transform failed in step 24: %s", ex.what());
        }
        if(eject_cnt >= 13)
        {
            servo_index_ = 1;
            RCLCPP_INFO(this->get_logger(), "下降投掷，第 %d 个投放位", servo_index_);
            if (servo_index_ == last_servo_index_)
                return;
            last_servo_index_ = servo_index_;
            set_parameter();
        }
        if_nav = false;
    }
    else if(current_step == 31)
    {
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
		action_goal.pose.header.frame_id = "map";
        action_goal.pose.pose.position.x = target_positions_[target_sequence_[1]][0];
        action_goal.pose.pose.position.y = target_positions_[target_sequence_[1]][1];
        action_goal.pose.pose.position.z = cruise_height_;
        action_goal.pose.pose.orientation.x = 0.0;
        action_goal.pose.pose.orientation.y = 0.0;
        action_goal.pose.pose.orientation.z = 0.0;
        action_goal.pose.pose.orientation.w = 1.0;
        navigate_to_pose_client_->async_send_goal(action_goal);
        if_nav = true;
        RCLCPP_INFO(this->get_logger(), "第二个目标点: %lf, %lf", target_positions_[target_sequence_[1]][0],
                            target_positions_[target_sequence_[1]][1]);
    }
    else if(current_step == 32)
    {
        current_target_position_.transform.translation.x = target_positions_[target_sequence_[1]][0];
        current_target_position_.transform.translation.y = target_positions_[target_sequence_[1]][1];
        current_target_position_.transform.translation.z = detection_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        RCLCPP_INFO(this->get_logger(), "拉高中...");
    }
    else if(current_step == 33)
    {
        camera_pt_.header.stamp = this->now();

        camera_pt_.point.x = detected_target_[0];
        camera_pt_.point.y = detected_target_[1];

        try
        {
            tf2::doTransform(camera_pt_, world_pt_, map_to_camera);

            current_target_position_.transform.translation.x = world_pt_.point.x;
            current_target_position_.transform.translation.y = world_pt_.point.y;
            current_target_position_.transform.translation.z = detection_height_;

            target_pose_pub_->publish(current_target_position_);
            RCLCPP_INFO(this->get_logger(), "识别中... (map坐标: %.2f, %.2f, %.2f)",
                        world_pt_.point.x, world_pt_.point.y, detection_height_);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF transform failed in step 33: %s", ex.what());
        }

        if_nav = false;
    }
    else if(current_step == 34)  // 下降投掷
    {
        camera_pt_.header.stamp = this->now();
        try
        {
            tf2::doTransform(camera_pt_, world_pt_, map_to_camera);

            current_target_position_.transform.translation.x = world_pt_.point.x;
            current_target_position_.transform.translation.y = world_pt_.point.y;

            if (current_z_ - eject_height_ >= 0.6)
                current_target_position_.transform.translation.z = current_z_ - 0.6;
            else
                current_target_position_.transform.translation.z = eject_height_;

            target_pose_pub_->publish(current_target_position_);
            RCLCPP_INFO(this->get_logger(), "下降投掷目标(map): %.2f, %.2f, %.2f",
                        world_pt_.point.x, world_pt_.point.y, current_target_position_.transform.translation.z);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF transform failed in step 34: %s", ex.what());
        }
        if(eject_cnt >= 13)
        {
            servo_index_ = 1;
            RCLCPP_INFO(this->get_logger(), "下降投掷，第 %d 个投放位", servo_index_);
            if (servo_index_ == last_servo_index_)
                return;
            last_servo_index_ = servo_index_;
            set_parameter();
        }
        if_nav = false;
    }

    else if(current_step == 41)
    {
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
		action_goal.pose.header.frame_id = "map";
        action_goal.pose.pose.position.x = target_positions_[target_sequence_[2]][0];
        action_goal.pose.pose.position.y = target_positions_[target_sequence_[2]][1];
        action_goal.pose.pose.position.z = cruise_height_;
        action_goal.pose.pose.orientation.x = 0.0;
        action_goal.pose.pose.orientation.y = 0.0;
        action_goal.pose.pose.orientation.z = 0.0;
        action_goal.pose.pose.orientation.w = 1.0;
        navigate_to_pose_client_->async_send_goal(action_goal);
        if_nav = true;
        RCLCPP_INFO(this->get_logger(), "第三个目标点: %lf, %lf", target_positions_[target_sequence_[2]][0],
                            target_positions_[target_sequence_[2]][1]);
    }
    else if(current_step == 42)
    {
        current_target_position_.transform.translation.x = target_positions_[target_sequence_[2]][0];
        current_target_position_.transform.translation.y = target_positions_[target_sequence_[2]][1];
        current_target_position_.transform.translation.z = detection_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        RCLCPP_INFO(this->get_logger(), "拉高中...");
    }
    else if(current_step == 43)
    {
        camera_pt_.header.stamp = this->now();

        camera_pt_.point.x = detected_target_[0];
        camera_pt_.point.y = detected_target_[1];

        try
        {
            tf2::doTransform(camera_pt_, world_pt_, map_to_camera);

            current_target_position_.transform.translation.x = world_pt_.point.x;
            current_target_position_.transform.translation.y = world_pt_.point.y;
            current_target_position_.transform.translation.z = detection_height_;

            target_pose_pub_->publish(current_target_position_);
            RCLCPP_INFO(this->get_logger(), "识别中... (map坐标: %.2f, %.2f, %.2f)",
                        world_pt_.point.x, world_pt_.point.y, detection_height_);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF transform failed in step 43: %s", ex.what());
        }

        if_nav = false;
    }
    else if(current_step == 44)  // 下降投掷
    {
        camera_pt_.header.stamp = this->now();
        try
        {
            tf2::doTransform(camera_pt_, world_pt_, map_to_camera);

            current_target_position_.transform.translation.x = world_pt_.point.x;
            current_target_position_.transform.translation.y = world_pt_.point.y;

            if (current_z_ - eject_height_ >= 0.6)
                current_target_position_.transform.translation.z = current_z_ - 0.6;
            else
                current_target_position_.transform.translation.z = eject_height_;

            target_pose_pub_->publish(current_target_position_);
            RCLCPP_INFO(this->get_logger(), "下降投掷目标(map): %.2f, %.2f, %.2f",
                        world_pt_.point.x, world_pt_.point.y, current_target_position_.transform.translation.z);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF transform failed in step 44: %s", ex.what());
        }
        if(eject_cnt >= 13)
        {
            servo_index_ = 1;
            RCLCPP_INFO(this->get_logger(), "下降投掷，第 %d 个投放位", servo_index_);
            if (servo_index_ == last_servo_index_)
                return;
            last_servo_index_ = servo_index_;
            set_parameter();
        }
        if_nav = false;
    }

    else if(current_step == 51)
    {
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
		action_goal.pose.header.frame_id = "map";
        action_goal.pose.pose.position.x = target_positions_[target_sequence_[3]][0];
        action_goal.pose.pose.position.y = target_positions_[target_sequence_[3]][1];
        action_goal.pose.pose.position.z = cruise_height_;
        action_goal.pose.pose.orientation.x = 0.0;
        action_goal.pose.pose.orientation.y = 0.0;
        action_goal.pose.pose.orientation.z = 0.0;
        action_goal.pose.pose.orientation.w = 1.0;
        navigate_to_pose_client_->async_send_goal(action_goal);
        if_nav = true;
        RCLCPP_INFO(this->get_logger(), "第四个目标点: %lf, %lf", target_positions_[target_sequence_[3]][0],
					target_positions_[target_sequence_[3]][1]);
    }
    else if(current_step == 52)
    {
        current_target_position_.transform.translation.x = target_positions_[target_sequence_[3]][0];
        current_target_position_.transform.translation.y = target_positions_[target_sequence_[3]][1];
        current_target_position_.transform.translation.z = detection_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        RCLCPP_INFO(this->get_logger(), "拉高中...");
    }
    else if(current_step == 53)
    {
        camera_pt_.header.stamp = this->now();

        camera_pt_.point.x = detected_target_[0];
        camera_pt_.point.y = detected_target_[1];

        try
        {
            tf2::doTransform(camera_pt_, world_pt_, map_to_camera);

            current_target_position_.transform.translation.x = world_pt_.point.x;
            current_target_position_.transform.translation.y = world_pt_.point.y;
            current_target_position_.transform.translation.z = detection_height_;

            target_pose_pub_->publish(current_target_position_);
            RCLCPP_INFO(this->get_logger(), "识别中... (map坐标: %.2f, %.2f, %.2f)",
                        world_pt_.point.x, world_pt_.point.y, detection_height_);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF transform failed in step 53: %s", ex.what());
        }

        if_nav = false;
    }
    else if(current_step == 54)  // 下降投掷
    {
        camera_pt_.header.stamp = this->now();
        try
        {
            tf2::doTransform(camera_pt_, world_pt_, map_to_camera);

            current_target_position_.transform.translation.x = world_pt_.point.x;
            current_target_position_.transform.translation.y = world_pt_.point.y;

            if (current_z_ - eject_height_ >= 0.6)
                current_target_position_.transform.translation.z = current_z_ - 0.6;
            else
                current_target_position_.transform.translation.z = eject_height_;

            target_pose_pub_->publish(current_target_position_);
            RCLCPP_INFO(this->get_logger(), "下降投掷目标(map): %.2f, %.2f, %.2f",
                        world_pt_.point.x, world_pt_.point.y, current_target_position_.transform.translation.z);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF transform failed in step 54: %s", ex.what());
        }
        if(eject_cnt >= 13)
        {
            servo_index_ = 1;
            RCLCPP_INFO(this->get_logger(), "下降投掷，第 %d 个投放位", servo_index_);
            if (servo_index_ == last_servo_index_)
                return;
            last_servo_index_ = servo_index_;
            set_parameter();
        }
        if_nav = false;
    }
//----------------------动态靶妹写完----------------------
    else if(current_step == 61)
    {
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
		action_goal.pose.header.frame_id = "map";
        if(!if_find_random_tank_target_)
        {
            random_tank_target_[0] = prev_random_tank_target_[0];
            random_tank_target_[1] = prev_random_tank_target_[1];
        }
        action_goal.pose.pose.position.x = random_tank_target_[0];
        action_goal.pose.pose.position.y = random_tank_target_[1];
        action_goal.pose.pose.position.z = detection_height_;
        action_goal.pose.pose.orientation.x = 0.0;
        action_goal.pose.pose.orientation.y = 0.0;
        action_goal.pose.pose.orientation.z = 0.0;
        action_goal.pose.pose.orientation.w = 1.0;
        navigate_to_pose_client_->async_send_goal(action_goal);
        if_nav = true;
        RCLCPP_INFO(this->get_logger(), "tank目标点: %lf, %lf", random_tank_target_[0], random_tank_target_[1]);
    }
    else if(current_step == 62)
  	{
  	    current_target_position_.transform.translation.x = random_tank_target_[0];
        current_target_position_.transform.translation.y = random_tank_target_[1];
        current_target_position_.transform.translation.z = detection_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        RCLCPP_INFO(this->get_logger(), "拉高中...");
 	}
    else if(current_step == 63)
    {
        camera_pt_.header.stamp = this->now();
        if(if_find_random_tank_target_)
        {
            current_target_position_.transform.translation.x = random_tank_target_[0] + offset_x_2_;
            current_target_position_.transform.translation.y = random_tank_target_[1] + offset_y_2_;
            current_target_position_.transform.translation.z = detection_height_;
            target_pose_pub_->publish(current_target_position_);
            RCLCPP_INFO(this->get_logger(), "识别中... (相机发来map坐标: %.2f, %.2f)",
                                random_tank_target_[0], random_tank_target_[1]);
        }
        else
        {
            camera_pt_.point.x = detected_target_[0];
            camera_pt_.point.y = detected_target_[1];
            try
            {
                tf2::doTransform(camera_pt_, world_pt_, map_to_camera);

                current_target_position_.transform.translation.x = world_pt_.point.x + offset_x_2_;
                current_target_position_.transform.translation.y = world_pt_.point.y + offset_y_2_;
                current_target_position_.transform.translation.z = detection_height_;
                target_pose_pub_->publish(current_target_position_);
                RCLCPP_INFO(this->get_logger(), "识别中... (相机发来map坐标: %.2f, %.2f)",
                                    world_pt_.point.x, world_pt_.point.y);
                // RCLCPP_INFO(this->get_logger(), "识别中... (飞机自身坐标: %.2f, %.2f)",
                //                 current_x_, current_y_);
                RCLCPP_INFO(this->get_logger(), "识别中... (转换前坐标: %.2f, %.2f)",
                            camera_pt_.point.x, camera_pt_.point.y);
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "TF transform failed in step 63: %s", ex.what());
            }
        }

        if_nav = false;
    }
    else if(current_step == 64) //下降投掷
    {
        camera_pt_.header.stamp = this->now();
        if(if_find_random_tank_target_)
        {
            current_target_position_.transform.translation.x = stored_point[0] + offset_x_2_;
            current_target_position_.transform.translation.y = stored_point[1] + offset_y_2_;
            if (current_z_ - eject_height_ >= 0.6)
                current_target_position_.transform.translation.z = current_z_ - 0.6;
            else
                current_target_position_.transform.translation.z = eject_height_;

            target_pose_pub_->publish(current_target_position_);
        }
        else
        {
            camera_pt_.point.x = stored_point[0];
            camera_pt_.point.y = stored_point[1];
            camera_pt_.point.z = 0.0;
            try
            {
                tf2::doTransform(camera_pt_, world_pt_, map_to_camera);

                current_target_position_.transform.translation.x = world_pt_.point.x + offset_x_2_;
                current_target_position_.transform.translation.y = world_pt_.point.y + offset_y_2_;
                if (current_z_ - eject_height_ >= 0.6)
                    current_target_position_.transform.translation.z = current_z_ - 0.6;
                else
                    current_target_position_.transform.translation.z = eject_height_;

                target_pose_pub_->publish(current_target_position_);
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "TF transform failed in step 63: %s", ex.what());
            }
        }

        if(eject_cnt >= 13)
        {
            servo_index_ = 2;
            RCLCPP_INFO(this->get_logger(), "下降投掷，第 %d 个投放位", servo_index_);
            if (servo_index_ == last_servo_index_)
                return;
            last_servo_index_ = servo_index_;
            set_parameter();
        }
        if_nav = false;
    }

    else if(current_step == 91)
    {
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
        action_goal.pose.header.frame_id = "map";
        action_goal.pose.pose.position.x = random_target_search_1_[0];
        action_goal.pose.pose.position.y = random_target_search_1_[1];
        action_goal.pose.pose.position.z = detection_height_;
        action_goal.pose.pose.orientation.x = 0.0;
        action_goal.pose.pose.orientation.y = 0.0;
        action_goal.pose.pose.orientation.z = 0.0;
        action_goal.pose.pose.orientation.w = 1.0;
        navigate_to_pose_client_->async_send_goal(action_goal);
        if_nav = true;
        RCLCPP_INFO(this->get_logger(), "第一个搜索点: %lf, %lf", random_target_search_1_[0], random_target_search_1_[1]);
    }
    else if(current_step == 92)
    {
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
        action_goal.pose.header.frame_id = "map";
        action_goal.pose.pose.position.x = random_target_search_2_[0];
        action_goal.pose.pose.position.y = random_target_search_2_[1];
        action_goal.pose.pose.position.z = detection_height_;
        action_goal.pose.pose.orientation.x = 0.0;
        action_goal.pose.pose.orientation.y = 0.0;
        action_goal.pose.pose.orientation.z = 0.0;
        action_goal.pose.pose.orientation.w = 1.0;
        navigate_to_pose_client_->async_send_goal(action_goal);
        if_nav = true;
        RCLCPP_INFO(this->get_logger(), "第二个搜索点: %lf, %lf", random_target_search_2_[0], random_target_search_2_[1]);
    }
    else if(current_step == 93)
    {
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
        action_goal.pose.header.frame_id = "map";
        action_goal.pose.pose.position.x = random_target_search_3_[0];
        action_goal.pose.pose.position.y = random_target_search_3_[1];
        action_goal.pose.pose.position.z = detection_height_;
        action_goal.pose.pose.orientation.x = 0.0;
        action_goal.pose.pose.orientation.y = 0.0;
        action_goal.pose.pose.orientation.z = 0.0;
        action_goal.pose.pose.orientation.w = 1.0;
        navigate_to_pose_client_->async_send_goal(action_goal);
        if_nav = true;
        RCLCPP_INFO(this->get_logger(), "第三个搜索点: %lf, %lf", random_target_search_3_[0], random_target_search_3_[1]);
    }

    else if(current_step == 101)
    {
        if(!if_find_random_tank_target_)
        {
            random_target_[0] = prev_random_target_[0];
            random_target_[1] = prev_random_target_[1];
        }
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
        action_goal.pose.header.frame_id = "map";
        action_goal.pose.pose.position.x = random_target_[0];
        action_goal.pose.pose.position.y = random_target_[1];
        action_goal.pose.pose.position.z = cruise_height_;
        action_goal.pose.pose.orientation.x = 0.0;
        action_goal.pose.pose.orientation.y = 0.0;
        action_goal.pose.pose.orientation.z = 0.0;
        action_goal.pose.pose.orientation.w = 1.0;
        navigate_to_pose_client_->async_send_goal(action_goal);
        if_nav = true;
        RCLCPP_INFO(this->get_logger(), "---------随机靶: %lf, %lf", random_target_[0], random_target_[1]);
    }
    else if(current_step == 102)
    {
        current_target_position_.transform.translation.x = random_target_[0];
        current_target_position_.transform.translation.y = random_target_[1];
        current_target_position_.transform.translation.z = detection_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        RCLCPP_INFO(this->get_logger(), "拉高中...");
    }
    else if(current_step == 103)
    {
        camera_pt_.header.stamp = this->now();
        if(if_find_random_target_)
        {
            current_target_position_.transform.translation.x = random_target_[0] + offset_x_3_;
            current_target_position_.transform.translation.y = random_target_[1] + offset_y_3_;
            current_target_position_.transform.translation.z = detection_height_;
            target_pose_pub_->publish(current_target_position_);
            RCLCPP_INFO(this->get_logger(), "识别中... (map坐标: %.2f, %.2f, %.2f)",
                        random_target_[0], random_target_[1], detection_height_);
        }
        else
        {
            camera_pt_.point.x = detected_target_[0];
            camera_pt_.point.y = detected_target_[1];

            try
            {
                tf2::doTransform(camera_pt_, world_pt_, map_to_camera);

                current_target_position_.transform.translation.x = world_pt_.point.x + offset_x_3_;
                current_target_position_.transform.translation.y = world_pt_.point.y + offset_y_3_;
                current_target_position_.transform.translation.z = detection_height_;

                target_pose_pub_->publish(current_target_position_);
                RCLCPP_INFO(this->get_logger(), "识别中... (map坐标: %.2f, %.2f, %.2f)",
                            world_pt_.point.x, world_pt_.point.y, detection_height_);
                RCLCPP_INFO(this->get_logger(), "识别中... (转换前坐标: %.2f, %.2f)",
                            camera_pt_.point.x, camera_pt_.point.y);
            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_WARN(this->get_logger(), "TF transform failed in step 103: %s", ex.what());
            }
        }

        if_nav = false;
    }

    else if(current_step == 104)
    {
        camera_pt_.header.stamp = this->now();
        if(if_find_random_target_)
        {
            current_target_position_.transform.translation.x = stored_point[0] + offset_x_2_;
            current_target_position_.transform.translation.y = stored_point[1] + offset_y_2_;
            if (current_z_ - eject_height_ >= 0.6)
                current_target_position_.transform.translation.z = current_z_ - 0.6;
            else
                current_target_position_.transform.translation.z = eject_height_;

            target_pose_pub_->publish(current_target_position_);
        }
        else
        {
            camera_pt_.point.x = stored_point[0];
            camera_pt_.point.y = stored_point[1];
            camera_pt_.point.z = 0.0;
            try
            {
                tf2::doTransform(camera_pt_, world_pt_, map_to_camera);

                current_target_position_.transform.translation.x = world_pt_.point.x + offset_x_2_;
                current_target_position_.transform.translation.y = world_pt_.point.y + offset_y_2_;
                if (current_z_ - eject_height_ >= 0.6)
                    current_target_position_.transform.translation.z = current_z_ - 0.6;
                else
                    current_target_position_.transform.translation.z = eject_height_;

                target_pose_pub_->publish(current_target_position_);
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "TF transform failed in step 63: %s", ex.what());
            }
        }

        if(eject_cnt >= 13)
        {
            servo_index_ = 3;
            RCLCPP_INFO(this->get_logger(), "下降投掷，第 %d 个投放位", servo_index_);
            if (servo_index_ == last_servo_index_)
                return;
            last_servo_index_ = servo_index_;
            set_parameter();
        }
    }

    else if(current_step == 71)
    {
        if(if_passing_door_){
            rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
            action_goal.pose.header.frame_id = "map";
            action_goal.pose.pose.position.x = passing_door_src_1_[0];
            action_goal.pose.pose.position.y = passing_door_src_1_[1];
            action_goal.pose.pose.position.z = passing_door_height_;
            action_goal.pose.pose.orientation.x = 0.0;
            action_goal.pose.pose.orientation.y = 0.0;
            action_goal.pose.pose.orientation.z = 0.0;
            action_goal.pose.pose.orientation.w = 1.0;
            navigate_to_pose_client_->async_send_goal(action_goal);
            RCLCPP_INFO(this->get_logger(), "穿门起点: %lf, %lf", passing_door_src_1_[0], passing_door_src_1_[1]);
        }
        else{
			passing_door_des_[0] = 0.0;
			passing_door_des_[1] = 0.0;
            rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
            action_goal.pose.header.frame_id = "map";
            action_goal.pose.pose.position.x = 0.0;
            action_goal.pose.pose.position.y = 0.0;
            action_goal.pose.pose.position.z = cruise_height_;
            action_goal.pose.pose.orientation.x = 0.0;
            action_goal.pose.pose.orientation.y = 0.0;
            action_goal.pose.pose.orientation.z = 0.0;
            action_goal.pose.pose.orientation.w = 1.0;
            navigate_to_pose_client_->async_send_goal(action_goal);
            RCLCPP_INFO(this->get_logger(), "返回起点，current x y: %lf, %lf", current_x_, current_y_);
        }
        if_nav = true;
        if_turning = false;
    }
    else if(current_step == 72)
    {
        obstacle_height_ = 1.4;
        current_target_position_.transform.translation.x = passing_door_src_1_[0];
        current_target_position_.transform.translation.y = passing_door_src_1_[1];
        current_target_position_.transform.translation.z = passing_door_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        current_passing_door_ = true;
        if_turning = true; //转向状态

        current_nav_mode = 1;
        if (current_nav_mode != last_nav_mode)
        {
            change_mode();
            last_nav_mode  = current_nav_mode;
        }
        RCLCPP_INFO(this->get_logger(), "穿门前转向中...");
    }
    else if(current_step == 73)
    {
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
        action_goal.pose.header.frame_id = "map";
        action_goal.pose.pose.position.x = passing_door_src_2_[0];
        action_goal.pose.pose.position.y = passing_door_src_2_[1];
        action_goal.pose.pose.position.z = passing_door_height_;
        action_goal.pose.pose.orientation.x = 0.0;
        action_goal.pose.pose.orientation.y = 0.0;
        action_goal.pose.pose.orientation.z = 0.0;
        action_goal.pose.pose.orientation.w = 1.0;
        navigate_to_pose_client_->async_send_goal(action_goal);
        if_nav = true;
        current_passing_door_ = true;
        if_turning = false;
        RCLCPP_INFO(this->get_logger(), "穿门中间点: %lf, %lf", passing_door_src_2_[0], passing_door_src_2_[1]);
    }
    else if(current_step == 74)
    {
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
        action_goal.pose.header.frame_id = "map";
        action_goal.pose.pose.position.x = passing_door_des_[0];
        action_goal.pose.pose.position.y = passing_door_des_[1];
        action_goal.pose.pose.position.z = passing_door_height_;
        action_goal.pose.pose.orientation.x = 0.0;
        action_goal.pose.pose.orientation.y = 0.0;
        action_goal.pose.pose.orientation.z = 0.0;
        action_goal.pose.pose.orientation.w = 1.0;
        navigate_to_pose_client_->async_send_goal(action_goal);
        if_nav = true;
        current_passing_door_ = true;
        if_turning = false;
        RCLCPP_INFO(this->get_logger(), "穿门终点: %lf, %lf", passing_door_des_[0], passing_door_des_[1]);
    }
    else if(current_step == 75)
    {
        current_target_position_.transform.translation.x = 0.0;
        current_target_position_.transform.translation.y = 0.0;
        current_target_position_.transform.translation.z = cruise_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        current_passing_door_ = false;
        if_turning = false;
    }
    else if(current_step == 81)
    {
        current_target_position_.transform.translation.x = 0.0;
        current_target_position_.transform.translation.y = 0.0;
        if(current_z_ + 0.3 >= 0.5)
            current_target_position_.transform.translation.z = current_z_ - 0.5;
        else
            current_target_position_.transform.translation.z = -0.3;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        current_passing_door_ = if_passing_door_;
        if_turning = false;
        RCLCPP_INFO(this->get_logger(), "降落中...");
    }
    else if(current_step == 82)
    {
        current_target_position_.transform.translation.x = passing_door_des_[0];
        current_target_position_.transform.translation.y = passing_door_des_[1];
        if(current_z_ + 0.3 >= 0.5)
            current_target_position_.transform.translation.z = current_z_ - 0.5;
        else
            current_target_position_.transform.translation.z = -0.3;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        current_passing_door_ = if_passing_door_;
        if_turning = false;
        RCLCPP_INFO(this->get_logger(), "穿门后降落中...");
    }
    std_msgs::msg::Bool nav_state_msg;
    nav_state_msg.data = if_nav;
    nav_state_pub_->publish(nav_state_msg);
    // RCLCPP_INFO(this->get_logger(), "<<<<<<<<<<<<<<<<<<<<<<< if_nav: %d", if_nav);
    std_msgs::msg::Bool passing_door_state_msg;
    passing_door_state_msg.data = current_passing_door_;
    passing_door_state_pub_->publish(passing_door_state_msg);
    // RCLCPP_INFO(this->get_logger(), "/////////////////////// current_passing_door_: %d", current_passing_door_);
    std_msgs::msg::Bool turning_state_msg;
    turning_state_msg.data = if_turning;
    turning_state_pub_->publish(turning_state_msg);
    // RCLCPP_INFO(this->get_logger(), "////////......... if_turning: %d", if_turning);
    std_msgs::msg::Float64 obstacle_height_msg;
    obstacle_height_msg.data = obstacle_height_;
    obstacle_height_pub_->publish(obstacle_height_msg);
}

void BehaviorControl::ArmStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    arming_state = msg->data;
    RCLCPP_INFO(this->get_logger(), ">>>>>>>>>>>>>>>>>>>>>>>>arming_state: %d", arming_state);
}

void BehaviorControl::set_parameter()
{
    // 构建请求
    auto servo_server_request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

    rcl_interfaces::msg::Parameter servo_server_param_servo_index;
    servo_server_param_servo_index.name = "/servo/servo";
    servo_server_param_servo_index.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    servo_server_param_servo_index.value.integer_value = servo_index_;

    servo_server_request->parameters.push_back(servo_server_param_servo_index);

    // 使用回调的异步调用
    auto servo_server_future = servo_parameter_client_->async_send_request(
        servo_server_request,
        [this](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future) {
            this->handle_parameter_response(future);
        });
}

void BehaviorControl::handle_parameter_response(
        rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future)
{
    try {
        auto response = future.get();
        for (const auto& result : response->results) {
            if (result.successful) {
                RCLCPP_INFO(this->get_logger(), ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Parameter set successfully");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to set parameter: %s",
                            result.reason.c_str());
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
    }
}

void BehaviorControl::change_mode()
{
    /* 修改FollowPath中的参数 */
    // 构建请求
    auto controller_server_request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

    rcl_interfaces::msg::Parameter controller_server_param_max_vel_theta;
    controller_server_param_max_vel_theta.name = "FollowPath.max_vel_theta";
    controller_server_param_max_vel_theta.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::Parameter controller_server_param_max_vel_x;
    controller_server_param_max_vel_x.name = "FollowPath.max_vel_x";
    controller_server_param_max_vel_x.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::Parameter controller_server_param_max_vel_y;
    controller_server_param_max_vel_y.name = "FollowPath.max_vel_y";
    controller_server_param_max_vel_y.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::Parameter controller_server_param_max_vel_x_backwards;
    controller_server_param_max_vel_x_backwards.name = "FollowPath.max_vel_x_backwards";
    controller_server_param_max_vel_x_backwards.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::Parameter controller_server_param_acc_lim_x;
    controller_server_param_acc_lim_x.name = "FollowPath.acc_lim_x";
    controller_server_param_acc_lim_x.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::Parameter controller_server_param_acc_lim_y;
    controller_server_param_acc_lim_y.name = "FollowPath.acc_lim_y";
    controller_server_param_acc_lim_y.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::Parameter controller_server_param_acc_lim_theta;
    controller_server_param_acc_lim_theta.name = "FollowPath.acc_lim_theta";
    controller_server_param_acc_lim_theta.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::Parameter controller_server_param_max_global_plan_lookahead_dist;
    controller_server_param_max_global_plan_lookahead_dist.name = "FollowPath.max_global_plan_lookahead_dist";
    controller_server_param_max_global_plan_lookahead_dist.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::Parameter controller_server_param_weight_inflation;
    controller_server_param_weight_inflation.name = "FollowPath.weight_inflation";
    controller_server_param_weight_inflation.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;

    controller_server_param_max_vel_theta.value.double_value = max_vel_theta_passing;
    controller_server_param_max_vel_x.value.double_value = max_vel_x_passing;
    controller_server_param_max_vel_y.value.double_value = max_vel_y_passing;
    controller_server_param_max_vel_x_backwards.value.double_value = max_vel_x_backwards_passing;
    controller_server_param_acc_lim_x.value.double_value = acc_lim_x_passing;
    controller_server_param_acc_lim_y.value.double_value = acc_lim_y_passing;
    controller_server_param_acc_lim_theta.value.double_value = acc_lim_theta_passing;
    controller_server_param_max_global_plan_lookahead_dist.value.double_value = max_global_plan_lookahead_dist;
    controller_server_param_weight_inflation.value.double_value = weight_inflation;

    // 所有要修改的参数一起push_back
    controller_server_request->parameters.push_back(controller_server_param_max_vel_theta);
    controller_server_request->parameters.push_back(controller_server_param_max_vel_x);
    controller_server_request->parameters.push_back(controller_server_param_max_vel_y);
    controller_server_request->parameters.push_back(controller_server_param_max_vel_x_backwards);
    controller_server_request->parameters.push_back(controller_server_param_acc_lim_x);
    controller_server_request->parameters.push_back(controller_server_param_acc_lim_y);
    controller_server_request->parameters.push_back(controller_server_param_acc_lim_theta);
    controller_server_request->parameters.push_back(controller_server_param_max_global_plan_lookahead_dist);
    controller_server_request->parameters.push_back(controller_server_param_weight_inflation);

    // 使用回调的异步调用
    auto controller_server_future = controller_server_parameter_client_->async_send_request(
        controller_server_request,
        [this](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future) {
            this->handle_parameter_response(future);
        });

    /* local_costmap */
    //构建请求
    auto local_costmap_request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

    rcl_interfaces::msg::Parameter local_costmap_param_robot_radius;
    local_costmap_param_robot_radius.name = "robot_radius";
    local_costmap_param_robot_radius.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;

    local_costmap_param_robot_radius.value.double_value = robot_radius;

    local_costmap_request->parameters.push_back(local_costmap_param_robot_radius);

    // 使用回调的异步调用
    auto local_costmap_future = local_costmap_parameter_client_->async_send_request(
        local_costmap_request,
        [this](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future) {
            this->handle_parameter_response(future);
        });
}
