#include "behavior_control/behavior_control.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<BehaviorControl>("behavior_control");
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
  return 0;
}