#include "merge_pcd/merge_pcd.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor;
    rclcpp::Node::SharedPtr node = std::make_shared<MergePCD>();

    executor.add_node(node->get_node_base_interface());

    executor.spin_once();

    rclcpp::shutdown();

    return 0;
}
