#include "pcd2pgm/pcd2pgm.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<Pcd2Pgm>("pcd2pgm_node");

    executor.add_node(node->get_node_base_interface());

    executor.spin_once();

    rclcpp::shutdown();

    return 0;
}
