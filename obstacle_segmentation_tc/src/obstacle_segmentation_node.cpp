#include "obstacle_segmentation/obstacle_segmentation.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    rclcpp::executors::SingleThreadedExecutor executor;
    rclcpp::NodeOptions options;
    std::shared_ptr<ObstacleSegmentationNode> node = std::make_shared<ObstacleSegmentationNode>("obstacle_segmentation_node", options);
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}