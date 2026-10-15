#ifndef ROBOT_BRING_UP__MANUAL_COSTMAP_LAYER_HPP_
#define ROBOT_BRING_UP__MANUAL_COSTMAP_LAYER_HPP_

#include <nav2_costmap_2d/costmap_layer.hpp>
#include <robot_interfaces/msg/costmap_rect_edit.hpp>
#include <mutex>
#include <vector>

namespace manual_costmap
{

class ManualCostmapLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  void onInitialize() override;
  void matchSize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;

private:
  struct Rectangle
  {
    double min_x;
    double min_y;
    double max_x;
    double max_y;
    unsigned char cost;
  };

  void editCallback(const robot_interfaces::msg::CostmapRectEdit::SharedPtr msg);

  std::mutex mutex_;
  std::vector<Rectangle> rectangles_;
  rclcpp::Subscription<robot_interfaces::msg::CostmapRectEdit>::SharedPtr edit_sub_;
};

}  // namespace manual_costmap

#endif  // ROBOT_BRING_UP__MANUAL_COSTMAP_LAYER_HPP_
