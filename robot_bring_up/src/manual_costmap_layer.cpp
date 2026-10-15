#include "robot_bring_up/manual_costmap_layer.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <nav2_costmap_2d/cost_values.hpp>

#include <algorithm>
#include <cmath>

namespace manual_costmap
{

void ManualCostmapLayer::onInitialize()
{
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node_->get_parameter(getFullName("enabled"), enabled_);
  current_ = true;
  matchSize();

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1000)).reliable().transient_local();
  edit_sub_ = rclcpp_node_->create_subscription<robot_interfaces::msg::CostmapRectEdit>(
    "/manual_costmap/edit_rect", qos,
    std::bind(&ManualCostmapLayer::editCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    rclcpp_node_->get_logger(),
    "Manual rectangular costmap layer ready; edits=/manual_costmap/edit_rect");
}

void ManualCostmapLayer::matchSize()
{
  if (layered_costmap_ != nullptr) {
    nav2_costmap_2d::Costmap2D * master = layered_costmap_->getCostmap();
    if (master != nullptr) {
      resizeMap(
        master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
        master->getOriginX(), master->getOriginY());
    }
  }
}

void ManualCostmapLayer::editCallback(
  const robot_interfaces::msg::CostmapRectEdit::SharedPtr msg)
{
  if (msg->operation == robot_interfaces::msg::CostmapRectEdit::UNDO) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (rectangles_.empty()) {
      RCLCPP_WARN(rclcpp_node_->get_logger(), "Ignoring manual costmap undo: edit history is empty");
      return;
    }
    const auto rect = rectangles_.back();
    rectangles_.pop_back();
    addExtraBounds(rect.min_x, rect.min_y, rect.max_x, rect.max_y);
    RCLCPP_WARN(
      rclcpp_node_->get_logger(), "Undid last manual rectangle; %zu edits remain",
      rectangles_.size());
    return;
  }
  const std::string expected_frame = layered_costmap_->getGlobalFrameID();
  if (!msg->header.frame_id.empty() && msg->header.frame_id != expected_frame) {
    RCLCPP_ERROR(
      rclcpp_node_->get_logger(), "Ignoring manual rectangle in frame '%s'; expected '%s'",
      msg->header.frame_id.c_str(), expected_frame.c_str());
    return;
  }
  const auto & a = msg->corner_a;
  const auto & b = msg->corner_b;
  const unsigned char cost = msg->operation == robot_interfaces::msg::CostmapRectEdit::ADD_OBSTACLE ?
    nav2_costmap_2d::LETHAL_OBSTACLE : nav2_costmap_2d::FREE_SPACE;
  if (msg->operation != robot_interfaces::msg::CostmapRectEdit::ADD_OBSTACLE &&
    msg->operation != robot_interfaces::msg::CostmapRectEdit::FORCE_FREE)
  {
    RCLCPP_ERROR(rclcpp_node_->get_logger(), "Ignoring manual rectangle with unknown operation %u", msg->operation);
    return;
  }
  Rectangle rect{
    std::min(a.x, b.x), std::min(a.y, b.y),
    std::max(a.x, b.x), std::max(a.y, b.y), cost};
  if (rect.max_x - rect.min_x < 0.02 || rect.max_y - rect.min_y < 0.02) {
    RCLCPP_WARN(rclcpp_node_->get_logger(), "Ignoring manual rectangle smaller than 2 cm");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    rectangles_.push_back(rect);
  }
  addExtraBounds(rect.min_x, rect.min_y, rect.max_x, rect.max_y);
  const char * action = cost == nav2_costmap_2d::LETHAL_OBSTACLE ? "obstacle" : "forced free";
  RCLCPP_WARN(
    rclcpp_node_->get_logger(), "Manual %s rectangle [%.2f, %.2f] - [%.2f, %.2f] in frame '%s'",
    action, rect.min_x, rect.min_y, rect.max_x, rect.max_y, msg->header.frame_id.c_str());
}

void ManualCostmapLayer::updateBounds(
  double, double, double, double * min_x, double * min_y, double * max_x, double * max_y)
{
  useExtraBounds(min_x, min_y, max_x, max_y);
  std::lock_guard<std::mutex> lock(mutex_);
  // Re-apply persistent world rectangles as a rolling local costmap moves.
  for (const auto & rect : rectangles_) {
    *min_x = std::min(*min_x, rect.min_x);
    *min_y = std::min(*min_y, rect.min_y);
    *max_x = std::max(*max_x, rect.max_x);
    *max_y = std::max(*max_y, rect.max_y);
  }
}

void ManualCostmapLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const double resolution = master_grid.getResolution();
  const double origin_x = master_grid.getOriginX();
  const double origin_y = master_grid.getOriginY();
  for (const auto & rect : rectangles_) {
    const int rect_min_i = std::max(
      min_i, static_cast<int>(std::floor((rect.min_x - origin_x) / resolution)));
    const int rect_max_i = std::min(
      max_i, static_cast<int>(std::ceil((rect.max_x - origin_x) / resolution)) + 1);
    const int rect_min_j = std::max(
      min_j, static_cast<int>(std::floor((rect.min_y - origin_y) / resolution)));
    const int rect_max_j = std::min(
      max_j, static_cast<int>(std::ceil((rect.max_y - origin_y) / resolution)) + 1);
    for (int j = rect_min_j; j < rect_max_j; ++j) {
      for (int i = rect_min_i; i < rect_max_i; ++i) {
        double wx = 0.0;
        double wy = 0.0;
        master_grid.mapToWorld(static_cast<unsigned int>(i), static_cast<unsigned int>(j), wx, wy);
        if (wx >= rect.min_x && wx <= rect.max_x && wy >= rect.min_y && wy <= rect.max_y) {
          master_grid.setCost(static_cast<unsigned int>(i), static_cast<unsigned int>(j), rect.cost);
        }
      }
    }
  }
}

void ManualCostmapLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  // Costmap clearing services should not silently discard the operator's edits.
  for (const auto & rect : rectangles_) {
    addExtraBounds(rect.min_x, rect.min_y, rect.max_x, rect.max_y);
  }
  current_ = true;
}

}  // namespace manual_costmap

PLUGINLIB_EXPORT_CLASS(manual_costmap::ManualCostmapLayer, nav2_costmap_2d::Layer)
