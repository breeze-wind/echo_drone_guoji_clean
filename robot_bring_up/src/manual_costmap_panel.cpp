#include <memory>

#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace robot_bring_up
{

class ManualCostmapPanel : public rviz_common::Panel
{
public:
  explicit ManualCostmapPanel(QWidget * parent = nullptr)
  : rviz_common::Panel(parent)
  {
    auto * layout = new QVBoxLayout(this);
    auto * explanation = new QLabel(
      "Undo removes the most recently added obstacle/free rectangle.", this);
    explanation->setWordWrap(true);
    layout->addWidget(explanation);
    undo_button_ = new QPushButton("Undo last rectangle", this);
    layout->addWidget(undo_button_);
    status_ = new QLabel("Waiting for /manual_costmap/undo", this);
    status_->setWordWrap(true);
    layout->addWidget(status_);

    QObject::connect(undo_button_, &QPushButton::clicked, [this]() {sendUndo();});
  }

  void onInitialize() override
  {
    auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
    if (!abstraction) {
      status_->setText("RViz ROS node is unavailable");
      undo_button_->setEnabled(false);
      return;
    }
    node_ = abstraction->get_raw_node();
    client_ = node_->create_client<std_srvs::srv::Trigger>("/manual_costmap/undo");
    status_->setText("Undo service: /manual_costmap/undo");
  }

private:
  void sendUndo()
  {
    if (!client_) {
      status_->setText("Undo service client is not initialized");
      return;
    }
    if (!client_->service_is_ready()) {
      status_->setText("Undo unavailable: start the navigation editor node first");
      return;
    }
    undo_button_->setEnabled(false);
    status_->setText("Undo requested...");
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        const auto response = future.get();
        const QString message = response->success ?
          QString::fromStdString(response->message) :
          QString("Undo failed: %1").arg(QString::fromStdString(response->message));
        QMetaObject::invokeMethod(
          this, [this, message]() {
            status_->setText(message);
            undo_button_->setEnabled(true);
          }, Qt::QueuedConnection);
      });
  }

  QPushButton * undo_button_{nullptr};
  QLabel * status_{nullptr};
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
};

}  // namespace robot_bring_up

PLUGINLIB_EXPORT_CLASS(robot_bring_up::ManualCostmapPanel, rviz_common::Panel)
