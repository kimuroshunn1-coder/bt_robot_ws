#include "slope_bt_nodes/move_forward_node.hpp"

#include <behaviortree_cpp/bt_factory.h>

namespace slope_bt_nodes
{

MoveForwardNode::MoveForwardNode(
  const std::string& name,
  const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
  node_ =
    rclcpp::Node::make_shared(
      "move_forward_bt_node");

  pub_ =
    node_->create_publisher<
      geometry_msgs::msg::Twist>(
        "/diff_drive_controller/cmd_vel",
        10);
}

BT::PortsList
MoveForwardNode::providedPorts()
{
  return {};
}

BT::NodeStatus
MoveForwardNode::onStart()
{
  RCLCPP_INFO(
    node_->get_logger(),
    "MoveForward START");

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus
MoveForwardNode::onRunning()
{
  RCLCPP_INFO(
    node_->get_logger(),
    "MoveForward RUNNING");

  geometry_msgs::msg::Twist msg;

  msg.linear.x = 0.20;
  msg.angular.z = 0.0;

  pub_->publish(msg);

  return BT::NodeStatus::RUNNING;
}

void
MoveForwardNode::onHalted()
{
  geometry_msgs::msg::Twist msg;

  msg.linear.x = 0.0;
  msg.angular.z = 0.0;

  for (int i = 0; i < 10; i++)
  {
    pub_->publish(msg);

    rclcpp::sleep_for(
      std::chrono::milliseconds(20));
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "MoveForward HALTED");
}

} // namespace slope_bt_nodes
