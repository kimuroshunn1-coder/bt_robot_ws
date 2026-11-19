#include "slope_bt_nodes/move_forward_node.hpp"

#include <algorithm>

#include <behaviortree_cpp/bt_factory.h>

namespace slope_bt_nodes
{

MoveForwardNode::MoveForwardNode(
  const std::string& name,
  const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
  // Gazebo の時計(/clock)で動かす
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);

  node_ =
    rclcpp::Node::make_shared(
      "move_forward_bt_node",
      options);
}

BT::PortsList
MoveForwardNode::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "topic", "/cmd_vel",
      "速度指令のトピック(geometry_msgs/Twist)"),
    BT::InputPort<double>(
      "speed", 0.2,
      "前進速度 [m/s]"),
    BT::InputPort<double>(
      "accel", 0.3,
      "加速度の上限 [m/s^2](0で制限なし)")
  };
}

void
MoveForwardNode::ensurePublisher(
  const std::string& topic)
{
  if (pub_ && topic == topic_)
  {
    return;
  }

  topic_ = topic;

  pub_ =
    node_->create_publisher<
      geometry_msgs::msg::Twist>(
        topic_,
        10);
}

void
MoveForwardNode::publishCmd(double vx)
{
  geometry_msgs::msg::Twist msg;

  msg.linear.x = vx;
  msg.angular.z = 0.0;

  pub_->publish(msg);
}

BT::NodeStatus
MoveForwardNode::onStart()
{
  std::string topic = "/cmd_vel";

  getInput("topic", topic);
  getInput("speed", speed_);
  getInput("accel", accel_);

  ensurePublisher(topic);

  cmd_ = 0.0;

  rclcpp::spin_some(node_);

  last_time_ = node_->now();

  RCLCPP_INFO(
    node_->get_logger(),
    "MoveForward START (topic=%s, speed=%.2f, accel=%.2f)",
    topic_.c_str(), speed_, accel_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus
MoveForwardNode::onRunning()
{
  rclcpp::spin_some(node_);

  const rclcpp::Time now = node_->now();

  double dt = (now - last_time_).seconds();

  last_time_ = now;

  if (dt < 0.0 || dt > 1.0)
  {
    dt = 0.0;
  }

  if (accel_ > 0.0)
  {
    const double step = accel_ * dt;

    if (cmd_ < speed_)
    {
      cmd_ = std::min(speed_, cmd_ + step);
    }
    else
    {
      cmd_ = std::max(speed_, cmd_ - step);
    }
  }
  else
  {
    cmd_ = speed_;
  }

  publishCmd(cmd_);

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(),
    *node_->get_clock(),
    2000,
    "MoveForward RUNNING (cmd=%.2f m/s)",
    cmd_);

  return BT::NodeStatus::RUNNING;
}

void
MoveForwardNode::onHalted()
{
  cmd_ = 0.0;

  if (pub_)
  {
    for (int i = 0; i < 10; i++)
    {
      publishCmd(0.0);

      rclcpp::sleep_for(
        std::chrono::milliseconds(20));
    }
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "MoveForward HALTED");
}

} // namespace slope_bt_nodes
