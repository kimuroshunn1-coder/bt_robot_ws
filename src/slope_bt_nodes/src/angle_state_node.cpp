#include "slope_bt_nodes/angle_state_node.hpp"

#include <cmath>
#include <algorithm>

#include <behaviortree_cpp/bt_factory.h>

namespace slope_bt_nodes
{

AngleStateNode::AngleStateNode(
  const std::string& name,
  const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
  node_ =
    rclcpp::Node::make_shared(
      "angle_state_bt_node");

  imu_sub_ =
    node_->create_subscription<
      sensor_msgs::msg::Imu>(
        "/imu/data",
        10,
        std::bind(
          &AngleStateNode::imuCallback,
          this,
          std::placeholders::_1));

  pub_ =
    node_->create_publisher<
      geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10);
}

BT::PortsList
AngleStateNode::providedPorts()
{
  return {
    BT::InputPort<double>(
      "angle",
      0.0,
      "target yaw [deg]")
  };
}

void
AngleStateNode::imuCallback(
  const sensor_msgs::msg::Imu::SharedPtr msg)
{
  const double qx = msg->orientation.x;
  const double qy = msg->orientation.y;
  const double qz = msg->orientation.z;
  const double qw = msg->orientation.w;

  const double siny =
    2.0 * (qw * qz + qx * qy);

  const double cosy =
    1.0 - 2.0 * (qy * qy + qz * qz);

  yaw_deg_ =
    std::atan2(siny, cosy)
    * 180.0 / M_PI;

  imu_received_ = true;
}

double
AngleStateNode::normalizeAngle(double deg)
{
  while (deg > 180.0)
  {
    deg -= 360.0;
  }

  while (deg < -180.0)
  {
    deg += 360.0;
  }

  return deg;
}

void
AngleStateNode::publishCmd(double wz)
{
  geometry_msgs::msg::Twist msg;

  msg.linear.x = 0.0;
  msg.angular.z = wz;

  pub_->publish(msg);
}

BT::NodeStatus
AngleStateNode::onStart()
{
  getInput("angle", target_deg_);

  imu_received_ = false;

  RCLCPP_INFO(
    node_->get_logger(),
    "AngleState START target=%.1f",
    target_deg_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus
AngleStateNode::onRunning()
{
  rclcpp::spin_some(node_);

  if (!imu_received_)
  {
    return BT::NodeStatus::RUNNING;
  }

  double error =
    normalizeAngle(
      target_deg_ - yaw_deg_);

  if (std::abs(error) < 1.0)
  {
    for (int i = 0; i < 10; i++)
    {
      publishCmd(0.0);

      rclcpp::sleep_for(
        std::chrono::milliseconds(20));
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "AngleState SUCCESS yaw=%.2f",
      yaw_deg_);

    return BT::NodeStatus::SUCCESS;
  }

  double wz =
    0.02 * error;

  wz =
    std::clamp(
      wz,
      -0.30,
      0.30);

  if (std::abs(wz) < 0.05)
  {
    wz =
      std::copysign(
        0.05,
        wz);
  }

  publishCmd(wz);

  return BT::NodeStatus::RUNNING;
}

void
AngleStateNode::onHalted()
{
  publishCmd(0.0);

  RCLCPP_INFO(
    node_->get_logger(),
    "AngleState HALTED");
}

}
