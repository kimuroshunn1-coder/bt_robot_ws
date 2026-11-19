#include "slope_bt_nodes/slope_down_node.hpp"
#include <cmath>
#include <behaviortree_cpp/bt_factory.h>
#include <slope_bt_nodes/move_forward_node.hpp>

namespace slope_bt_nodes
{

SlopeDownNode::SlopeDownNode(
  const std::string& name,
  const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
  node_ =
    rclcpp::Node::make_shared(
      "slope_down_bt_node");

  state_ =
    State::WAIT_SLOPE;

  odom_initialized_ = false;

  pitch_deg_ = 0.0;

  current_x_ = 0.0;
  current_y_ = 0.0;

  start_x_ = 0.0;
  start_y_ = 0.0;

  climb_start_dist_ = 0.0;

  imu_sub_ =
    node_->create_subscription<
      sensor_msgs::msg::Imu>(
        "/imu/data",
        10,
        std::bind(
          &SlopeDownNode::imuCallback,
          this,
          std::placeholders::_1));

  odom_sub_ =
    node_->create_subscription<
      nav_msgs::msg::Odometry>(
        "/odom",
        10,
        std::bind(
          &SlopeDownNode::odomCallback,
          this,
          std::placeholders::_1));
}

BT::PortsList
SlopeDownNode::providedPorts()
{
  return {};
}

void SlopeDownNode::imuCallback(
  const sensor_msgs::msg::Imu::SharedPtr msg)
{
  double qx = msg->orientation.x;
  double qy = msg->orientation.y;
  double qz = msg->orientation.z;
  double qw = msg->orientation.w;

  double sinp =
    2.0 * (qw * qy - qz * qx);

  if (std::abs(sinp) >= 1.0)
  {
    pitch_deg_ =
      std::copysign(
        90.0,
        sinp);
  }
  else
  {
    pitch_deg_ =
      std::asin(sinp)
      * 180.0 / M_PI;
  }
}

void SlopeDownNode::odomCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_x_ =
    msg->pose.pose.position.x;

  current_y_ =
    msg->pose.pose.position.y;

  if (!odom_initialized_)
  {
    start_x_ = current_x_;
    start_y_ = current_y_;

    odom_initialized_ = true;
  }
}

BT::NodeStatus
SlopeDownNode::onStart()
{

  state_ = State::WAIT_SLOPE;

  odom_initialized_ = false;

  pitch_deg_ = 0.0;

  current_x_ = 0.0;
  current_y_ = 0.0;

  start_x_ = 0.0;
  start_y_ = 0.0;

  climb_start_dist_ = 0.0;

  rclcpp::spin_some(node_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus
SlopeDownNode::onRunning()
{
  rclcpp::spin_some(node_);

  if (!odom_initialized_)
  {
    return BT::NodeStatus::RUNNING;
  }

  double dist =
    std::hypot(
      current_x_ - start_x_,
      current_y_ - start_y_);

  switch(state_)
  {
    case State::WAIT_SLOPE:

      if (pitch_deg_ < -5.0)
      {
        climb_start_dist_ =
          dist;

        state_ =
          State::DESCENDING;

        RCLCPP_INFO(
          node_->get_logger(),
          "DESCENDING START");
      }

      return BT::NodeStatus::RUNNING;

    case State::DESCENDING:
    {
      double travel =
        dist - climb_start_dist_;

      if (
        pitch_deg_ > -2.0 &&
        travel > 3.5)
      {
        RCLCPP_INFO(
          node_->get_logger(),
          "SUCCESS");

        return BT::NodeStatus::SUCCESS;
      }

      return BT::NodeStatus::RUNNING;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void SlopeDownNode::onHalted()
{
  RCLCPP_INFO(
    node_->get_logger(),
    "HALTED");
}

} // namespace slope_bt_nodes

