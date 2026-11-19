#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <behaviortree_cpp/action_node.h>

namespace slope_bt_nodes
{

class SlopeUpNode : public BT::StatefulActionNode
{
public:

  SlopeUpNode(
    const std::string& name,
    const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:

  enum class State
  {
    WAIT_SLOPE,
    CLIMBING
  };

  void imuCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg);

  void odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<
    sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Subscription<
    nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  State state_;

  bool odom_initialized_;

  double pitch_deg_;

  double current_x_;
  double current_y_;

  double start_x_;
  double start_y_;

  double climb_start_dist_;
};

}
