#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <behaviortree_cpp/action_node.h>

namespace slope_bt_nodes
{

class AngleStateNode : public BT::StatefulActionNode
{
public:

  AngleStateNode(
    const std::string& name,
    const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:

  void imuCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg);

  void publishCmd(double wz);

  double normalizeAngle(double deg);

  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<
    sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<
    geometry_msgs::msg::Twist>::SharedPtr pub_;

  double yaw_deg_ = 0.0;

  bool imu_received_ = false;

  double target_deg_ = 0.0;
};

}
