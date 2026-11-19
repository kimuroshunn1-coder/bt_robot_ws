#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <behaviortree_cpp/action_node.h>

namespace slope_bt_nodes
{

// 前進し続ける。速度は加速度制限つきで上げる(急に動かすと、加速度でIMUの傾きの推定が乱れるため)。
// ポート: topic(既定 /cmd_vel), speed(既定 0.2 m/s), accel(既定 0.3 m/s^2、0で制限なし)
class MoveForwardNode : public BT::StatefulActionNode
{
public:

  MoveForwardNode(
    const std::string& name,
    const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:

  void ensurePublisher(const std::string& topic);

  void publishCmd(double vx);

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<
    geometry_msgs::msg::Twist>::SharedPtr pub_;

  std::string topic_;

  double speed_ = 0.2;
  double accel_ = 0.3;
  double cmd_ = 0.0;

  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
};

}
