#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/xml_parsing.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <chrono>
#include <thread>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

using namespace std::chrono_literals;

rclcpp::Node::SharedPtr g_node;
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr g_pub;

class StartNode : public BT::SyncActionNode
{
public:
  StartNode(
      const std::string& name,
      const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    std::cout << "Start" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

class ForwardNode : public BT::StatefulActionNode
{
public:
  ForwardNode(
      const std::string& name,
      const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus onStart() override
  {
    start_time_ = std::chrono::steady_clock::now();

    geometry_msgs::msg::TwistStamped cmd;

    cmd.header.stamp = g_node->now();
    cmd.header.frame_id = "base_link";

    cmd.twist.linear.x = 0.2;

    g_pub->publish(cmd);

    std::cout << "Forward" << std::endl;

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    auto now = std::chrono::steady_clock::now();

    if(now - start_time_ > 3s)
    {
      geometry_msgs::msg::TwistStamped stop;

      stop.header.stamp = g_node->now();
      stop.header.frame_id = "base_link";

      g_pub->publish(stop);

      return BT::NodeStatus::SUCCESS;
    }

    geometry_msgs::msg::TwistStamped cmd;

    cmd.header.stamp = g_node->now();
    cmd.header.frame_id = "base_link";
    cmd.twist.linear.x = 0.2;

    g_pub->publish(cmd);

    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  std::chrono::steady_clock::time_point start_time_;
};

class BackwardNode : public BT::StatefulActionNode
{
public:
  BackwardNode(
      const std::string& name,
      const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus onStart() override
  {
    start_time_ = std::chrono::steady_clock::now();

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = g_node->now();
    cmd.header.frame_id = "base_link";
    cmd.twist.linear.x = 0.2;

    g_pub->publish(cmd);

    std::cout << "Backward" << std::endl;

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    auto now = std::chrono::steady_clock::now();

    if(now - start_time_ > 3s)
    {
      geometry_msgs::msg::TwistStamped stop;

      stop.header.stamp = g_node->now();
      stop.header.frame_id = "base_link";

      g_pub->publish(stop);

      return BT::NodeStatus::SUCCESS;
    }

    geometry_msgs::msg::TwistStamped cmd;

    cmd.header.stamp = g_node->now();
    cmd.header.frame_id = "base_link";
    cmd.twist.linear.x = -0.2;

    g_pub->publish(cmd);

    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  std::chrono::steady_clock::time_point start_time_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  g_node =
      rclcpp::Node::make_shared(
          "bt_robot_demo");

  g_pub =
      g_node->create_publisher<
          geometry_msgs::msg::TwistStamped>(
          "/diff_drive_controller/cmd_vel",
          10);

  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<StartNode>(
      "StartNode");

  factory.registerNodeType<ForwardNode>(
      "ForwardNode");

  factory.registerNodeType<BackwardNode>(
      "BackwardNode");

  auto xml_file =
      ament_index_cpp::get_package_share_directory(
          "bt_robot_demo")
      + "/config/tree.xml";

  auto tree =
      factory.createTreeFromFile(
          xml_file);

  BT::Groot2Publisher groot_monitor(tree, 1667);

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(g_node);

  while(rclcpp::ok())
  {
    tree.tickOnce();
    executor.spin_some();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(100));
  }

  rclcpp::shutdown();

  return 0;
}
