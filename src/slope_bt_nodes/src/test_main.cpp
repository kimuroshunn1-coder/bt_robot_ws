#include <rclcpp/rclcpp.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include <thread>
#include <chrono>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    BT::BehaviorTreeFactory factory;

    factory.registerFromPlugin(
      "/home/kimuro/bt_robot_ws/install/slope_bt_nodes/lib/libslope_bt_plugin.so");

    auto tree =
      factory.createTreeFromFile(
        "/home/kimuro/bt_robot_ws/test_bt/slope_test.xml");

    BT::StdCoutLogger logger(tree);

    while (rclcpp::ok())
    {
     auto status = tree.tickOnce();

     if (status == BT::NodeStatus::SUCCESS)
     {
        break;
     }

     std::this_thread::sleep_for(
        std::chrono::milliseconds(100));
     }

    rclcpp::shutdown();

    return 0;
}
