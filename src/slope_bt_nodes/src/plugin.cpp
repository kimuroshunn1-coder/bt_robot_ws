#include <behaviortree_cpp/bt_factory.h>

#include "slope_bt_nodes/move_forward_node.hpp"
#include "slope_bt_nodes/slope_up_node.hpp"
#include "slope_bt_nodes/slope_down_node.hpp"
#include "slope_bt_nodes/move_back_node.hpp"
#include <slope_bt_nodes/follow_waypoints_nav2_node.hpp>
#include <slope_bt_nodes/angle_state_node.hpp>
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<
    slope_bt_nodes::MoveForwardNode>(
      "MoveForward");

  factory.registerNodeType<
    slope_bt_nodes::SlopeUpNode>(
      "SlopeUp");

  factory.registerNodeType<
    slope_bt_nodes::SlopeDownNode>(
      "SlopeDown");

  factory.registerNodeType<
    slope_bt_nodes::MoveBackNode>(
      "MoveBack");

  factory.registerNodeType<
    slope_bt_nodes::FollowWaypointsNav2Node>(
      "FollowWaypointsNav2");

  factory.registerNodeType<
    slope_bt_nodes::AngleStateNode>(
      "AngleState");

}
