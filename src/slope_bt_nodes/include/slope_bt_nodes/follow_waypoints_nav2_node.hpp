#pragma once

#include <chrono>
#include <string>
#include <vector>

#include <sys/types.h>

#include <behaviortree_cpp/action_node.h>

namespace slope_bt_nodes
{

// Nav2 でウェイポイントを巡る。my_robot_sim の follow_waypoints.py を子プロセスとして起動し、終わるまで RUNNING を返す。
//   終了コード 0 -> SUCCESS、それ以外 -> FAILURE。ツリーから外されたとき(halt)は、走行をキャンセルして止める。
// ポート:
//   file                 ウェイポイントの yaml(絶対パス)
//   points               ファイル内の n 番目(1始まり)だけを、この順に巡る。例 "5 2 3 4 5"(空なら全部)
//   initial_pose_world   AMCLの初期位置 "x y yaw_deg"(世界座標)。空なら送らない
//   send_initial_pose    initial_pose_world が空のとき、スポーン位置(0,0,0)を初期位置として送るか(既定 false)
//   timeout              全体の制限時間 [s](実時間。既定 600)
class FollowWaypointsNav2Node : public BT::StatefulActionNode
{
public:

  FollowWaypointsNav2Node(
    const std::string& name,
    const BT::NodeConfig& config);

  ~FollowWaypointsNav2Node() override;

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:

  static std::vector<std::string> splitTokens(
    const std::string& text);

  void stopChild();

  pid_t pid_ = -1;

  double timeout_ = 600.0;

  std::chrono::steady_clock::time_point t_start_;
};

}
