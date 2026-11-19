#include "slope_bt_nodes/follow_waypoints_nav2_node.hpp"

#include <csignal>
#include <cstdlib>
#include <iostream>
#include <thread>

#include <sys/wait.h>
#include <unistd.h>

namespace slope_bt_nodes
{

FollowWaypointsNav2Node::FollowWaypointsNav2Node(
  const std::string& name,
  const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
}

FollowWaypointsNav2Node::~FollowWaypointsNav2Node()
{
  stopChild();
}

BT::PortsList
FollowWaypointsNav2Node::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "file", "",
      "ウェイポイントのyaml(絶対パス)"),
    BT::InputPort<std::string>(
      "points", "",
      "ファイル内の n 番目(1始まり)だけを、この順に巡る。例: 5 2 3 4 5"),
    BT::InputPort<std::string>(
      "initial_pose_world", "",
      "AMCLの初期位置 'x y yaw_deg'(世界座標)。空なら送らない"),
    BT::InputPort<bool>(
      "send_initial_pose", false,
      "初期位置が空のとき、スポーン位置(0,0,0)を送るか"),
    BT::InputPort<double>(
      "timeout", 600.0,
      "全体の制限時間 [s]")
  };
}

std::vector<std::string>
FollowWaypointsNav2Node::splitTokens(
  const std::string& text)
{
  std::vector<std::string> out;

  std::string cur;

  for (const char c : text)
  {
    if (c == ' ' || c == ',' || c == '\t')
    {
      if (!cur.empty())
      {
        out.push_back(cur);
        cur.clear();
      }
    }
    else
    {
      cur += c;
    }
  }

  if (!cur.empty())
  {
    out.push_back(cur);
  }

  return out;
}

BT::NodeStatus
FollowWaypointsNav2Node::onStart()
{
  std::string file;
  std::string points;
  std::string pose;
  bool send_initial = false;

  getInput("file", file);
  getInput("points", points);
  getInput("initial_pose_world", pose);
  getInput("send_initial_pose", send_initial);
  getInput("timeout", timeout_);

  if (file.empty())
  {
    std::cerr << "[FollowWaypointsNav2] file が指定されていません" << std::endl;

    return BT::NodeStatus::FAILURE;
  }

  std::vector<std::string> args = {
    "ros2", "run", "my_robot_sim", "follow_waypoints.py",
    "--file", file};

  const auto pts = splitTokens(points);

  if (!pts.empty())
  {
    args.push_back("--points");

    for (const auto& p : pts)
    {
      args.push_back(p);
    }
  }

  const auto pose_tokens = splitTokens(pose);

  if (pose_tokens.size() == 3)
  {
    args.push_back("--initial-pose-world");

    for (const auto& t : pose_tokens)
    {
      args.push_back(t);
    }
  }
  else if (!send_initial)
  {
    // 階段などで動いたあとに、位置推定をスポーン位置へ戻さない
    args.push_back("--no-initial-pose");
  }

  args.push_back("--ros-args");
  args.push_back("-p");
  args.push_back("use_sim_time:=true");

  std::vector<char*> argv;

  for (auto& a : args)
  {
    argv.push_back(const_cast<char*>(a.c_str()));
  }

  argv.push_back(nullptr);

  pid_ = fork();

  if (pid_ < 0)
  {
    std::cerr << "[FollowWaypointsNav2] fork に失敗しました" << std::endl;

    pid_ = -1;

    return BT::NodeStatus::FAILURE;
  }

  if (pid_ == 0)
  {
    // 子プロセス: 独立したプロセスグループにして、haltのときにまとめてCtrl+C相当を送れるようにする
    setpgid(0, 0);

    execvp("ros2", argv.data());

    _exit(127);
  }

  t_start_ = std::chrono::steady_clock::now();

  std::cout << "[FollowWaypointsNav2] START: " << file << std::endl;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus
FollowWaypointsNav2Node::onRunning()
{
  int status = 0;

  const pid_t r = waitpid(pid_, &status, WNOHANG);

  if (r == 0)
  {
    const double elapsed =
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t_start_).count();

    if (timeout_ > 0.0 && elapsed > timeout_)
    {
      std::cerr << "[FollowWaypointsNav2] タイムアウト" << std::endl;

      stopChild();

      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
  }

  pid_ = -1;

  if (r < 0)
  {
    return BT::NodeStatus::FAILURE;
  }

  if (WIFEXITED(status) && WEXITSTATUS(status) == 0)
  {
    std::cout << "[FollowWaypointsNav2] SUCCESS" << std::endl;

    return BT::NodeStatus::SUCCESS;
  }

  std::cerr << "[FollowWaypointsNav2] FAILURE (終了コード "
            << (WIFEXITED(status) ? WEXITSTATUS(status) : -1) << ")"
            << std::endl;

  return BT::NodeStatus::FAILURE;
}

void
FollowWaypointsNav2Node::onHalted()
{
  stopChild();

  std::cout << "[FollowWaypointsNav2] HALTED" << std::endl;
}

void
FollowWaypointsNav2Node::stopChild()
{
  if (pid_ <= 0)
  {
    return;
  }

  // Ctrl+C を送ると、follow_waypoints.py が Nav2 の走行をキャンセルして終了する
  killpg(pid_, SIGINT);

  int status = 0;

  for (int i = 0; i < 50; i++)
  {
    if (waitpid(pid_, &status, WNOHANG) != 0)
    {
      pid_ = -1;

      return;
    }

    std::this_thread::sleep_for(
      std::chrono::milliseconds(100));
  }

  killpg(pid_, SIGKILL);

  waitpid(pid_, &status, 0);

  pid_ = -1;
}

} // namespace slope_bt_nodes
