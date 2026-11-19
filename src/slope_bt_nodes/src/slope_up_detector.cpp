#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"

class SlopeDetector : public rclcpp::Node
{
public:

  enum class State
  {
    WAIT_SLOPE,
    CLIMBING,
    SUCCESS
  };

  SlopeDetector()
  : Node("slope_up_detector")
  {
    imu_sub_ =
      this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data",
        10,
        std::bind(
          &SlopeDetector::imuCallback,
          this,
          std::placeholders::_1));

    odom_sub_ =
      this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        10,
        std::bind(
          &SlopeDetector::odomCallback,
          this,
          std::placeholders::_1));

    timer_ =
      this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(
          &SlopeDetector::update,
          this));

    RCLCPP_INFO(
      get_logger(),
      "WAIT_SLOPE");
  }

private:

  void imuCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    double qx = msg->orientation.x;
    double qy = msg->orientation.y;
    double qz = msg->orientation.z;
    double qw = msg->orientation.w;

    double sinp =
      2.0 * (qw * qy - qz * qx);

    if (std::abs(sinp) >= 1.0)
      pitch_deg_ =
        std::copysign(90.0, sinp);
    else
      pitch_deg_ =
        std::asin(sinp) * 180.0 / M_PI;
  }

  void odomCallback(
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

  void update()
  {
    if (!odom_initialized_)
      return;

    double dist =
      std::hypot(
        current_x_ - start_x_,
        current_y_ - start_y_);

    switch(state_)
    {
      case State::WAIT_SLOPE:

        if (pitch_deg_ < -5.0)
        {
          climb_start_dist_ = dist;

          state_ = State::CLIMBING;

          RCLCPP_INFO(
            get_logger(),
            "CLIMBING START pitch=%.2f",
            pitch_deg_);
        }

        break;

      case State::CLIMBING:
      {
        double travel =
          dist - climb_start_dist_;

        RCLCPP_INFO(
          get_logger(),
          "pitch=%.2f travel=%.2f",
          pitch_deg_,
          travel);

        if (
          pitch_deg_ > -2.0 &&
          travel > 3.5)
        {
          state_ = State::SUCCESS;

          RCLCPP_INFO(
            get_logger(),
            "SUCCESS");
        }

        break;
      }

      case State::SUCCESS:
        break;
    }
  }

  rclcpp::Subscription<
    sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Subscription<
    nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  State state_ =
    State::WAIT_SLOPE;

  bool odom_initialized_ = false;

  double pitch_deg_ = 0.0;

  double current_x_ = 0.0;
  double current_y_ = 0.0;

  double start_x_ = 0.0;
  double start_y_ = 0.0;

  double climb_start_dist_ = 0.0;
};

int main(
  int argc,
  char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<SlopeDetector>());

  rclcpp::shutdown();

  return 0;
}
