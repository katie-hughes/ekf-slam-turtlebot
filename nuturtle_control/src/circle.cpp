/// \file circle.cpp
/// \brief Sends cmd_vel commands to command a circular path.
///
/// PARAMETERS:
///     frequency (int): frequency of the timer, in Hz
/// PUBLISHES:
///     cmd_vel (geometry_msgs::msg::Twist): twist for robot to follow
/// SUBSCRIBES:
///    red/wheel_cmd (nuturtlebot_msgs::msg::WheelCommands): simulated controls for robot to follow
/// SERVERS:
///     control (nuturtle_control::srv::Control): Specify the radius and velocity of the desired
///             circular path.
///     reverse (std_srvs::srv::Empty): provide commands to reverse the direction of the circle
///     stop (std_srvs::srv::Empty): publish a single cmd_vel of 0, then stop publishing cmd_vel
/// CLIENTS:
///     none


#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nuturtle_control/srv/initial_pose.hpp"
#include "nuturtle_control/srv/control.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;

class Circle : public rclcpp::Node
{
public:
  Circle()
  : Node("circle")
  {
    declare_parameter("frequency", 100);
    int frequency_hz = get_parameter("frequency").as_int();
    RCLCPP_INFO_STREAM(get_logger(), "Publish frequency is " << frequency_hz << " Hz");
    auto rate = (std::chrono::milliseconds) ((int)(1000. / frequency_hz));

    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    timer_ = create_wall_timer(
      rate, std::bind(&Circle::timer_callback, this));

    control_srv_ = this->create_service<nuturtle_control::srv::Control>(
      "control",
      std::bind(&Circle::control_cb, this, std::placeholders::_1, std::placeholders::_2));

    reverse_srv_ = this->create_service<std_srvs::srv::Empty>(
      "reverse",
      std::bind(&Circle::reverse_cb, this, std::placeholders::_1, std::placeholders::_2));

    stop_srv_ = this->create_service<std_srvs::srv::Empty>(
      "stop",
      std::bind(&Circle::stop_cb, this, std::placeholders::_1, std::placeholders::_2));

  }

private:
  /// @brief Publish cmd_vel if not in the stopped state
  void timer_callback()
  {
    if (publish_twist) {
      twist_pub_->publish(current_twist);
    }
  }


  /// @brief Provide parameters for driving in a specific circle
  /// @param req velocity and radius of desired circular path
  /// @param res boolean, if setting is successful
  void control_cb(
    std::shared_ptr<nuturtle_control::srv::Control::Request> req,
    std::shared_ptr<nuturtle_control::srv::Control::Response> res)
  {
    const auto radius = req->radius;
    if (radius > 0) {
      RCLCPP_INFO_STREAM(get_logger(), "Driving in a Circle!");
      const auto velocity = req->velocity;
      current_twist.linear.x = velocity;
      current_twist.angular.z = velocity / radius;
      publish_twist = true;
      res->success = true;
    } else {
      RCLCPP_INFO_STREAM(get_logger(), "Radius of circle must be positive!");
      res->success = false;
    }

  }

  /// @brief Reverse the circle
  /// @param req: empty request
  /// @param res: emtpy response
  void reverse_cb(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Reversing the Circle!");
    current_twist.linear.x *= -1.0;
    current_twist.angular.z *= -1.0;
  }

  /// @brief stop publishing cmd_vels
  /// @param req: empty request
  /// @param res: emtpy response
  void stop_cb(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Stopping the Circle!");
    twist_pub_->publish(zero_twist);
    publish_twist = false;
  }

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

  rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_srv_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv_;

  geometry_msgs::msg::Twist current_twist;
  geometry_msgs::msg::Twist zero_twist;

  bool publish_twist = false;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}
