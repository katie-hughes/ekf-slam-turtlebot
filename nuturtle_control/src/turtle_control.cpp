/// \file turtle_control.cpp
/// \brief Provide controls for interpreting sensor data and cmd_vel
///
/// PARAMETERS:
///     wheel_radius: radius of the robot's wheels (m)
///     track_width: distance from the center of the robot to the wheels (m)
///     encoder_ticks_per_rad: Number of ticks that correspond to a one radian turn on the wheels
///     motor_cmd_per_rad_sec: The number of motor commands sent per rad/sec
///     motor_cmd_max: The maximum value that the motors can turn at
/// PUBLISHES:
///    wheel_cmd (nuturtlebot_msgs::msg::WheelCommands): wheel controls for the robot to follow
///    joint_states (sensor_msgs::msg::JointState): The updated robot joint states required to
///       follow the desired wheel commands
/// SUBSCRIBES:
///     cmd_vel (geometry_msgs::msg::Twist): twist for robot to follow
///     sensor_data (nuturtlebot_msgs::msg::SensorData): reading of the robot's internal encoders
/// SERVERS:
///     none
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

#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;

class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("turtle_control")
  {
    declare_parameter("wheel_radius", -1.0);
    wheel_radius = get_parameter("wheel_radius").as_double();
    RCLCPP_INFO_STREAM(get_logger(), "Wheel Radius: " << wheel_radius);
    if (wheel_radius < 0.0) {
      throw std::logic_error("Invalid wheel_radius!");
    }

    declare_parameter("track_width", -1.0);
    track_width = get_parameter("track_width").as_double();
    RCLCPP_INFO_STREAM(get_logger(), "Track Width: " << track_width);
    if (track_width < 0.0) {
      throw std::logic_error("Invalid track_width!");
    }

    declare_parameter("encoder_ticks_per_rad", -1.0);
    encoder_ticks = get_parameter("encoder_ticks_per_rad").as_double();
    RCLCPP_INFO_STREAM(get_logger(), "Encoder Ticks: " << encoder_ticks);
    if (wheel_radius < 0.0) {
      throw std::logic_error("Invalid encoder_ticks_per_rad!");
    }

    declare_parameter("motor_cmd_per_rad_sec", -1.0);
    motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();
    RCLCPP_INFO_STREAM(get_logger(), "motor_cmd_per_rad_sec: " << motor_cmd_per_rad_sec);
    if (motor_cmd_per_rad_sec < 0.0) {
      throw std::logic_error("Invalid motor_cmd_per_rad_sec!");
    }

    declare_parameter("motor_cmd_max", -1);
    motor_cmd_max = get_parameter("motor_cmd_max").as_int();
    RCLCPP_INFO_STREAM(get_logger(), "motor_cmd_max: " << motor_cmd_max);
    if (motor_cmd_max < 0.0) {
      throw std::logic_error("Invalid motor_cmd_max!");
    }

    // slightly hacky workaround to get new values in
    turtlelib::DiffDrive temp(track_width, wheel_radius);
    robot = temp;

    wheel_pub_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);
    js_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&TurtleControl::cmd_vel_cb, this, std::placeholders::_1));

    sensor_sub_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", 10, std::bind(&TurtleControl::sensor_cb, this, std::placeholders::_1));

    js.name = {"wheel_left_joint", "wheel_right_joint"};
  }

private:
  /// @brief callback to convert cmd_vel commands into appropriate wheel commands
  /// @param twist: desired cmd_vel to follow
  void cmd_vel_cb(const geometry_msgs::msg::Twist & twist)
  {
    // assume twist is in the body frame. Convert into turtlelib object.
    const auto Vb_w = twist.angular.z;
    const auto Vb_x = twist.linear.x;
    const auto Vb_y = twist.linear.y;
    turtlelib::Twist2D Vb(Vb_w, turtlelib::Vector2D{Vb_x, Vb_y});
    turtlelib::WheelState ws = robot.ik(Vb);
    nuturtlebot_msgs::msg::WheelCommands wc;
    // RCLCPP_INFO_STREAM(get_logger(), "IK Wheel States "<<ws.l<<" and "<<ws.r);
    wc.left_velocity = static_cast<int>(ws.l / motor_cmd_per_rad_sec);
    wc.right_velocity = static_cast<int>(ws.r / motor_cmd_per_rad_sec);
    // adjust if over the motor command max
    if (wc.left_velocity < -1 * motor_cmd_max) {
      wc.left_velocity = -motor_cmd_max;
    }
    if (wc.left_velocity > motor_cmd_max) {
      wc.left_velocity = motor_cmd_max;
    }
    if (wc.right_velocity < -1 * motor_cmd_max) {
      wc.right_velocity = -motor_cmd_max;
    }
    if (wc.right_velocity > motor_cmd_max) {
      wc.right_velocity = motor_cmd_max;
    }
    // RCLCPP_INFO_STREAM(get_logger(), "Sending "<<wc.left_velocity<<" and "<<wc.right_velocity);
    wheel_pub_->publish(wc);
  }

  /// @brief callback to convert sensor data messages into updated joint states
  /// @param sensor_data current wheel encoder data
  void sensor_cb(const nuturtlebot_msgs::msg::SensorData & sensor_data)
  {
    std::vector<double> joint_position(2);
    joint_position.at(0) = static_cast<double>(sensor_data.left_encoder) / encoder_ticks;
    joint_position.at(1) = static_cast<double>(sensor_data.right_encoder) / encoder_ticks;
    // RCLCPP_INFO_STREAM(get_logger(), "Joint Pos: " << joint_position.at(0) << " and " << joint_position.at(1));
    js.position = joint_position;
    std::vector<double> joint_velocity(2);
    js.header.stamp = sensor_data.stamp; // this->get_clock()->now();
    if (!first_js) {
      const auto current_s = js.header.stamp.sec + 1e-9 * js.header.stamp.nanosec;
      const auto last_s = last_js.header.stamp.sec + 1e-9 * last_js.header.stamp.nanosec;
      const auto dt = current_s - last_s;
      const auto v_left = (js.position.at(0) - last_js.position.at(0)) / dt;
      const auto v_right = (js.position.at(1) - last_js.position.at(1)) / dt;
      joint_velocity.at(0) = v_left;
      joint_velocity.at(1) = v_right;
      js.velocity = joint_velocity;
    } else {
      first_js = false;
    }
    js_pub_->publish(js);
    last_js = js;
  }

  // rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_sub_;
  double wheel_radius, track_width, encoder_ticks, motor_cmd_per_rad_sec;
  int motor_cmd_max;
  // initialize with garbage values. overwrite later
  turtlelib::DiffDrive robot{0.0, 0.0};
  // initialize joint states message template to reuse
  sensor_msgs::msg::JointState js, last_js;
  bool first_js = true;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}
