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

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class TurtleControl : public rclcpp::Node
{
  public:
    TurtleControl()
    : Node("turtle_control")
    {
      // TODO Figure out how to have these as uninitialized ;-;
      // https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1Node.html#a095ea977b26e7464d9371efea5f36c42
      this->declare_parameter("wheel_radius",0.0);
      this->declare_parameter("track_width",0.0);

      wheel_radius = this->get_parameter("wheel_radius").as_double();
      track_width = this->get_parameter("track_width").as_double();

      RCLCPP_INFO_STREAM(get_logger(), "Wheel Radius: "<<wheel_radius);
      RCLCPP_INFO_STREAM(get_logger(), "Track Widht: "<<track_width);

      // slightly hacky workaround to get new values in
      turtlelib::DiffDrive temp(track_width, wheel_radius);
      robot = temp;

      // publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      wheel_pub_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);
      js_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
      
      cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&TurtleControl::cmd_vel_cb, this, std::placeholders::_1));

      sensor_sub_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
        "sensor_data", 10, std::bind(&TurtleControl::sensor_cb, this, std::placeholders::_1));

      timer_ = create_wall_timer(
      500ms, std::bind(&TurtleControl::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      // auto cmd_vel = geometry_msgs::msg::Twist();
      RCLCPP_INFO_STREAM(get_logger(), "Timer Tick");
      // publisher_->publish(cmd_vel);
    }

    void cmd_vel_cb(const geometry_msgs::msg::Twist & twist)
    {
      (void) twist;
      RCLCPP_INFO_STREAM(get_logger(), "Twist Received");
    }

    void sensor_cb(const nuturtlebot_msgs::msg::SensorData & sensor_data)
    {
      (void) sensor_data;
      RCLCPP_INFO_STREAM(get_logger(), "Sensor Data Received");
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_sub_;
    double wheel_radius, track_width;
    // initialize with garbage values. overwrite later
    turtlelib::DiffDrive robot{0.0, 0.0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}