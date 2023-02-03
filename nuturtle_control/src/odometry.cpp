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

#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Odometry : public rclcpp::Node
{
  public:
    Odometry()
    : Node("turtle_control")
    {
      // TODO Figure out how to have these as uninitialized ;-;
      // https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1Node.html#a095ea977b26e7464d9371efea5f36c42
      // this->declare_parameter("body_id","");
      // this->declare_parameter("odom_id","odom");
      // this->declare_parameter("wheel_left","");
      // this->declare_parameter("wheel_left","");

      // wheel_radius = this->get_parameter("wheel_radius").as_double();
      // track_width = this->get_parameter("track_width").as_double();
      // encoder_ticks = this->get_parameter("encoder_ticks_per_rad").as_double();

      // RCLCPP_INFO_STREAM(get_logger(), "Wheel Radius: "<<wheel_radius);
      // RCLCPP_INFO_STREAM(get_logger(), "Track Width: "<<track_width);
      // RCLCPP_INFO_STREAM(get_logger(), "Encoder Ticks: "<<encoder_ticks);

      // slightly hacky workaround to get new values in
      // turtlelib::DiffDrive temp(track_width, wheel_radius);
      // robot = temp;

      // publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      // wheel_pub_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);
      // js_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
      
      js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&Odometry::odom_cb, this, std::placeholders::_1));

      timer_ = create_wall_timer(
        500ms, std::bind(&Odometry::timer_callback, this));

      initial_pose_srv_ = this->create_service<std_srvs::srv::Empty>(
        "initial_pose",
        std::bind(&Odometry::initial_pose, this, std::placeholders::_1, std::placeholders::_2));
    }

  private:
    void timer_callback()
    {
      RCLCPP_INFO_STREAM(get_logger(), "Timer Tick");
    }

    void odom_cb(const sensor_msgs::msg::JointState & js)
    {
      (void) js;
      RCLCPP_INFO_STREAM(get_logger(), "JS Received");
    }

    void initial_pose(
      std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
      /// \brief Reset the simulation
      ///
      /// \param Request: The empty request
      /// \param Response: The empty response
      RCLCPP_INFO_STREAM(get_logger(), "Service Call!");
    }

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;

    // rclcpp::Service<std_srvs::srv::Empty>::SharedPtr initial_pose_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr initial_pose_srv_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}