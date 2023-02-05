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

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Circle : public rclcpp::Node
{
  public:
    Circle()
    : Node("circle")
    {

      twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      
      timer_ = create_wall_timer(
        500ms, std::bind(&Circle::timer_callback, this));

      control_srv_ = this->create_service<nuturtle_control::srv::Control>(
        "control",
        std::bind(&Circle::control_cb, this, std::placeholders::_1, std::placeholders::_2));

    }

  private:
    void timer_callback()
    {
      RCLCPP_INFO_STREAM(get_logger(), "Timer Tick");
      twist_pub_->publish(current_twist);
    }

    void control_cb(
      std::shared_ptr<nuturtle_control::srv::Control::Request> req,
      std::shared_ptr<nuturtle_control::srv::Control::Response> res)
    {
      /// \brief Reset the simulation
      ///
      /// \param Request: x, y, and theta of desired position
      /// \param Response: boolean, if relocation is successful
      RCLCPP_INFO_STREAM(get_logger(), "Service Call!");
      (void) req;
      (void) res;
    }

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

    rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_srv_;

    geometry_msgs::msg::Twist current_twist;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}