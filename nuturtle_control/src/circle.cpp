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
      this->declare_parameter("frequency",100);
      int frequency_hz = this->get_parameter("frequency").as_int();
      RCLCPP_INFO_STREAM(get_logger(), "Publish frequency is "<<frequency_hz<<" Hz");
      std::chrono::milliseconds rate = (std::chrono::milliseconds) ((int)(1000. / frequency_hz));

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
    void timer_callback()
    {
      if (publish_twist){
        twist_pub_->publish(current_twist);
      }
    }

    void control_cb(
      std::shared_ptr<nuturtle_control::srv::Control::Request> req,
      std::shared_ptr<nuturtle_control::srv::Control::Response> res)
    {
      /// \brief Provide parameters for driving in a specific circle
      ///
      /// \param Request: velocity and radius of desired circular path
      /// \param Response: boolean, if setting is successful
      double radius = req->radius;
      if (radius > 0){
        RCLCPP_INFO_STREAM(get_logger(), "Driving in a Circle!");
        double velocity = req->velocity;
        current_twist.linear.x = velocity;
        current_twist.angular.z = velocity/radius;
        publish_twist = true;
        res->success = true;
      } else {
        RCLCPP_INFO_STREAM(get_logger(), "Radius of circle must be positive!");
        res->success = false;
      }
      
    }

    void reverse_cb(
      std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
      /// \brief Reverse the circle
      ///
      /// \param Request: empty request
      /// \param Response: empty response
      RCLCPP_INFO_STREAM(get_logger(), "Reversing the Circle!");
      current_twist.linear.x *= -1.0;
      current_twist.angular.z *= -1.0;
    }

    void stop_cb(
      std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
      /// \brief Stop publishing cmd_vels
      ///
      /// \param Request: empty request
      /// \param Response: empty response
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