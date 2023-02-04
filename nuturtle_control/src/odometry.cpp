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

      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
      
      js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&Odometry::js_cb, this, std::placeholders::_1));

      timer_ = create_wall_timer(
        500ms, std::bind(&Odometry::timer_callback, this));

      initial_pose_srv_ = this->create_service<nuturtle_control::srv::InitialPose>(
        "initial_pose",
        std::bind(&Odometry::initial_pose, this, std::placeholders::_1, std::placeholders::_2));
    }

  private:
    void timer_callback()
    {
      RCLCPP_INFO_STREAM(get_logger(), "Current Pose: "<<current_pose);
    }

    void js_cb(const sensor_msgs::msg::JointState & js)
    {
      (void) js;
      RCLCPP_INFO_STREAM(get_logger(), "JS Received");
    }

    void initial_pose(
      std::shared_ptr<nuturtle_control::srv::InitialPose::Request> req,
      std::shared_ptr<nuturtle_control::srv::InitialPose::Response> res)
    {
      /// \brief Reset the simulation
      ///
      /// \param Request: x, y, and theta of desired position
      /// \param Response: boolean, if relocation is successful
      RCLCPP_INFO_STREAM(get_logger(), "Service Call!");
      turtlelib::Transform2D new_pose(turtlelib::Vector2D{req->x, req->y},req->theta);
      current_pose = new_pose;
      res->success = true;
    }

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;

    rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr initial_pose_srv_;

    turtlelib::Transform2D current_pose;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}