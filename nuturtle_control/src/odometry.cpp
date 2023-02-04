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
      this->declare_parameter("body_id","base_footprint");
      // this->declare_parameter("body_id",body_id);
      body_id = this->get_parameter("body_id").as_string();
      RCLCPP_INFO_STREAM(get_logger(), "Body ID: "<<body_id);
      if (body_id.empty()){
         throw std::logic_error("Invalid body_id!");
      }

      // default = odom
      this->declare_parameter("odom_id","odom");
      odom_id = this->get_parameter("odom_id").as_string();
      RCLCPP_INFO_STREAM(get_logger(), "Odom ID: "<<odom_id);

      this->declare_parameter("wheel_left","wheel_left_joint");
      // this->declare_parameter("wheel_left",wheel_left);
      wheel_left = this->get_parameter("wheel_left").as_string();
      RCLCPP_INFO_STREAM(get_logger(), "Wheel Left: "<<wheel_left);
      if (wheel_left.empty()){
         throw std::logic_error("Invalid wheel_left!");
      }

      this->declare_parameter("wheel_right","wheel_right_joint");
      // this->declare_parameter("wheel_right",wheel_right);
      wheel_right = this->get_parameter("wheel_right").as_string();
      RCLCPP_INFO_STREAM(get_logger(), "Wheel Right: "<<wheel_right);
      if (wheel_right.empty()){
         throw std::logic_error("Invalid wheel_right!");
      }

      this->declare_parameter("wheel_radius",-1.0);
      wheel_radius = this->get_parameter("wheel_radius").as_double();
      RCLCPP_INFO_STREAM(get_logger(), "Wheel Radius: "<<wheel_radius);
      if (wheel_radius<0.0){
        throw std::logic_error("Invalid wheel_radius!");
      }

      this->declare_parameter("track_width",-1.0);
      track_width = this->get_parameter("track_width").as_double();
      RCLCPP_INFO_STREAM(get_logger(), "Track Width: "<<track_width);
      if (wheel_radius<0.0){
        throw std::logic_error("Invalid track_width!");
      }

      // slightly hacky workaround to get new values in
      turtlelib::DiffDrive temp(track_width, wheel_radius);
      robot = temp;

      // udpate header fields of the curr_odom object. 
      // ie .header.stamp, .header.frame_id, .child_frame_id

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
      RCLCPP_INFO_STREAM(get_logger(), "Current Pose: "<<robot.get_config());
      // publish odometry message based on current config.
      current_odom.header.stamp = this->get_clock()->now();
      current_odom.pose.pose.position.x = robot.get_x();
      current_odom.pose.pose.position.y = robot.get_y();
      // then there is also a quaternion i need to edit...
      odom_pub_->publish(current_odom);
    }

    void js_cb(const sensor_msgs::msg::JointState & js)
    {
      RCLCPP_INFO_STREAM(get_logger(), "JS Received");
      if (!first_iteration){
        double dl = js.position[0] - last_js.position[0];
        double dr = js.position[1] - last_js.position[1];
        RCLCPP_INFO_STREAM(get_logger(), "dl: "<<dl);
        RCLCPP_INFO_STREAM(get_logger(), "dr: "<<dr);
        double current_s = js.header.stamp.sec + 1e-9*js.header.stamp.nanosec;
        double last_s = last_js.header.stamp.sec + 1e-9*last_js.header.stamp.nanosec;
        double dt = current_s - last_s;
        RCLCPP_INFO_STREAM(get_logger(), "dt: "<<dt);
        // apply forward kinematics
        robot.fk(dl/dt, dr/dt);
      } else {
        first_iteration = false;
      }
      last_js = js;
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
      turtlelib::DiffDrive new_robot(new_pose, track_width, wheel_radius);
      robot = new_robot;
      res->success = true;
    }

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;

    rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr initial_pose_srv_;

    // initialize with garbage values, overwrite later
    turtlelib::DiffDrive robot{0.0, 0.0};
    double wheel_radius, track_width;
    sensor_msgs::msg::JointState last_js;
    bool first_iteration = true;
    nav_msgs::msg::Odometry current_odom;
    std::string body_id, odom_id, wheel_left, wheel_right;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}