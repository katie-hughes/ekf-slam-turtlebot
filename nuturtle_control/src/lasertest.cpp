/// \file odometry.cpp
/// \brief Updates odometry estimations to determine where the robot is
///
/// PARAMETERS:
///     body_id (string): the frame id of the robot's footprint, will be a child of odom_id frame
///     odom_id (string): the name of the odom frame (default: odom)
///     wheel_left (string): the name of the left wheel joint in the robot
///     wheel_right (string): the name of the right wheel joint in the robot
///     wheel_radius: radius of the robot's wheels (m)
///     track_width: distance from the center of the robot to the wheels (m)
/// PUBLISHES:
///     odom (nav_msgs::msg::Odometry): current transform between odom_id and body_id
/// SUBSCRIBES:
///    joint_states (sensor_msgs::msg::JointState): the joint states of the odometry robot
/// SERVERS:
///     initial_pose (nuturtle_control::srv::InitialPose): manually set the pose of the odom robot
/// CLIENTS:
///     none
/// BROADCASTS:
///    odom_id -> body_id

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */


class LaserTest : public rclcpp::Node
{
public:
  LaserTest()
  : Node("laser_test")
  {
    laser_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", 100);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    fast_timer_ = create_wall_timer(
      5ms,
      std::bind(&LaserTest::fast_timer_callback, this));

    slow_timer_ = create_wall_timer(
      100ms,
      std::bind(&LaserTest::slow_timer_callback, this));
  }

private:
  void fast_timer_callback()
  {
    current_time = this->get_clock()->now();
    send_transform();
    // rclcpp::sleep_for(10ms);
    // test_publish_laser();
    // do_all();
  }

  void slow_timer_callback()
  {
    test_publish_laser();
  }


  void send_transform()
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = current_time;
    // t.header.stamp.nanosec = 5e6*tf_count;
    //
    // t.header.stamp.sec = 0;
    // t.header.stamp.nanosec = 2e8; // 21e7;
    t.header.frame_id = "f1";
    t.child_frame_id = "f2";
    // give it some offset
    t.transform.translation.x = 0.5;
    t.transform.translation.y = 1.0;
    t.transform.translation.z = 0.0;
    // Send the transformation
    tf_broadcaster_->sendTransform(t);
    tf_count++;
  }

  /// @brief Publish a test LaserScan that should update quicker than real
  void test_publish_laser()
  {
    sensor_msgs::msg::LaserScan laser;
    laser.header.stamp = current_time;
    // laser.header.stamp.nanosec = 1e8*laser_count;
    // laser.header.stamp.sec = 0;
    // laser.header.stamp.nanosec = 2e8; // 0;
    // RCLCPP_INFO_STREAM(get_logger(), "LASER S: " << laser.header.stamp.sec << " ns " <<
    //                                  laser.header.stamp.nanosec);
    laser.header.frame_id = "f2";
    laser.angle_min = 0.0;
    laser.angle_max = 6.2657318115234375;
    laser.angle_increment = 0.01745329238474369;
    laser.time_increment = 0.0; // 0.0005574136157520115;
    laser.scan_time = 0.20066890120506287;
    laser.range_min = 0.1;
    laser.range_max = 3.5;
    const int laser_nsamples = 360;
    // fill in the laser.ranges array
    // just make this super simple to see if it's running too slow in other
    for (int n = 0; n < laser_nsamples; n++) {
      laser.ranges.push_back(1.0);
    }
    laser_pub_->publish(laser);
    laser_count++;
  }

  rclcpp::TimerBase::SharedPtr fast_timer_;
  rclcpp::TimerBase::SharedPtr slow_timer_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;

  builtin_interfaces::msg::Time current_time;

  long laser_count = 0;
  long tf_count = 0;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserTest>());
  rclcpp::shutdown();
  return 0;
}
