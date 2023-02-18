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
#include "tf2_msgs/msg/tf_message.hpp"
#include "rclcpp/qos.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */



class Test : public rclcpp::Node
{
public:
  Test()
  : Node("test")
  {

    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(), std::bind(&Test::laser_cb, this, std::placeholders::_1));

    tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "tf", 10, std::bind(&Test::tf_cb, this, std::placeholders::_1));

    sensor_sub_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", 10, std::bind(&Test::sensor_cb, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      2s,
      std::bind(&Test::timer_callback, this));
  }

private:

  void timer_callback()
  {
    RCLCPP_INFO_STREAM(get_logger(), "Laser  Publishes: "<<lasers_count);
    RCLCPP_INFO_STREAM(get_logger(), "Sensor Publishes: "<<sensors_count);
    RCLCPP_INFO_STREAM(get_logger(), "Odom   Publishes: "<<tfs_count<<"\n");
    lasers_count = 0;
    sensors_count = 0;
    tfs_count = 0;
    tfs.clear();
    sensors.clear();
    lasers.clear();
  }

  void laser_cb(const sensor_msgs::msg::LaserScan & las)
  {
    lasers.push_back(las);
    lasers_count ++;
    // RCLCPP_ERROR_STREAM(get_logger(), "Laser Sec: " << las.header.stamp.sec <<
    //                                        " ns: " << las.header.stamp.nanosec);
  }

  void sensor_cb(const nuturtlebot_msgs::msg::SensorData & sensor)
  {
    sensors.push_back(sensor);
    sensors_count ++;
    // RCLCPP_ERROR_STREAM(get_logger(), "Laser Sec: " << las.header.stamp.sec <<
    //                                        " ns: " << las.header.stamp.nanosec);
  }

  void tf_cb(const tf2_msgs::msg::TFMessage & tf)
  {
    for (size_t n = 0; n < tf.transforms.size(); n++){
      const auto transform = tf.transforms.at(n);
      if (transform.header.frame_id == "odom"){
        tfs.push_back(tf);
        tfs_count ++;
        // RCLCPP_INFO_STREAM(get_logger(), "Odom Sec: " << transform.header.stamp.sec <<
        //                                       " ns: " << transform.header.stamp.nanosec);
      }
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_sub_;

  std::vector<sensor_msgs::msg::LaserScan> lasers;
  std::vector<tf2_msgs::msg::TFMessage> tfs;
  std::vector<nuturtlebot_msgs::msg::SensorData> sensors;

  long tfs_count = 0;
  long lasers_count = 0;
  long sensors_count = 0;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Test>());
  rclcpp::shutdown();
  return 0;
}
