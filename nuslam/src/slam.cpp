/// \file slam.cpp
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
#include <armadillo>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nuturtle_control/srv/initial_pose.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;

class Slam : public rclcpp::Node
{
public:
  Slam()
  : Node("slam")
  {
    // this->declare_parameter("body_id","base_footprint");
    declare_parameter("body_id", body_id);
    body_id = get_parameter("body_id").as_string();
    RCLCPP_INFO_STREAM(get_logger(), "Body ID: " << body_id);
    if (body_id.empty()) {
      throw std::logic_error("Invalid body_id!");
    }

    // default = odom
    declare_parameter("odom_id", "odom");
    odom_id = get_parameter("odom_id").as_string();
    RCLCPP_INFO_STREAM(get_logger(), "Odom ID: " << odom_id);

    // this->declare_parameter("wheel_left","wheel_left_joint");
    declare_parameter("wheel_left", wheel_left);
    wheel_left = get_parameter("wheel_left").as_string();
    RCLCPP_INFO_STREAM(get_logger(), "Wheel Left: " << wheel_left);
    if (wheel_left.empty()) {
      throw std::logic_error("Invalid wheel_left!");
    }

    // this->declare_parameter("wheel_right","wheel_right_joint");
    declare_parameter("wheel_right", wheel_right);
    wheel_right = get_parameter("wheel_right").as_string();
    RCLCPP_INFO_STREAM(get_logger(), "Wheel Right: " << wheel_right);
    if (wheel_right.empty()) {
      throw std::logic_error("Invalid wheel_right!");
    }

    declare_parameter("wheel_radius", -1.0);
    wheel_radius = get_parameter("wheel_radius").as_double();
    RCLCPP_INFO_STREAM(get_logger(), "Wheel Radius: " << wheel_radius);
    if (wheel_radius < 0.0) {
      throw std::logic_error("Invalid wheel_radius!");
    }

    declare_parameter("track_width", -1.0);
    track_width = get_parameter("track_width").as_double();
    RCLCPP_INFO_STREAM(get_logger(), "Track Width: " << track_width);
    if (wheel_radius < 0.0) {
      throw std::logic_error("Invalid track_width!");
    }

    // initialize the odometry object that I will publish with frames
    current_odom.header.frame_id = odom_id;
    current_odom.child_frame_id = body_id;

    // do the same for tf object
    T_odom_base.header.frame_id = odom_id;
    T_odom_base.child_frame_id = body_id;
    // this is provided by the slam update
    T_map_odom.header.frame_id = "map";
    T_map_odom.child_frame_id = odom_id;
    
    robot = turtlelib::DiffDrive(track_width, wheel_radius);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    path_pub_ = create_publisher<nav_msgs::msg::Path>("~/path", 10);

    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&Slam::js_cb, this, std::placeholders::_1));

    fake_sensor_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "fake_sensor", 10, std::bind(&Slam::fake_sensor_cb, this, std::placeholders::_1));

    initial_pose_srv_ = create_service<nuturtle_control::srv::InitialPose>(
      "initial_pose",
      std::bind(&Slam::initial_pose, this, std::placeholders::_1, std::placeholders::_2));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    max_obstacles = 3;
    Covariance = arma::mat(max_obstacles*2 + 3, max_obstacles*2 + 3);
    Covariance.fill(0.0);
    // 
    for (int i = 0; i < 2*max_obstacles; i++){
      // These represent that in the beginning we don't know anything about obstacle loc.
      Covariance(i+3, i+3) = 999999;
    }
    RCLCPP_INFO_STREAM(get_logger(), "Covariance:\n" << Covariance);
  }

private:
  void fake_sensor_cb(const visualization_msgs::msg::MarkerArray & sensor)
  {
    // RCLCPP_INFO_STREAM(get_logger(), "Received Marker");
    if (first_slam_iteration){
      first_slam_iteration = false;
    }
    // PREDICTION

    // 1. Update state estimate. This is already handled by odometry?

    // 2. Propogate state uncertainty using linearized state transition model
    // For this I need A and Q. 
    // Sigma_t^- = At SigmaHat_{t-1} At^T + Qbar
    // Qbar = [Q & 03x2n \\ 02nx3 02nx2n]
    // But what is Q??? "Process noise for robot motion model"
    for(int i = 0; i < static_cast<double>(sensor.markers.size()); i++){
      const auto mx = sensor.markers.at(i).pose.position.x;
      const auto my = sensor.markers.at(i).pose.position.y;
      RCLCPP_INFO_STREAM(get_logger(), "X and Y: "<< mx << ", " << my);
    }
    // DO SOMETHING HERE BASED ON SLAM UPDATE rn it's identity transform
    T_map_odom.header.stamp = get_clock()->now();
    tf_broadcaster_->sendTransform(T_map_odom);
  }

  void js_cb(const sensor_msgs::msg::JointState & js)
  {
    if (!first_iteration) {
      // RCLCPP_INFO_STREAM(get_logger(), "JS CALLBACK!");
      // std::find function call: Example I took from here
      // https://www.geeksforgeeks.org/how-to-find-index-of-a-given-element-in-a-vector-in-cpp/
      const auto find_left = std::find(js.name.begin(), js.name.end(), wheel_left);
      const auto find_right = std::find(js.name.begin(), js.name.end(), wheel_right);
      if ((find_left != js.name.end()) && (find_right != js.name.end())) {
        const auto index_left = find_left - js.name.begin();
        const auto index_right = find_right - js.name.begin();
        // find change in joint positions of the wheels
        const auto dl = js.position.at(index_left) - last_js.position.at(index_left);
        const auto dr = js.position.at(index_right) - last_js.position.at(index_right);
        // RCLCPP_INFO_STREAM(get_logger(), "dl: "<<dl);
        // RCLCPP_INFO_STREAM(get_logger(), "dr: "<<dr);
        const auto current_s = js.header.stamp.sec + 1e-9 * js.header.stamp.nanosec;
        const auto last_s = last_js.header.stamp.sec + 1e-9 * last_js.header.stamp.nanosec;
        const auto dt = current_s - last_s;
        // apply forward kinematics. Save these params to calculate odom velocity
        const auto prev_x = robot.get_x();
        const auto prev_y = robot.get_y();
        const auto prev_phi = robot.get_phi();
        robot.fk(dl, dr);
        // RCLCPP_INFO_STREAM(get_logger(), "Odom FK: " << dl << " and " << dr);
        // publish the location
        // RCLCPP_INFO_STREAM(get_logger(), "Odom Pose: " << robot.get_config());
        // publish odometry message based on current config.
        tf2::Quaternion q;
        q.setRPY(0, 0, robot.get_phi());
        current_odom.header.stamp = js.header.stamp;
        current_odom.pose.pose.position.x = robot.get_x();
        current_odom.pose.pose.position.y = robot.get_y();
        current_odom.pose.pose.orientation.x = q.x();
        current_odom.pose.pose.orientation.y = q.y();
        current_odom.pose.pose.orientation.z = q.z();
        current_odom.pose.pose.orientation.w = q.w();
        // Add twist
        current_odom.twist.twist.linear.x = (robot.get_x() - prev_x) / dt;
        current_odom.twist.twist.linear.y = (robot.get_y() - prev_y) / dt;
        current_odom.twist.twist.angular.z = (robot.get_phi() - prev_phi) / dt;
        odom_pub_->publish(current_odom);
        // update publisher
        T_odom_base.header.stamp = js.header.stamp;
        T_odom_base.transform.translation.x = robot.get_x();
        T_odom_base.transform.translation.y = robot.get_y();
        T_odom_base.transform.rotation.x = q.x();
        T_odom_base.transform.rotation.y = q.y();
        T_odom_base.transform.rotation.z = q.z();
        T_odom_base.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(T_odom_base);
        // update path. ONLY SELECTIVELY
        if (iterations % 100 == 0){
          geometry_msgs::msg::PoseStamped ps;
          ps.header.stamp = js.header.stamp;
          ps.header.frame_id = "nusim/world";
          ps.pose.position.x = robot.get_x();
          ps.pose.position.y = robot.get_y();
          ps.pose.orientation.x = q.x();
          ps.pose.orientation.y = q.y();
          ps.pose.orientation.z = q.z();
          ps.pose.orientation.w = q.w();
          followed_path.poses.push_back(ps);
          // keep array from getting too big!
          if (followed_path.poses.size()>100){
            followed_path.poses.erase(followed_path.poses.begin());
          }
          followed_path.header.stamp = ps.header.stamp;
          followed_path.header.frame_id = "nusim/world";
          path_pub_->publish(followed_path);
        }
        iterations++;
      } else {
        RCLCPP_INFO_STREAM(get_logger(), "Wheel IDs not found in Joint State Message!");
      }
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
    RCLCPP_INFO_STREAM(get_logger(), "Reset initial pose!");
    turtlelib::Transform2D new_pose(turtlelib::Vector2D{req->x, req->y}, req->theta);
    robot = turtlelib::DiffDrive(new_pose, track_width, wheel_radius);
    res->success = true;
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_sub_;

  rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr initial_pose_srv_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // initialize with garbage values, overwrite later
  turtlelib::DiffDrive robot{0.0, 0.0};
  double wheel_radius, track_width;
  sensor_msgs::msg::JointState last_js;
  bool first_iteration = true, first_slam_iteration = true;
  nav_msgs::msg::Odometry current_odom;
  std::string body_id, odom_id, wheel_left, wheel_right;
  geometry_msgs::msg::TransformStamped T_odom_base, T_map_odom;
  nav_msgs::msg::Path followed_path;
  long iterations = 0;
  int max_obstacles;

  // Sigma
  arma::mat Covariance;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Slam>());
  rclcpp::shutdown();
  return 0;
}
