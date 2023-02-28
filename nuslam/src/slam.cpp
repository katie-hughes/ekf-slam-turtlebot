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

    measure_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/measured", 10);

    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&Slam::js_cb, this, std::placeholders::_1));

    fake_sensor_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "fake_sensor", 10, std::bind(&Slam::fake_sensor_cb, this, std::placeholders::_1));

    initial_pose_srv_ = create_service<nuturtle_control::srv::InitialPose>(
      "initial_pose",
      std::bind(&Slam::initial_pose, this, std::placeholders::_1, std::placeholders::_2));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    max_obstacles = 3;

    obstacle_initialized = std::vector<bool>(max_obstacles, false);

    slam_state = arma::vec(max_obstacles*2 + 3, arma::fill::zeros);
    RCLCPP_INFO_STREAM(get_logger(), "Slam State:\n" << slam_state);
    last_slam_state = arma::vec(max_obstacles*2 + 3, arma::fill::zeros);

    Covariance = arma::mat(max_obstacles*2 + 3, max_obstacles*2 + 3, arma::fill::zeros);
    // set initial covariance
    for (int i = 0; i < 2*max_obstacles; i++){
      // These represent that in the beginning we don't know anything about obstacle loc.
      Covariance.at(i+3, i+3) = 999999;
    }
    RCLCPP_INFO_STREAM(get_logger(), "Covariance:\n" << Covariance);

    // Set Q and R covariance matrices
    // Right now they are just Identity. Might need to fiddle with values
    Q = arma::mat(3, 3, arma::fill::zeros);
    Q.diag() += 1e-3; // e-3
    RCLCPP_INFO_STREAM(get_logger(), "Q:\n" << Q);
    R = arma::mat(2*max_obstacles, 2*max_obstacles, arma::fill::zeros);
    R.diag() += 1e-1; // e-1
    RCLCPP_INFO_STREAM(get_logger(), "R:\n" << R);

    // Qbar = [Q & 03x2n \\ 02nx3 02nx2n]
    Qbar = arma::mat(max_obstacles*2 + 3, max_obstacles*2 + 3, arma::fill::zeros);
    Qbar.submat(0, 0, 2, 2) = Q;
    RCLCPP_INFO_STREAM(get_logger(), "Qbar:\n" << Qbar);

    // i don't know why I can't use their identity constructor but here we are
    myIdentity = arma::mat(max_obstacles*2 + 3, max_obstacles*2 + 3, arma::fill::zeros);
    myIdentity.diag() += 1.0;
    RCLCPP_INFO_STREAM(get_logger(), "myIdentity:\n" << myIdentity);

    last_odom_x = 0;
    last_odom_y = 0;
    last_odom_theta = 0;
  }

private:
  void fake_sensor_cb(const visualization_msgs::msg::MarkerArray & sensor)
  {
    // RCLCPP_INFO_STREAM(get_logger(), "Received Marker");
    // PREDICTION

    // 1. Update state estimate. This is already handled by odometry.
    odom_x = robot.get_x();
    odom_y = robot.get_y();
    odom_theta = robot.get_phi();

    slam_state.at(0) += (odom_theta - last_odom_theta);
    slam_state.at(0) = turtlelib::normalize_angle(slam_state.at(0));
    slam_state.at(1) += (odom_x - last_odom_x);
    slam_state.at(2) += (odom_y - last_odom_y);
    RCLCPP_INFO_STREAM(get_logger(), "Initial Slam State\n"<< slam_state);
    // const auto dtheta = turtlelib::normalize_angle(slam_state.at(0) - last_slam_state.at(0));
    const auto dx = slam_state.at(1) - last_slam_state.at(1);
    const auto dy = slam_state.at(2) - last_slam_state.at(2);
    // 2. Propogate state uncertainty using linearized state transition model
    // For this I need At and Qbar. 
    // Sigma_t^- = At SigmaHat_{t-1} At^T + Qbar
    arma::mat At(3 + 2*max_obstacles, 3 + 2*max_obstacles, arma::fill::zeros);
    // only 2 nonzero elements
    At.at(1, 0) = -dy;
    At.at(2, 0) = dx;
    // At is a sum with the identity matrix
    At.diag() += 1.0;
    // RCLCPP_INFO_STREAM(get_logger(), "At:\n"<< At);

    Covariance = At * Covariance * At.t() + Qbar;

    for(int i = 0; i < static_cast<double>(sensor.markers.size()); i++){
      const auto mx = sensor.markers.at(i).pose.position.x;
      const auto my = sensor.markers.at(i).pose.position.y;
      // this id will be unique for each marker.
      // If one goes out of range, the id will no longer show up.
      const auto id = sensor.markers.at(i).id;
      // RCLCPP_INFO_STREAM(get_logger(), "X and Y: "<< mx << ", " << my);
      // RCLCPP_INFO_STREAM(get_logger(), "id: "<< id);
      // CHECK THAT IT IS NOT "DELETE"
      const auto act = sensor.markers.at(i).action;
      if (act == 2){
        // Obstacle is out of range. Skip this loop iteration!
        continue;
      }
      // this is z (current sensor measurement)
      const auto dxj = mx;
      const auto dyj = my;
      const auto dj = dxj*dxj + dyj*dyj;
      const auto rj = sqrt(dj);
      const auto phij = turtlelib::normalize_angle(atan2(dyj, dxj));
      arma::vec zj(2, arma::fill::zeros);
      zj.at(0) = rj;
      zj.at(1) = phij;
      // RCLCPP_INFO_STREAM(get_logger(), "zj\n"<< zj);

      // check if this obstacle has been seen before. 
      if (!obstacle_initialized.at(id)){
        // Obstacle has not been seen. Initialize it!
        // x coordinate
        slam_state.at(3 + 2*id) = slam_state.at(1) + rj * 
                                  cos(turtlelib::normalize_angle(phij + slam_state.at(0)));
        // y coordinate
        slam_state.at(3 + 2*id + 1) = slam_state.at(2) + rj * 
                                      sin(turtlelib::normalize_angle(phij + slam_state.at(0)));
        obstacle_initialized.at(id) = true;
        RCLCPP_INFO_STREAM(get_logger(), "Initialized Obstacle # "<< id);
      }

      // this is Zbar (ESTIMATED measurement). Take from state estimation .
      const auto dxj_hat = slam_state.at(3 + 2*id) - slam_state.at(1);
      const auto dyj_hat = slam_state.at(3 + 2*id + 1) - slam_state.at(2);
      const auto dj_hat = dxj_hat*dxj_hat + dyj_hat*dyj_hat;
      const auto rj_hat = sqrt(dj_hat);
      const auto phij_hat = turtlelib::normalize_angle(atan2(dyj_hat, dxj_hat) - slam_state.at(0));
      arma::vec zj_hat(2, arma::fill::zeros);
      zj_hat.at(0) = rj_hat;
      zj_hat.at(1) = phij_hat;
      // RCLCPP_INFO_STREAM(get_logger(), "zj_hat\n"<< zj_hat);

      arma::mat Hj(2, 3+2*max_obstacles, arma::fill::zeros);
      // "j" is id. 
      Hj.at(1,0) = -1.0;
      Hj.at(0,1) = -dxj_hat/rj_hat;
      Hj.at(1,1) =  dyj_hat/dj_hat;
      Hj.at(0,2) = -dyj_hat/rj_hat;
      Hj.at(1,2) = -dxj_hat/dj_hat;
      // skip 2*(j-1) elements except id starts at 0 and j starts at 1 so it's just 2*id
      Hj.at(0,3 + 2*id) =  dxj_hat/rj_hat;
      Hj.at(1,3 + 2*id) = -dyj_hat/dj_hat;
      Hj.at(0,4 + 2*id) =  dyj_hat/rj_hat;
      Hj.at(1,4 + 2*id) =  dxj_hat/dj_hat;
      // RCLCPP_INFO_STREAM(get_logger(), "Hj:\n"<< Hj);

      // Kalman gain
      arma::mat Ri = R.submat(id, id, id+1, id+1);
      arma::mat help_Kj = Hj * Covariance * Hj.t() + Ri;
      arma::mat Kj = Covariance * Hj.t() * help_Kj.i(); 
      // RCLCPP_INFO_STREAM(get_logger(), "Kalman Gain:\n"<< Kj);

      // State update
      arma::vec dzj = zj - zj_hat;
      dzj.at(1) = turtlelib::normalize_angle(dzj.at(1));
      // RCLCPP_INFO_STREAM(get_logger(), "dzj:\n"<< dzj);
      slam_state = slam_state + Kj*dzj;
      // RCLCPP_INFO_STREAM(get_logger(), "New Slam State:\n"<< slam_state);

      // Covariance update
      Covariance = (myIdentity - Kj * Hj) * Covariance;
    }
    // I want T_map_basefootprint = slam_state
    // But I can only publish T_map_odom
    // 1. Lookup T_odom_basefootprint
    // Except I don't need to look up, it's just robot.get_x() and robot.get_y() etc.
    // 2. Desired x = slamx - robot.get_x() etc
    // That should be it? 
    RCLCPP_INFO_STREAM(get_logger(), "Slam State\n"<< slam_state);
    T_map_odom.header.stamp = get_clock()->now();
    const turtlelib::Transform2D Tob = robot.get_config();
    const turtlelib::Transform2D Tmb{turtlelib::Vector2D{slam_state.at(1), slam_state.at(2)},
                                     slam_state.at(0)};
    const auto Tmo = Tmb*Tob.inv();
    RCLCPP_INFO_STREAM(get_logger(), "Tmb: "<< Tmb);
    // T_map_odom.transform.translation.x = slam_state.at(1) - robot.get_x();
    // T_map_odom.transform.translation.y = slam_state.at(2) - robot.get_y();
    T_map_odom.transform.translation.x = Tmo.translation().x;
    T_map_odom.transform.translation.y = Tmo.translation().y;
    tf2::Quaternion q;
    // q.setRPY(0, 0, turtlelib::normalize_angle(slam_state.at(0) - robot.get_phi()));
    q.setRPY(0, 0, turtlelib::normalize_angle(Tmo.rotation()));
    T_map_odom.transform.rotation.x = q.x();
    T_map_odom.transform.rotation.y = q.y();
    T_map_odom.transform.rotation.z = q.z();
    T_map_odom.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(T_map_odom);
    // update path. ONLY SELECTIVELY
    if (iterations % 5 == 0){
      geometry_msgs::msg::PoseStamped ps;
      ps.header.stamp = get_clock()->now();
      ps.header.frame_id = "nusim/world";
      ps.pose.position.x = slam_state.at(1);
      ps.pose.position.y = slam_state.at(2);
      followed_path.poses.push_back(ps);
      // keep array from getting too big!
      if (followed_path.poses.size()>100){
        followed_path.poses.erase(followed_path.poses.begin());
      }
      followed_path.header.stamp = get_clock()->now();
      followed_path.header.frame_id = "nusim/world";
      path_pub_->publish(followed_path);
    }
    iterations++;
    publish_measurements();
    // Current slam state now becomes the last slam state
    last_slam_state = slam_state;
    // these for keeping track of odometry
    last_odom_x = odom_x;
    last_odom_y = odom_y;
    last_odom_theta = odom_theta;
  }

  /// @brief Publish slam measurement marker array
  void publish_measurements(){
    visualization_msgs::msg::MarkerArray ma;
    for (int i = 0; i < max_obstacles; i++) {
      if (obstacle_initialized.at(i)){
        visualization_msgs::msg::Marker m;
        m.header.stamp = get_clock()->now();
        m.header.frame_id = "map";
        m.id = i;         // so each has a unique ID
        m.type = 3;       // cylinder
        m.action = 0; // add/modify
        // Set color as green
        m.color.r = 0.0;
        m.color.g = 1.0;
        m.color.b = 0.0;
        m.color.a = 1.0;
        // Set Radius
        const auto obr = 0.038;
        m.scale.x = 2*obr;
        m.scale.y = 2*obr;
        m.scale.z = 0.25;
        // set position
        // get the obstacle location relative to the robot frame
        m.pose.position.x = slam_state.at(3 + 2*i);
        m.pose.position.y = slam_state.at(3 + 2*i+1);
        m.pose.position.z = 0.125;
        // Add to marker array
        ma.markers.push_back(m);
      }
    }
    // RCLCPP_INFO_STREAM(get_logger(), "Publishing Marker Array");
    measure_pub_->publish(ma);
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
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr measure_pub_;
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
  double odom_x, odom_y, odom_theta, last_odom_x, last_odom_y, last_odom_theta;

  // Sigma
  arma::mat Covariance, Q, R, Qbar, myIdentity;
  arma::vec slam_state, last_slam_state;
  std::vector<bool> obstacle_initialized;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Slam>());
  rclcpp::shutdown();
  return 0;
}
