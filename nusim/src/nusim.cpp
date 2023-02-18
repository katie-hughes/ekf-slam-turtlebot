/// \file nusim.cpp
/// \brief Simulates a turtlebot.
///
/// PARAMETERS:
///     rate (double): frequency of the timer, in Hz
///     x0 (double): starting x location of the turtlebot (m)
///     y0 (double): starting y location of the turtlebot (m)
///     theta0 (double): starting theta location of the turtlebot (rad)
///     obstacles/x (double[]): list of x coordinates of cylindrical obstacles (m)
///     obstacles/y (double[]): list of r coordinates of cylindrical obstacles (m)
///     obstacles/r (double): radius of cylindrical obstacles (m)
///     wheel_radius: radius of the robot's wheels (m)
///     track_width: distance from the center of the robot to the wheels (m)
///     encoder_ticks_per_rad: Number of ticks that correspond to a one radian turn on the wheels
///     motor_cmd_per_rad_sec: The number of motor commands sent per rad/sec
///     motor_cmd_max: The maximum value that the motors can turn at
///     ~x_length: X length of rectangular arena (m)
///     ~y_length: Y length of rectangular arena (m)
/// PUBLISHES:
///     ~/timestep (std_msgs::msg::Uint64): current timestep of simulation
///     ~/obstacles (visualization_msgs::msg::MarkerArray): marker objects representing cylinders
///     ~/walls (visualization_msgs::msg::MarkerArray): marker objects representing walls of arena
///     red/sensor_data (nuturtlebot_msgs::msg::SensorData): current simulated sensor readings
/// SUBSCRIBES:
///    red/wheel_cmd (nuturtlebot_msgs::msg::WheelCommands): simulated controls for robot to follow
/// SERVERS:
///     ~/reset (std_srvs::srv::Empty): resets the simulation to the initial state
///     ~/teleport (nusim::srv::Teleport): teleports the turtle to a given x, y, theta value
/// CLIENTS:
///     none
/// BROADCASTS:
///    nusim/world -> red/base_footprint


#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nusim/srv/teleport.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;

class Nusim : public rclcpp::Node
{
public:
  Nusim()
  : Node("nusim"), timestep_(0)
  {
    declare_parameter("rate", 200.);

    declare_parameter("x0", 0.);
    x0 = get_parameter("x0").as_double();

    declare_parameter("y0", 0.);
    y0 = get_parameter("y0").as_double();

    declare_parameter("theta0", 0.);
    theta0 = get_parameter("theta0").as_double();

    declare_parameter("obstacles/x", std::vector<double>{});
    obx = get_parameter("obstacles/x").as_double_array();

    declare_parameter("obstacles/y", std::vector<double>{});
    oby = get_parameter("obstacles/y").as_double_array();

    declare_parameter("obstacles/r", 0.0);
    obr = get_parameter("obstacles/r").as_double();

    // params relating to physical robot.
    // Shouldn't be required (ie if you are running nusim launch on its own)
    // but need to be there if it's being used for control.
    declare_parameter("wheel_radius", -1.0);
    wheel_radius = get_parameter("wheel_radius").as_double();
    RCLCPP_INFO_STREAM(get_logger(), "Wheel Radius: " << wheel_radius);

    declare_parameter("track_width", -1.0);
    track_width = get_parameter("track_width").as_double();
    RCLCPP_INFO_STREAM(get_logger(), "Track Width: " << track_width);

    declare_parameter("collision_radius", -1.0);
    collision_radius = get_parameter("collision_radius").as_double();
    RCLCPP_INFO_STREAM(get_logger(), "Collision Radius: " << collision_radius);

    declare_parameter("encoder_ticks_per_rad", -1.0);
    encoder_ticks = get_parameter("encoder_ticks_per_rad").as_double();
    RCLCPP_INFO_STREAM(get_logger(), "Encoder Ticks: " << encoder_ticks);

    declare_parameter("motor_cmd_per_rad_sec", -1.0);
    motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();
    RCLCPP_INFO_STREAM(get_logger(), "motor_cmd_per_rad_sec: " << motor_cmd_per_rad_sec);

    declare_parameter("motor_cmd_max", -1);
    motor_cmd_max = get_parameter("motor_cmd_max").as_int();
    RCLCPP_INFO_STREAM(get_logger(), "motor_cmd_max: " << motor_cmd_max);

    declare_parameter("~x_length", 10.0);
    x_length = get_parameter("~x_length").as_double();

    declare_parameter("~y_length", 10.0);
    y_length = get_parameter("~y_length").as_double();

    declare_parameter("draw_only", false);
    draw_only = get_parameter("draw_only").as_bool();

    declare_parameter("do_noise", true);
    do_noise = get_parameter("do_noise").as_bool();

    declare_parameter("input_noise", 1.0);
    input_noise = get_parameter("input_noise").as_double();

    declare_parameter("slip_fraction", 0.05);
    slip_fraction = get_parameter("slip_fraction").as_double();

    declare_parameter("basic_sensor_variance", 0.05);
    basic_sensor_variance = get_parameter("basic_sensor_variance").as_double();

    declare_parameter("max_range", 10.0);
    max_range = get_parameter("max_range").as_double();

    declare_parameter("laser_frame_id", "base_scan");
    laser_frame_id = get_parameter("laser_frame_id").as_string();

    declare_parameter("laser_min_range", 0.11999999731779099);
    laser_min_range = get_parameter("laser_min_range").as_double();

    declare_parameter("laser_max_range", 3.5);
    laser_max_range = get_parameter("laser_max_range").as_double();

    declare_parameter("laser_angle_increment", 0.01745329238474369);
    laser_angle_increment = get_parameter("laser_angle_increment").as_double();

    declare_parameter("laser_nsamples", 360);
    laser_nsamples = get_parameter("laser_nsamples").as_int();

    declare_parameter("laser_resolution", 10.0);
    laser_resolution = get_parameter("laser_resolution").as_double();

    declare_parameter("laser_noise", 10.0);
    laser_noise = get_parameter("laser_noise").as_double();

    commands_dist = std::normal_distribution<>{0, sqrt(input_noise)};
    
    slip_dist = std::uniform_real_distribution<>{-1.0*slip_fraction, slip_fraction};
    
    sensor_dist = std::normal_distribution<>{0, sqrt(basic_sensor_variance)};

    // slightly hacky workaround to get new values into my diff drive object
    auto start_pose = turtlelib::Transform2D(turtlelib::Vector2D{x0, y0}, theta0);
    // turtlelib::DiffDrive temp_robot(start_pose, track_width, wheel_radius);
    robot = turtlelib::DiffDrive{start_pose, track_width, wheel_radius};

    if (obx.size() == oby.size()) {
      // this is a valid input
      RCLCPP_INFO_STREAM(get_logger(), "Valid Marker Input!");
      n_cylinders = obx.size();
    } else {
      RCLCPP_INFO_STREAM(get_logger(), "Marker Inputs not the same size!");
      n_cylinders = 0;
    }

    rate_hz = get_parameter("rate").as_double();
    RCLCPP_INFO_STREAM(get_logger(), "Rate is " << ((int)(1000. / rate_hz)) << "ms");
    std::chrono::milliseconds rate = (std::chrono::milliseconds) ((int)(1000. / rate_hz));

    timestep_pub_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

    obs_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);

    walls_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", 10);

    fake_sensor_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("fake_sensor", 10);

    sensor_pub_ = create_publisher<nuturtlebot_msgs::msg::SensorData>("red/sensor_data", 10);

    path_pub_ = create_publisher<nav_msgs::msg::Path>("~/path", 10);

    laser_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

    wheel_sub_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "red/wheel_cmd", 10, std::bind(&Nusim::wheel_cb, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      rate,
      std::bind(&Nusim::timer_callback, this));

    fake_sensor_timer_ = create_wall_timer(
      200ms,
      std::bind(&Nusim::slower_timer_callback, this));

    reset_ = create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&Nusim::reset, this, std::placeholders::_1, std::placeholders::_2));

    teleport_ = create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(&Nusim::teleport, this, std::placeholders::_1, std::placeholders::_2));

    // From: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/
    // Writing-A-Tf2-Broadcaster-Cpp.html
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    current_time = this->get_clock()->now();
  }

private:
  /// @brief Publish timestep, markers, and transform on each simulation timestep
  /// also update the location of the robot with forward kinematics
  void timer_callback()
  {
    current_time = this->get_clock()->now();

    if(!draw_only){
      auto message = std_msgs::msg::UInt64();
      message.data = timestep_;
      timestep_pub_->publish(message);
      // udpate wheel states
      auto vl = left_velocity;
      auto vr = right_velocity;
      if (do_noise){
        // RCLCPP_INFO_STREAM(get_logger(), "Random # "<<commands_dist(get_random()));
        // RCLCPP_INFO_STREAM(get_logger(), "Mean "<<commands_dist.mean());
        // RCLCPP_INFO_STREAM(get_logger(), "Stddev "<<commands_dist.stddev());
        if (vl != 0.0){
          vl += commands_dist(get_random());
        }
        if (vr != 0.0){
          vr += commands_dist(get_random());
        }
      }
      // RCLCPP_INFO_STREAM(get_logger(), "Before & after: " << left_velocity<< " " << vl);
      const auto ws_left = vl * motor_cmd_per_rad_sec * (1.0 / rate_hz);
      const auto ws_right = vr * motor_cmd_per_rad_sec * (1.0 / rate_hz);
      // udpate robot position with fk
      // this will update in tfs as the broadcaster reads from diff drive object
      robot.fk(ws_left, ws_right);
      // RCLCPP_INFO_STREAM(get_logger(), "Nusim FK: " << ws_left << " and " << ws_right);
      // RCLCPP_INFO_STREAM(get_logger(), "Nusim Pose: " << robot.get_config());
      // update sensor data
      // i am suspicious that it is this simple, but let's try it
      // try calculating stamp with timestep_????
      current_sensor.stamp = current_time;

      if (do_noise){
        // RCLCPP_INFO_STREAM(get_logger(), "Random # "<<slip_dist(get_random()));
        // RCLCPP_INFO_STREAM(get_logger(), "low "<<slip_dist.a());
        // RCLCPP_INFO_STREAM(get_logger(), "hi "<<slip_dist.b());
        left_encoder_save +=  left_velocity * motor_cmd_per_rad_sec * (1.0 / rate_hz) * (1 + slip_dist(get_random())) * encoder_ticks;
        right_encoder_save += right_velocity * motor_cmd_per_rad_sec * (1.0 / rate_hz) * (1 + slip_dist(get_random())) * encoder_ticks;
      } else {
        left_encoder_save += ws_left * encoder_ticks;
        right_encoder_save += ws_right * encoder_ticks;
      }

      current_sensor.left_encoder = left_encoder_save;
      current_sensor.right_encoder = right_encoder_save;
      // sensor_pub_->publish(current_sensor);
      sensor_pub_->publish(current_sensor);
      // send robot transform
      // Before I send the transform, I need to check if it has intersected and update config if so
      for (size_t i = 0; i < n_cylinders; i++) {
        const auto ob_dist = turtlelib::distance(robot.get_x(),robot.get_y(),obx.at(i),oby.at(i));
        if (ob_dist < collision_radius + obr){
          // RCLCPP_INFO_STREAM(get_logger(), "Collision w obstacle " << i);
          // there was a collision. Get unit vector along line from obstacle to robot centers.
          const auto ux = (robot.get_x() - obx.at(i)) / ob_dist;
          const auto uy = (robot.get_y() - oby.at(i)) / ob_dist;
          const auto amount_to_shift = collision_radius + obr - ob_dist;
          const auto new_x = robot.get_x() + amount_to_shift * ux;
          const auto new_y = robot.get_y() + amount_to_shift * uy;
          // update diffdrive object location. Keep same theta
          auto new_pos = turtlelib::Transform2D(turtlelib::Vector2D{new_x, new_y}, robot.get_phi());
          robot.change_state(new_pos);
        }
      }
      // Update the tfs of the robot!
      send_transform();
      timestep_++;
    }
  }

  // publish the fake sensor markers and path at a slower rate
  void slower_timer_callback(){
    if(!draw_only){
      publish_fake_obstacles();
      publish_laser();
      update_path();
    }
    publish_obstacles();
    publish_walls();
  }

  /// @brief Reset the simulation
  /// @param Request: The empty request
  /// @param Response: The empty response
  void reset(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Resetting!");
    timestep_ = 0;
    auto start_pose = turtlelib::Transform2D(turtlelib::Vector2D{x0, y0}, theta0);
    turtlelib::DiffDrive temp(start_pose, track_width, wheel_radius);
    robot = temp;
  }

  /// @brief Teleport the turtle to a given location
  /// @param req contains x, y, and theta to teleport the turtle to
  /// @param res boolean response (if teleport is successful)
  void teleport(
    std::shared_ptr<nusim::srv::Teleport::Request> req,
    std::shared_ptr<nusim::srv::Teleport::Response> res)
  {
    x0 = req->x;
    y0 = req->y;
    theta0 = req->theta;
    RCLCPP_INFO_STREAM(get_logger(), "Teleporting! x=" << x0 << " y=" << y0 << " theta=" << theta0);
    res->success = true;
  }

  /// @brief Broadcast transform between turtle and world frame
  // from here: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/
  // Writing-A-Tf2-Broadcaster-Cpp.html
  void send_transform()
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = current_time;
    // print stamp
    // RCLCPP_INFO_STREAM(get_logger(), "TF S: " << t.header.stamp.sec << " ns " <<
    //                                  t.header.stamp.nanosec);
    // DO THIS TO GET RID OF TF ERROR LOL
    t.header.stamp.nanosec += 3e8;
    // RCLCPP_INFO_STREAM(get_logger(), "TF S: " << t.header.stamp.sec << " ns " <<
    //                                  t.header.stamp.nanosec);
    t.header.frame_id = "nusim/world";
    t.child_frame_id = "red/base_footprint";
    t.transform.translation.x = robot.get_x();
    t.transform.translation.y = robot.get_y();
    t.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, robot.get_phi());
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }

  void update_path(){
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = current_time;
    ps.header.frame_id = "nusim/world";
    ps.pose.position.x = robot.get_x();
    ps.pose.position.y = robot.get_y();
    tf2::Quaternion q;
    q.setRPY(0, 0, robot.get_phi());
    ps.pose.orientation.x = q.x();
    ps.pose.orientation.y = q.y();
    ps.pose.orientation.z = q.z();
    ps.pose.orientation.w = q.w();
    followed_path.poses.push_back(ps);
    // keep array from getting too big!
    if (followed_path.poses.size()>500){
      followed_path.poses.erase(followed_path.poses.begin());
    }
    followed_path.header.stamp = ps.header.stamp;
    followed_path.header.frame_id = "nusim/world";
    path_pub_->publish(followed_path);
  }

  /// @brief publish obstacle marker locations
  void publish_obstacles()
  {
    visualization_msgs::msg::MarkerArray ma;
    for (size_t i = 0; i < n_cylinders; i++) {
      visualization_msgs::msg::Marker m;
      m.header.stamp = current_time;
      m.header.frame_id = "nusim/world";
      m.id = i;         // so each has a unique ID
      m.type = 3;       // cylinder
      m.action = 0;     // add/modify
      // Set color as red
      m.color.r = 1.0;
      m.color.g = 0.0;
      m.color.b = 0.0;
      m.color.a = 1.0;
      // Set Radius
      m.scale.x = 2*obr;
      m.scale.y = 2*obr;
      m.scale.z = 0.25;
      m.pose.position.x = obx.at(i);
      m.pose.position.y = oby.at(i);
      m.pose.position.z = 0.125;
      // Add to marker array
      ma.markers.push_back(m);
    }
    // RCLCPP_INFO_STREAM(get_logger(), "Publishing Marker Array");
    obs_pub_->publish(ma);
  }


  /// @brief publish simulated obstacle marker locations
  void publish_fake_obstacles()
  {
    visualization_msgs::msg::MarkerArray ma;
    for (size_t i = 0; i < n_cylinders; i++) {
      // for each thing
      visualization_msgs::msg::Marker m;
      m.header.stamp = current_time;
      m.header.frame_id = "nusim/world";
      m.id = i;         // so each has a unique ID
      m.type = 3;       // cylinder
      // add marker if within range. Delete if not.
      if (turtlelib::distance(robot.get_x(),robot.get_y(),obx.at(i),oby.at(i)) > max_range){
        m.action = 2; // delete
      } else {
        m.action = 0; // add/modify
      }
      // Set color as yellow
      m.color.r = 1.0;
      m.color.g = 1.0;
      m.color.b = 0.0;
      m.color.a = 1.0;
      // Set Radius
      m.scale.x = 2*obr;
      m.scale.y = 2*obr;
      m.scale.z = 0.25;
      m.pose.position.x = obx.at(i);
      m.pose.position.y = oby.at(i);
      if (do_noise){
        // RCLCPP_INFO_STREAM(get_logger(), "Random # "<<sensor_dist(get_random()));
        // RCLCPP_INFO_STREAM(get_logger(), "Mean "<<sensor_dist.mean());
        // RCLCPP_INFO_STREAM(get_logger(), "Stddev "<<sensor_dist.stddev());
        m.pose.position.x += sensor_dist(get_random());
        m.pose.position.y += sensor_dist(get_random());
      }
      m.pose.position.z = 0.125;
      // Add to marker array
      ma.markers.push_back(m);
    }
    // RCLCPP_INFO_STREAM(get_logger(), "Publishing Marker Array");
    fake_sensor_pub_->publish(ma);
  }

  /// @brief Publish wall marker locations
  void publish_walls()
  {
    visualization_msgs::msg::MarkerArray ma;

    visualization_msgs::msg::Marker m1, m2, m3, m4;
    ma.markers.push_back(m1);
    ma.markers.push_back(m2);
    ma.markers.push_back(m3);
    ma.markers.push_back(m4);

    for (int i = 0; i < 4; i++) {
      ma.markers.at(i).header.stamp = current_time;
      ma.markers.at(i).header.frame_id = "nusim/world";
      ma.markers.at(i).id = i;
      ma.markers.at(i).type = 1;
      ma.markers.at(i).action = 0;
      // set color
      ma.markers.at(i).color.r = 1.0;
      ma.markers.at(i).color.g = 0.0;
      ma.markers.at(i).color.b = 0.0;
      ma.markers.at(i).color.a = 1.0;
      // they are all same z height
      ma.markers.at(i).scale.x = 0.0;
      ma.markers.at(i).scale.y = 0.0;
      ma.markers.at(i).scale.z = 0.25;
      ma.markers.at(i).pose.position.x = 0.0;
      ma.markers.at(i).pose.position.y = 0.0;
      ma.markers.at(i).pose.position.z = 0.125;
    }

    const auto wall_thickness = 0.1;

    ma.markers.at(0).scale.x = wall_thickness;
    ma.markers.at(0).scale.y = y_length + 2 * wall_thickness;
    ma.markers.at(0).pose.position.x = 0.5 * (x_length + wall_thickness);

    ma.markers.at(1).scale.x = x_length + 2 * wall_thickness;
    ma.markers.at(1).scale.y = wall_thickness;
    ma.markers.at(1).pose.position.y = 0.5 * (y_length + wall_thickness);

    ma.markers.at(2).scale.x = wall_thickness;
    ma.markers.at(2).scale.y = y_length + 2 * wall_thickness;
    ma.markers.at(2).pose.position.x = -0.5 * (x_length + wall_thickness);

    ma.markers.at(3).scale.x = x_length + 2 * wall_thickness;
    ma.markers.at(3).scale.y = wall_thickness;
    ma.markers.at(3).pose.position.y = -0.5 * (y_length + wall_thickness);

    walls_pub_->publish(ma);
  }

  /// @brief Publish LaserScan message based on the obstacles/walls
  void publish_laser(){
    sensor_msgs::msg::LaserScan laser;
    laser.header.stamp = current_time;
    // RCLCPP_INFO_STREAM(get_logger(), "LASER S: " << laser.header.stamp.sec << " ns " <<
    //                                  laser.header.stamp.nanosec);
    // copied from turtlebot live message
    laser.header.frame_id = laser_frame_id;
    laser.angle_min = 0.0;
    laser.angle_max = 6.2657318115234375;
    laser.angle_increment = laser_angle_increment;
    laser.time_increment = 0.0005574136157520115;
    laser.scan_time = 0.20066890120506287;
    laser.range_min = laser_min_range;
    laser.range_max = laser_max_range;
    // fill in the laser.ranges array
    const auto dangle = laser.angle_max - laser.angle_min;
    for (int n=0; n<laser_nsamples; n++){
      const auto angle = laser.angle_min + (dangle/laser_nsamples) * n;
      auto measurement = laser_max_range;
      // If not in range, it's 0. If in range, it's the distance.
      const auto xmax = robot.get_x() + laser_max_range*cos(angle + robot.get_phi());
      const auto ymax = robot.get_y() + laser_max_range*sin(angle + robot.get_phi());
      // I should *probably* check if slope = 0 but this is highly unlikely to happen
      const auto slope = (ymax-robot.get_y())/(xmax - robot.get_x());
      // iterate through obstacles
      for (size_t i = 0; i < n_cylinders; i++){
        // here I am solving a quadratic equation in x
        // which is the solution of points of intersection between circle and line
        // another useful constant that got repeated a lot in my derivation
        const auto alpha = robot.get_y() - slope*robot.get_x() - oby.at(i);
        // here are the constants for the equation a x^2 + b x + c = 0
        const auto a = 1 + slope*slope;
        const auto b = 2 * (alpha * slope - obx.at(i));
        const auto c = obx.at(i)*obx.at(i) + alpha*alpha - obr*obr;
        // the determinant ie b^2 - 4ac is what determines the number of solutions
        const auto det = b*b - 4*a*c;
        // if it is <0, there is no solution.
        if (det == 0){
          // 1 solution
          const auto x1 = (-1.0*b)/(2*a);
          const auto y1 = slope * (x1 - robot.get_x()) + robot.get_y();
          // check if the solution is in range
          const auto dst1 = turtlelib::distance(robot.get_x(),robot.get_y(),x1,y1);
          // IF in range AND closer than current measurement, set
          if ((dst1 < laser_max_range) && (dst1 < measurement)){
            measurement = dst1;
          }
        } else if (det > 0) {
          // 2 solutions
          const auto x1 = (-1.0*b + sqrt(det))/(2*a);
          const auto x2 = (-1.0*b - sqrt(det))/(2*a);
          const auto y1 = slope * (x1 - robot.get_x()) + robot.get_y();
          const auto y2 = slope * (x2 - robot.get_x()) + robot.get_y();
          const auto dst1 = turtlelib::distance(robot.get_x(),robot.get_y(),x1,y1);
          const auto dst2 = turtlelib::distance(robot.get_x(),robot.get_y(),x2,y2);
          // FIRST need to check that the intersection is on the right side
          const bool right_side_x1 = ((x1 - robot.get_x())/(xmax - robot.get_x())) > 0;
          const bool right_side_x2 = ((x2 - robot.get_x())/(xmax - robot.get_x())) > 0;
          const bool right_side_y1 = ((y1 - robot.get_y())/(ymax - robot.get_y())) > 0;
          const bool right_side_y2 = ((y2 - robot.get_y())/(ymax - robot.get_y())) > 0;
          const bool right_side_1 = right_side_x1 && right_side_y1;
          const bool right_side_2 = right_side_x2 && right_side_y2;
          // IF in range AND closer than current measurement AND right side, set
          if ((dst1 < laser_max_range) && (dst1 < measurement) && right_side_1){
            measurement = dst1;
          }
          if ((dst2 < laser_max_range) && (dst2 < measurement) && right_side_2){
            measurement = dst2;
          }
        }
      }
      // iterate through the walls
      // wall at x = x_length/2
      const auto w1x = 0.5 * x_length;
      const auto w1y = slope * (w1x - robot.get_x()) + robot.get_y();
      const bool right_side_w1x = (robot.get_x() <= w1x) && (w1x <= xmax);
      const auto dstw1 = turtlelib::distance(robot.get_x(),robot.get_y(),w1x,w1y);
      if ((dstw1 < laser_max_range) && (dstw1 < measurement) && right_side_w1x){
        measurement = dstw1;
      }
      // wall at x = -x_length/2
      const auto w3x = -0.5 * x_length;
      const auto w3y = slope * (w3x - robot.get_x()) + robot.get_y();
      const bool right_side_w3x = (xmax <= w3x) && (w3x <= robot.get_x());
      const auto dstw3 = turtlelib::distance(robot.get_x(),robot.get_y(),w3x,w3y);
      if ((dstw3 < laser_max_range) && (dstw3 < measurement) && right_side_w3x){
        measurement = dstw3;
      }
      // wall at y = y_length/2
      const auto w2y = 0.5 * y_length;
      const auto w2x = (1.0/slope) * (w2y - robot.get_y()) + robot.get_x();
      const bool right_side_w2y = (robot.get_y() <= w2y) && (w2y <= ymax);
      const auto dstw2 = turtlelib::distance(robot.get_x(),robot.get_y(),w2x,w2y);
      if ((dstw2 < laser_max_range) && (dstw2 < measurement) && right_side_w2y){
        measurement = dstw2;
      }
      // wall at y = -y_length/2
      const auto w4y = - 0.5 * y_length;
      const auto w4x = (1.0/slope) * (w4y - robot.get_y()) + robot.get_x();
      const bool right_side_w4y = (ymax <= w4y) && (w4y <= robot.get_y());
      const auto dstw4 = turtlelib::distance(robot.get_x(),robot.get_y(),w4x,w4y);
      if ((dstw4 < laser_max_range) && (dstw4 < measurement) && right_side_w4y){
        measurement = dstw4;
      }
      // check if anything has changed. If there is nothing in range, it should be 0.
      if (measurement == laser_max_range){
        measurement = 0.0;
      }
      // add sensor noise
      if ((do_noise) && (measurement != 0.0)){
        // RCLCPP_INFO_STREAM(get_logger(), "Random # "<<sensor_dist(get_random()));
        // RCLCPP_INFO_STREAM(get_logger(), "Mean "<<sensor_dist.mean());
        // RCLCPP_INFO_STREAM(get_logger(), "Stddev "<<sensor_dist.stddev());
        measurement += sensor_dist(get_random());
      }
      laser.ranges.push_back(measurement);
    }
    // laser.intensities = leave blank
    laser_pub_->publish(laser);
  }

  /// @brief Callback function for receiving wheel commands message
  /// @param wc wheel commands message
  void wheel_cb(const nuturtlebot_msgs::msg::WheelCommands & wc)
  {
    // just store left and right velocity and do this update in the timer
    left_velocity = wc.left_velocity;   // multiply by timestep*period
    right_velocity = wc.right_velocity;
    // RCLCPP_INFO_STREAM(get_logger(), "Receiving "<<wc.left_velocity<<" and "<<wc.right_velocity);
  }

  std::mt19937 & get_random()
  {
    // static variables inside a function are created once and persist for the remainder of the program
    static std::random_device rd{}; 
    static std::mt19937 mt{rd()};
    // we return a reference to the pseudo-random number genrator object. This is always the
    // same object every time get_random is called
    return mt;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr fake_sensor_timer_;

  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obs_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_pub_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;

  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_sub_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  size_t timestep_;
  size_t last_timestep_;
  double x0, y0, theta0, rate_hz;
  // Initializations for the markers
  std::vector<double> obx, oby;
  double obr;
  size_t n_cylinders;
  // for my params
  double wheel_radius, track_width, encoder_ticks, motor_cmd_per_rad_sec, collision_radius;
  int motor_cmd_max;
  nuturtlebot_msgs::msg::SensorData current_sensor;
  // initialize with garbage values, overwrite later
  turtlelib::DiffDrive robot{0.0, 0.0};
  double x_length, y_length;
  bool first_wc = true;
  int32_t left_velocity = 0;
  int32_t right_velocity = 0;
  double left_encoder_save = 0.0;
  double right_encoder_save = 0.0;
  nav_msgs::msg::Path followed_path;
  bool draw_only;
  double input_noise, slip_fraction, basic_sensor_variance, max_range;
  // https://en.cppreference.com/w/cpp/numeric/random/normal_distribution
  std::normal_distribution<> commands_dist;
  std::normal_distribution<> sensor_dist;
  std::uniform_real_distribution<> slip_dist;

  double laser_min_range, laser_max_range, laser_angle_increment, laser_resolution, laser_noise;
  int laser_nsamples;
  bool do_noise;
  std::string laser_frame_id;
  builtin_interfaces::msg::Time current_time;
};

int main(int argc, char * argv[])
{
  /// \brief Spin the node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}
