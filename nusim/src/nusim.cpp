/// \file
/// \brief Simulates a turtlebot.
///
/// PARAMETERS:
///     parameter_name (parameter_type): description of the parameter
///     rate (double): frequency of the timer, in Hz
///     x0 (double): starting x location of the turtlebot (m)
///     y0 (double): starting y location of the turtlebot (m)
///     theta0 (double): starting theta location of the turtlebot (rad)
///     obstacles/x (double[]): list of x coordinates of cylindrical obstacles (m)
///     obstacles/y (double[]): list of r coordinates of cylindrical obstacles (m)
///     obstacles/r (double): radius of cylindrical obstacles (m)
/// PUBLISHES:
///     ~/timestep (std_msgs::msg::Uint64): current timestep of simulation
///     ~/obstacles (visualization_msgs::msg::MarkerArray): marker objects representing cylinders
/// SUBSCRIBES:
///     none
/// SERVERS:
///     ~/reset (std_srvs::srv::Empty): resets the simulation to the initial state
///     ~/teleport (nusim::srv::Teleport): teleports the turtle to a given x, y, theta value
/// CLIENTS:
///     none


#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nusim/srv/teleport.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"

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

    // shoudl default to empty list. I feel like this is a very hacky solution?
    declare_parameter("obstacles/x", obx);
    obx = get_parameter("obstacles/x").as_double_array();

    declare_parameter("obstacles/y", oby);
    oby = get_parameter("obstacles/y").as_double_array();

    declare_parameter("obstacles/r", obr);
    obr = get_parameter("obstacles/r").as_double();

    // params relating to physical robot.
    // Shouldn't be required (ie if you are running nusim launch on its own)
    // but need to be there if it's being used for control.
    declare_parameter("wheel_radius",-1.0);
    wheel_radius = get_parameter("wheel_radius").as_double();
    RCLCPP_INFO_STREAM(get_logger(), "Wheel Radius: "<<wheel_radius);

    declare_parameter("track_width",-1.0);
    track_width = get_parameter("track_width").as_double();
    RCLCPP_INFO_STREAM(get_logger(), "Track Width: "<<track_width);

    declare_parameter("encoder_ticks_per_rad",-1.0);
    encoder_ticks = get_parameter("encoder_ticks_per_rad").as_double();
    RCLCPP_INFO_STREAM(get_logger(), "Encoder Ticks: "<<encoder_ticks);

    declare_parameter("motor_cmd_per_rad_sec",-1.0);
    motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();
    RCLCPP_INFO_STREAM(get_logger(), "motor_cmd_per_rad_sec: "<<motor_cmd_per_rad_sec);

    declare_parameter("motor_cmd_max",-1);
    motor_cmd_max = get_parameter("motor_cmd_max").as_int();
    RCLCPP_INFO_STREAM(get_logger(), "motor_cmd_max: "<<motor_cmd_max);

    declare_parameter("~x_length", 10.0);
    x_length = get_parameter("~x_length").as_double();

    declare_parameter("~y_length", 10.0);
    y_length = get_parameter("~y_length").as_double();

    // slightly hacky workaround to get new values in
    auto start_pose = turtlelib::Transform2D(turtlelib::Vector2D{x0, y0}, theta0);
    turtlelib::DiffDrive temp(start_pose, track_width, wheel_radius);
    robot = temp;

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

    sensor_pub_ = create_publisher<nuturtlebot_msgs::msg::SensorData>("red/sensor_data", 10);

    wheel_sub_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
        "red/wheel_cmd", 10, std::bind(&Nusim::wheel_cb, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      rate,
      std::bind(&Nusim::timer_callback, this));

    reset_ = create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&Nusim::reset, this, std::placeholders::_1, std::placeholders::_2));

    teleport_ = create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(&Nusim::teleport, this, std::placeholders::_1, std::placeholders::_2));

    // From: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/
    // Writing-A-Tf2-Broadcaster-Cpp.html
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  void timer_callback()
  {
    /// \brief Publish timestep, markers, and transform on each simulation timestep
    auto message = std_msgs::msg::UInt64();
    message.data = timestep_;
    //   RCLCPP_INFO_STREAM(get_logger(), "Timestep: " << message.data);
    timestep_pub_->publish(message);
    // current_sensor.stamp = this->get_clock()->now();
    sensor_pub_->publish(current_sensor);
    // send robot transform
    send_transform();
    publish_obstacles();
    publish_walls();
    timestep_++;
  }

  void reset(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    /// \brief Reset the simulation
    ///
    /// \param Request: The empty request
    /// \param Response: The empty response
    RCLCPP_INFO_STREAM(get_logger(), "Resetting!");
    timestep_ = 0;
    auto start_pose = turtlelib::Transform2D(turtlelib::Vector2D{x0, y0}, theta0);
    turtlelib::DiffDrive temp(start_pose, track_width, wheel_radius);
    robot = temp;
  }

  void teleport(
    std::shared_ptr<nusim::srv::Teleport::Request> req,
    std::shared_ptr<nusim::srv::Teleport::Response> res)
  {
    /// \brief Teleport the turtle
    ///
    /// \param req: contains x, y, and theta to teleport the turtle to
    /// \param res: boolean response (if teleport is successful)
    x0 = req->x;
    y0 = req->y;
    theta0 = req->theta;
    RCLCPP_INFO_STREAM(get_logger(), "Teleporting! x=" << x0 << " y=" << y0 << " theta=" << theta0);
    res->success = true;
  }

  void send_transform()
  {
    /// \brief Broadcast the transform between the turtle's coordinates and world frame
    // from here: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/
    // Writing-A-Tf2-Broadcaster-Cpp.html
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
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

  void publish_obstacles()
  {
    /// \brief Publish obstacle marker locations
    visualization_msgs::msg::MarkerArray ma;
    for (int i = 0; i < n_cylinders; i++) {
      visualization_msgs::msg::Marker m;
      m.header.stamp = this->get_clock()->now();
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
      m.scale.x = obr;
      m.scale.y = obr;
      m.scale.z = 0.25;
      m.pose.position.x = obx[i];
      m.pose.position.y = oby[i];
      m.pose.position.z = 0.125;
      // Add to marker array
      ma.markers.push_back(m);
    }
    // RCLCPP_INFO_STREAM(get_logger(), "Publishing Marker Array");
    obs_pub_->publish(ma);
  }

  void publish_walls()
  {
    /// \brief Publish wall marker locations
    visualization_msgs::msg::MarkerArray ma;

    visualization_msgs::msg::Marker m1, m2, m3, m4;
    ma.markers.push_back(m1);
    ma.markers.push_back(m2);
    ma.markers.push_back(m3);
    ma.markers.push_back(m4);

    for(int i=0; i<4; i++){
      ma.markers.at(i).header.stamp = this->get_clock()->now();
      ma.markers.at(i).header.frame_id = "nusim/world";
      ma.markers.at(i).id = i;
      ma.markers.at(i).type = 1;
      ma.markers.at(i).action = 0;
      // set color
      ma.markers.at(i).color.r = 1.0;
      ma.markers.at(i).color.g = 1.0;
      ma.markers.at(i).color.b = 1.0;
      ma.markers.at(i).color.a = 1.0;
      // they are all same z height
      ma.markers.at(i).scale.x = 0.0;
      ma.markers.at(i).scale.y = 0.0;
      ma.markers.at(i).scale.z = 0.25;
      ma.markers.at(i).pose.position.x = 0.0;
      ma.markers.at(i).pose.position.y = 0.0;
      ma.markers.at(i).pose.position.z = 0.125;
    }

    ma.markers.at(0).scale.y = y_length;
    ma.markers.at(0).pose.position.x = 0.5*x_length;

    ma.markers.at(1).scale.x = x_length;
    ma.markers.at(1).pose.position.y = 0.5*y_length;

    ma.markers.at(2).scale.y = y_length;
    ma.markers.at(2).pose.position.x = -0.5*x_length;

    ma.markers.at(3).scale.x = x_length;
    ma.markers.at(3).pose.position.y = -0.5*y_length;

    walls_pub_->publish(ma);
  }

  void wheel_cb(const nuturtlebot_msgs::msg::WheelCommands & wc)
    {
      if(!first_wc){
        int left_velocity = wc.left_velocity; // multiply by timestep*period
        int right_velocity = wc.right_velocity;
        // RCLCPP_INFO_STREAM(get_logger(), "Wheel Vels:"<<left_velocity<<" and "<<right_velocity);
        // convert wheel commands to sensor data
        double dt = (timestep_-last_timestep_)/rate_hz;
        // RCLCPP_INFO_STREAM(get_logger(), "dt: "<<dt);
        double ws_left = left_velocity*motor_cmd_per_rad_sec*dt;
        double ws_right = right_velocity*motor_cmd_per_rad_sec*dt;
        // udpate robot position with fk
        // this will update in tfs as the broadcaster reads from diff drive object
        robot.fk(ws_left, ws_right);
        RCLCPP_INFO_STREAM(get_logger(), "Nusim Pose: "<<robot.get_config());
        // update sensor data
        // i am suspicious that it is this simple, but let's try it
        current_sensor.stamp = this->get_clock()->now();
        current_sensor.left_encoder += ws_left*encoder_ticks;
        current_sensor.right_encoder += ws_right*encoder_ticks;
        sensor_pub_->publish(current_sensor);
      }else{
        first_wc = false;
      }
      last_timestep_ = timestep_;
      
    }

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obs_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_pub_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_pub_;

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
  int n_cylinders;
  // for my params
  double wheel_radius, track_width, encoder_ticks, motor_cmd_per_rad_sec;
  int motor_cmd_max;
  nuturtlebot_msgs::msg::SensorData current_sensor;
  // initialize with garbage values, overwrite later
  turtlelib::DiffDrive robot{0.0, 0.0};
  double x_length, y_length;
  bool first_wc = true;
};

int main(int argc, char * argv[])
{
  /// \brief Spin the node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}
