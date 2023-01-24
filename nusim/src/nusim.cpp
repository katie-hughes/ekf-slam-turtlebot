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

using namespace std::chrono_literals;

class Nusim : public rclcpp::Node
{
public:
  Nusim()
  : Node("nusim"), timestep_(0)
  {
    this->declare_parameter("rate", 200.);
    this->declare_parameter("x0", 0.);
    this->declare_parameter("y0", 0.);
    this->declare_parameter("theta0", 0.);

    // shoudl default to empty list. I feel like this is a very hacky solution?
    this->declare_parameter("obstacles/x", obx);
    this->declare_parameter("obstacles/y", oby);
    this->declare_parameter("obstacles/r", obr);

    x0 = this->get_parameter("x0").as_double();
    y0 = this->get_parameter("y0").as_double();
    theta0 = this->get_parameter("theta0").as_double();

    obx = this->get_parameter("obstacles/x").as_double_array();
    oby = this->get_parameter("obstacles/y").as_double_array();
    obr = this->get_parameter("obstacles/r").as_double();

    if (obx.size() == oby.size()) {
      // this is a valid input
      RCLCPP_INFO_STREAM(get_logger(), "Valid Marker Input!");
      n_cylinders = obx.size();
    } else {
      RCLCPP_INFO_STREAM(get_logger(), "Marker Inputs not the same size!");
      n_cylinders = 0;
    }

    auto rate_param = this->get_parameter("rate").as_double();
    RCLCPP_INFO_STREAM(get_logger(), "Rate is " << ((int)(1000. / rate_param)) << "ms");
    std::chrono::milliseconds rate = (std::chrono::milliseconds) ((int)(1000. / rate_param));

    timestep_pub_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);

    timer_ = this->create_wall_timer(
      rate,
      std::bind(&Nusim::timer_callback, this));

    reset_ = this->create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&Nusim::reset, this, std::placeholders::_1, std::placeholders::_2));

    teleport_ = this->create_service<nusim::srv::Teleport>(
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
    send_transform();
    publish_markers();
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
    t.transform.translation.x = x0;
    t.transform.translation.y = y0;
    t.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }

  void publish_markers()
  {
    /// \brief Publish marker locations
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
    marker_pub_->publish(ma);
  }

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  size_t timestep_;
  double x0, y0, theta0;
  // Initializations for the markers
  std::vector<double> obx, oby;
  double obr;
  int n_cylinders;
};

int main(int argc, char * argv[])
{
  /// \brief Spin the node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}
