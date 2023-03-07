#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <armadillo>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "turtlelib/circles.hpp"
#include "turtlelib/rigid2d.hpp"

class Landmarks : public rclcpp::Node
{
  public:
    Landmarks()
    : Node("landmarks")
    {
      laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&Landmarks::laser_cb, this, std::placeholders::_1));
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

    std::vector<std::vector<turtlelib::Polar>> clusters;
    double cluster_threshold = 0.1;

    void laser_cb(const sensor_msgs::msg::LaserScan & msg)
    {
      RCLCPP_INFO_STREAM(get_logger(), "\n\nLaser Received");
      const auto angle_min = msg.angle_min;
      const auto angle_max = msg.angle_max;
      // Cluster is a list of range-bearing measurements
      // at the end I will have a list of clusters
      turtlelib::Polar last_measurement{0.0, 0.0};
      for (int i = 0; i < static_cast<int>(msg.ranges.size()); i++){
        const auto bearing = angle_min + i*(angle_max-angle_min)/(msg.ranges.size());
        const turtlelib::Polar current_measurement{msg.ranges.at(i), bearing};
        // calculate distance between this and last
        if (current_measurement.r > msg.range_min){
          RCLCPP_INFO_STREAM(get_logger(), ""<<current_measurement);
          // then this is a "real" measurement
          // compare it against the last cluster measurement, if it's real.
          if (!turtlelib::atOrigin(last_measurement)){
            const auto dst = turtlelib::polarDistance(current_measurement, last_measurement);
            RCLCPP_INFO_STREAM(get_logger(), "Dist: "<<dst);
            // depending on what dist is, then I either add it to previous cluster or make new one
            if (dst < cluster_threshold){
              RCLCPP_INFO_STREAM(get_logger(), "Add to previous cluster");
            } else {
              RCLCPP_INFO_STREAM(get_logger(), "Make New Cluster");
            }
          } else {
            RCLCPP_INFO_STREAM(get_logger(), "Make New Cluster");
          }
          last_measurement = current_measurement;
        }
      }
    }
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Landmarks>());
  rclcpp::shutdown();
  return 0;
}