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

    std::vector<std::vector<turtlelib::Vector2D>> clusters;
    double cluster_threshold = 0.1;

    void laser_cb(const sensor_msgs::msg::LaserScan & msg)
    {
      RCLCPP_INFO_STREAM(get_logger(), "\n\nLaser Received");
      const auto angle_min = msg.angle_min;
      const auto angle_max = msg.angle_max;
      // Cluster is a list of range-bearing measurements
      // at the end I will have a list of clusters
      turtlelib::Vector2D last_measurement;
      std::vector<turtlelib::Vector2D> current_cluster;
      bool first_measurement = true;
      for (int i = 0; i < static_cast<int>(msg.ranges.size()); i++){
        const auto bearing = angle_min + i*(angle_max-angle_min)/(msg.ranges.size());
        const turtlelib::Polar current_polar{msg.ranges.at(i), bearing};
        const turtlelib::Vector2D current_measurement = toVector(current_polar);
        // calculate distance between this and last
        if (current_polar.r > msg.range_min){
          // RCLCPP_INFO_STREAM(get_logger(), ""<<current_measurement);
          // then this is a "real" measurement
          // compare it against the last cluster measurement, if it's real.
          if (first_measurement){
            current_cluster.push_back(current_measurement);
            first_measurement = false;
          } else {
            const auto dst = turtlelib::distance(current_measurement, last_measurement);
            // RCLCPP_INFO_STREAM(get_logger(), "Dist: "<<dst);
            // depending on what dist is, then I either add it to previous cluster or make new one
            if (dst < cluster_threshold){
              // add to current cluster
              // RCLCPP_INFO_STREAM(get_logger(), "Add to previous cluster");
              current_cluster.push_back(current_measurement);
            } else {
              // make a new cluster
              // RCLCPP_INFO_STREAM(get_logger(), "Make New Cluster");
              clusters.push_back(current_cluster);
              current_cluster = std::vector<turtlelib::Vector2D>{current_measurement};
            }
          }
          last_measurement = current_measurement;
        }
      }
      // add the last cluster
      if (current_cluster.size() != 0){
        clusters.push_back(current_cluster);
      }
      // check the first index of first cluster and the last index of the last cluster
      // 

      printClusters();

      clusters.clear();
    }

    void printClusters(){
      for (int i = 0; i < static_cast<int>(clusters.size()); i++){
        RCLCPP_INFO_STREAM(get_logger(), "Cluster "<<i);
        for (int j = 0; j < static_cast<int>(clusters.at(i).size()); j++){
          RCLCPP_INFO_STREAM(get_logger(), ""<<clusters.at(i).at(j));
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