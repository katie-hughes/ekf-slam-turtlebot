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
      const int nclusters = static_cast<int>(clusters.size());
      if (nclusters > 1){
        // save first and last cluster
        auto first_cluster = clusters.at(0);
        auto last_cluster = clusters.at(nclusters-1);
        // check first of first and last of last
        const auto dst = turtlelib::distance(first_cluster.at(0), last_cluster.back());
        RCLCPP_INFO_STREAM(get_logger(), "First and last cluster distance " << dst);
        if (dst < cluster_threshold){
          RCLCPP_INFO_STREAM(get_logger(), "Concatenate first and last clusters");
          // RCLCPP_INFO_STREAM(get_logger(), "Before:");
          // printClusters();
          // concatenate: new first cluster = last_cluster + first_cluster
          last_cluster.insert(last_cluster.end(), first_cluster.begin(), first_cluster.end());
          clusters.at(0) = last_cluster;
          clusters.erase(clusters.end());
        }
      }

      printClusters();

      // do circle detect
      for (int i = 0; i < static_cast<int>(clusters.size()); i++){
        RCLCPP_INFO_STREAM(get_logger(), "Circles in Cluster "<<i);
        // ONLY DO IF THERE ARE MORE THAN 3 POINTS!!!
        if (static_cast<int>(clusters.at(i).size()) > 3){
          const auto detected_circle = turtlelib::detectCircle(clusters.at(i));
          RCLCPP_INFO_STREAM(get_logger(), "Circle: "<<detected_circle);
        } else {
          RCLCPP_INFO_STREAM(get_logger(), "Too Few points");
        }
      }

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