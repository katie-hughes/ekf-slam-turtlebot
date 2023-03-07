#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <armadillo>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "turtlelib/circles.hpp"

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

    void laser_cb(const sensor_msgs::msg::LaserScan & msg)
    {
      RCLCPP_INFO_STREAM(get_logger(), "Laser Received");
      const auto angle_min = msg.angle_min;
      const auto angle_max = msg.angle_max;
      for (int i = 0; i < static_cast<int>(msg.ranges.size()); i++){
        const auto bearing = angle_min + i*(angle_max-angle_min)/(msg.ranges.size());
        // RCLCPP_INFO_STREAM(get_logger(), i << ": "<<msg.ranges.at(i));
        const turtlelib::RangeBearing rb{msg.ranges.at(i), bearing};
        // RCLCPP_INFO_STREAM(get_logger(), ""<<rb);
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