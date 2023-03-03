#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <armadillo>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

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
      RCLCPP_INFO(get_logger(), "Laser Received");
    }
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Landmarks>());
  rclcpp::shutdown();
  return 0;
}