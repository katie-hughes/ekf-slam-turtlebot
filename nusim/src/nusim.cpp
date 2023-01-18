#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Nusim : public rclcpp::Node
{
  public:
    Nusim()
    : Node("nusim"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~timestep", 10);
      timer_ = this->create_wall_timer(
        200ms,
        std::bind(&Nusim::timer_callback, this));

      rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_;

      reset_ = this->create_service<std_srvs::srv::Empty>(
        "~reset",
        std::bind(&Nusim::reset, this, std::placeholders::_1, std::placeholders::_2));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::UInt64();
      message.data = count_;
      RCLCPP_INFO_STREAM(get_logger(), "Publishing: '" << message.data << "'");
      publisher_->publish(message);
      count_++;
    }
    void reset(std_srvs::srv::Empty::Request::SharedPtr req,
                      std_srvs::srv::Empty::Response::SharedPtr res){
        count_ = 0;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}