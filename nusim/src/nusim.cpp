#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nusim/srv/teleport.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Nusim : public rclcpp::Node
{
  public:
    Nusim()
    : Node("nusim"), count_(0)
    {
      this->declare_parameter("rate", 200);
      this->declare_parameter("x0", 0.);
      this->declare_parameter("y0", 0.);
      this->declare_parameter("theta0", 0.);

      x0 = this->get_parameter("x0").as_double();
      y0 = this->get_parameter("y0").as_double();
      theta0 = this->get_parameter("theta0").as_double();

      auto rate_param = this->get_parameter("rate");
      std::chrono::milliseconds rate = (std::chrono::milliseconds) (rate_param.as_int());

      publisher_ = this->create_publisher<std_msgs::msg::UInt64>("timestep", 10);

      timer_ = this->create_wall_timer(
        rate,
        std::bind(&Nusim::timer_callback, this));

      reset_ = this->create_service<std_srvs::srv::Empty>(
        "reset",
        std::bind(&Nusim::reset, this, std::placeholders::_1, std::placeholders::_2));
    
      teleport_ = this->create_service<nusim::srv::Teleport>(
        "teleport",
        std::bind(&Nusim::teleport, this, std::placeholders::_1, std::placeholders::_2));
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

    void reset(std::shared_ptr<std_srvs::srv::Empty::Request> req,
               std::shared_ptr<std_srvs::srv::Empty::Response> res){
        (void)req;
        (void)res;
        RCLCPP_INFO_STREAM(get_logger(), "Resetting!");
        count_ = 0;
    }

    void teleport(std::shared_ptr<nusim::srv::Teleport::Request> req,
                  std::shared_ptr<nusim::srv::Teleport::Response> res){
        x0 = req->x;
        y0 = req->y;
        theta0 = req->theta;
        RCLCPP_INFO_STREAM(get_logger(), "Teleporting! x="<<x0<<" y="<<y0<<" theta="<<theta0);
        res->success = true;
    }

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_;

    rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_;

    size_t count_;

    double x0, y0, theta0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}