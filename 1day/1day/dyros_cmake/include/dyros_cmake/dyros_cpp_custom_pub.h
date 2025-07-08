#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "dyros_interface/msg/num.hpp"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class DyrosCppPublisher : public rclcpp::Node
{
public:
  DyrosCppPublisher(std::string node_name, std::string topic_name)
  : Node(node_name), count_(0)
  {
    publisher_ = this->create_publisher<dyros_interface::msg::Num>(topic_name, 10);
    timer_ = this->create_wall_timer(
      1s, std::bind(&DyrosCppPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = dyros_interface::msg::Num();
    message.num = count_++;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.num);
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<dyros_interface::msg::Num>::SharedPtr publisher_;
  int32_t count_;

};