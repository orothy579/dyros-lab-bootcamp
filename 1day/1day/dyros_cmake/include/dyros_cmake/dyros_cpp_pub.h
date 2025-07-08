#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class DyrosCppPublisher : public rclcpp::Node
{
public:
  DyrosCppPublisher(std::string node_name, std::string topic_name)
  : Node(node_name), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Int32>(topic_name, 10);
    timer_ = this->create_wall_timer(
      1s, std::bind(&DyrosCppPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Int32();
    message.data = count_++;
    RCLCPP_INFO(this->get_logger(), "Publishing: %d", message.data);
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  int32_t count_;

};