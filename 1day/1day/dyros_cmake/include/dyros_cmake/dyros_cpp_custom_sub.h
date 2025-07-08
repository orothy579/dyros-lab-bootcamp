#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "dyros_interface/msg/num.hpp"

using std::placeholders::_1;

class DyrosCppSubscriber : public rclcpp::Node
{
public:
  DyrosCppSubscriber(std::string node_name, std::string topic_name)
  : Node(node_name)
  {
    subscription_ = this->create_subscription<dyros_interface::msg::Num>(
      topic_name, 10, std::bind(&DyrosCppSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const dyros_interface::msg::Num & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Subscribing: '%d'", msg.num);
  }
  rclcpp::Subscription<dyros_interface::msg::Num>::SharedPtr subscription_;
};