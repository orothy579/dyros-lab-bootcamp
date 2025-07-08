#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;

class DyrosCppSubscriber : public rclcpp::Node
{
public:
  DyrosCppSubscriber(std::string node_name, std::string topic_name)
  : Node(node_name)
  {
    subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      topic_name, 10, std::bind(&DyrosCppSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::Int32 & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Hello ROS: %d", msg.data);
  }
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};