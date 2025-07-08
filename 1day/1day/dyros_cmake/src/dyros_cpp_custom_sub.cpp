#include "dyros_cmake/dyros_cpp_custom_sub.h"

int main(int argc, char * argv[])
{
  std::string node_name = "dyros_cpp_sub_node";
  std::string topic_name = "dyros_pub_sub_topic";
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DyrosCppSubscriber>(node_name, topic_name));
  rclcpp::shutdown();
  return 0;
}
