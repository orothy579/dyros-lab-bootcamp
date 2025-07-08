
#include "dyros_cmake/dyros_cpp_custom_pub.h"

int main(int argc, char * argv[])
{
  std::string node_name = "dyros_cpp_pub_node";
  std::string topic_name = "dyros_pub_sub_topic";
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DyrosCppPublisher>(node_name, topic_name));
  rclcpp::shutdown();
  return 0;
}
