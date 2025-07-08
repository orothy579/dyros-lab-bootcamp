
# include "dyros_cmake/dyros_cpp_pub.h"


/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

int main(int argc, char * argv[])
{
  std::string node_name = "dyros_cpp_pub_node";
  std::string topic_name = "dyros_pub_sub_topic";
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DyrosCppPublisher>(node_name, topic_name));
  rclcpp::shutdown();
  return 0;
}
