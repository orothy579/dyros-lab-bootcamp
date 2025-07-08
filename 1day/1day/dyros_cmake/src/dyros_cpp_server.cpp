#include "rclcpp/rclcpp.hpp"
#include "dyros_interface/srv/add_three_ints.hpp"                                        

#include <memory>

void add(const std::shared_ptr<dyros_interface::srv::AddThreeInts::Request> request,     
          std::shared_ptr<dyros_interface::srv::AddThreeInts::Response>       response)  
{
  response->sum = request->a + request->b + request->c;                                      
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %d" " b: %d" " c: %d",  
                request->a, request->b, request->c);                                         
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%d]", (int)response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_server");   

  rclcpp::Service<dyros_interface::srv::AddThreeInts>::SharedPtr service =               
    node->create_service<dyros_interface::srv::AddThreeInts>("add_three_ints",  &add);   

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add three ints.");                     

  rclcpp::spin(node);
  rclcpp::shutdown();
}