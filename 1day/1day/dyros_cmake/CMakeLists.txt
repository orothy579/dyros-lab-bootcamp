cmake_minimum_required(VERSION 3.8)
project(dyros_cmake)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dyros_interface REQUIRED)

include_directories(include)

add_executable(dyros_cpp_pub src/dyros_cpp_pub.cpp)
ament_target_dependencies(dyros_cpp_pub rclcpp std_msgs)

add_executable(dyros_cpp_sub src/dyros_cpp_sub.cpp)
ament_target_dependencies(dyros_cpp_sub rclcpp std_msgs)

add_executable(dyros_cpp_custom_pub src/dyros_cpp_custom_pub.cpp)
ament_target_dependencies(dyros_cpp_custom_pub rclcpp dyros_interface)

add_executable(dyros_cpp_custom_sub src/dyros_cpp_custom_sub.cpp)
ament_target_dependencies(dyros_cpp_custom_sub rclcpp dyros_interface)

add_executable(dyros_cpp_server src/dyros_cpp_server.cpp)
ament_target_dependencies(dyros_cpp_server rclcpp dyros_interface)

add_executable(dyros_cpp_client src/dyros_cpp_client.cpp)
ament_target_dependencies(dyros_cpp_client rclcpp dyros_interface)


install(TARGETS
  dyros_cpp_pub
  dyros_cpp_sub
  dyros_cpp_custom_pub
  dyros_cpp_custom_sub
  dyros_cpp_client
  dyros_cpp_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
