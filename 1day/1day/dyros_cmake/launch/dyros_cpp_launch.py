from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="dyros_cmake",
            executable = "dyros_cpp_pub",
            name = "dyros_cpp_pub"
        ),
        Node(
            package="dyros_cmake",
            executable = "dyros_cpp_sub",
            name = "dyros_cpp_sub_1"
        ),
        Node(
            package="dyros_cmake",
            executable = "dyros_cpp_sub",
            name = "dyros_cpp_sub_2"
        )
    ])
