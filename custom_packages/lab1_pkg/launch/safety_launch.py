from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="lab1_pkg",
            executable = "safety",
            name="safety",
            output="screen"
        )
    ])