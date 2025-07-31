from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="lab1_pkg",
            executable = "talker",
            name="talker",
            parameters=[{
                'v': 69,
                'd': 5
            }],
            output='screen'
        ),
        Node(
            package="lab1_pkg",
            executable = "relay",
            name="relay",
            output="screen"
        )
    ])