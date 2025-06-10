import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    gamepad = Node(
            package='joy',
            executable='joy_node',
            )
    launch_description_list.append(joy_node)

    return LaunchDescription([
        gamepad,
    ])