from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    gamepad = Node(
        package='joy',
        executable='joy_node',
        parameters=[{'robot_namespace': 'auv25'}],
        remappings=[('/joy', '/remote_pc/joy')],
    )

    return LaunchDescription([
        gamepad,
    ])
