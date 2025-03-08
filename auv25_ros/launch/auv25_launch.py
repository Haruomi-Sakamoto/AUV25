import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = "auv25_ros"

    # pingsonner ノード
    pingsonner = Node(
        package=package_name,
        executable='pingsonner',
        name='pingsonner_node',
        namespace='auv25',
        output='log',
        parameters=[{'robot_namespace': 'auv25'}],
        remappings=[('/sonner_data', '/auv25/sonner_data')],
    )

    # GPIO制御ノード
    gpiocontrol = Node(
        package=package_name,
        executable='gpiocontrol',
        name='gpiocontrol_node',
        namespace='auv25',
        output='log',
    )

    return LaunchDescription([
        pingsonner,
        gpiocontrol,
    ])

