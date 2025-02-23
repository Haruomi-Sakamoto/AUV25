import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = "auv25_ros"
    return LaunchDescription([
        Node(
            package=package_name,
            executable='pingsonner',
            name='pingsonner_node',
            namespace='auv25',
            output='screen',
            parameters=[{'robot_namespace': 'auv25'}],
            remappings=[('/sensor_data', '/auv25/sensor_data')]
        ),
    ])

