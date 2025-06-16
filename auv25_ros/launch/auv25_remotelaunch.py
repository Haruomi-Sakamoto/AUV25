from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = "auv25_ros"

    gamepad = Node(
        package='joy',
        executable='joy_node',
    )

    j2tw = Node(
        package=package_name,
        executable='j2tw',
        name='j2tw_node',
        namespace='auv25',
        output='screen',
    )

    return LaunchDescription([
        gamepad,
        j2tw,
    ])
