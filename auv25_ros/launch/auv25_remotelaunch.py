from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    gamepad = Node(
        package='joy',
        executable='joy_node',
        parameters=[{'robot_namespace': 'auv25'}],
        remappings=[('/joy', '/remote_pc/joy')],
    )

    camsub = Node(
        package='auv25_ros',
        executable='camsub',
        name='camsub_node',
        output='screen'
    )

    return LaunchDescription([
        gamepad,
        camsub,
    ])
