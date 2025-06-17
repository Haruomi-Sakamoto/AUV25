from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = "auv25_ros"

    gamepad = Node(
        package='joy',
        executable='joy_node',
        parameters=[{'robot_namespace': 'auv25'}],
        remappings=[('/joy', '/remote_pc/joy')],
    )
    

    j2tw = Node(
        package=package_name,
        executable='j2tw',
        name='j2tw_node',
        namespace='auv25',
        output='screen',
    )

    rqt = Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            )

    return LaunchDescription([
        gamepad,
        #j2tw,
        #rqt,
    ])
