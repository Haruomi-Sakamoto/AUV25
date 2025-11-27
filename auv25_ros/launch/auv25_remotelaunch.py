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

    autocmd = Node(
        package='auv25_ros',
        executable='autocmd',
        name='autocmd_node',
        output='screen'
    )

    j2tw = Node(
        package='auv25_ros',
        executable='j2tw',
        name='j2tw_node',
        namespace='auv25',
        output='log',
    )

    amslc = Node(
        package='auv25_ros',
        executable='amslc',
        name='amslc_node',
        output='screen'
    )

    logger = Node(
        package='auv25_ros',
        executable='logger',
        name='logger_node',
        output='screen'
    )

    return LaunchDescription([
        gamepad,
        camsub,
        autocmd,
        j2tw,
        amslc,
        logger,
    ])
