import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = "auv25_ros"

    pingsonner = Node(
        package=package_name,
        executable='pingsonner',
        name='pingsonner_node',
        namespace='auv25',
        output='log',
        parameters=[{'robot_namespace': 'auv25'}],
        remappings=[('/sonner_data', '/auv25/sonner_data')],
    )

    gpioctrl = Node(
        package=package_name,
        executable='gpioctrl',
        name='gpioctrl_node',
        namespace='auv25',
        output='log',
    )
    
    pwmgen = Node(
        package=package_name,
        executable='pwmgen',
        name='pwmgen_node',
        namespace='auv25',
        output='log',
    )

    mpu6050 = Node(
        package=package_name,
        executable='mpu6050',
        name='mpu6050_node',
        namespace='auv25',
        output='log',
    )

    imu_madgwick = Node(
        package=package_name,
        executable='imu_madgwick',
        name='imu_madgwick_node',
        namespace='auv25',
        output='log',
    )

    j2tw = Node(
        package=package_name,
        executable='j2tw',
        name='j2tw_node',
        namespace='auv25',
        output='log',
    )
    

    return LaunchDescription([
        #pingsonner,
        #gpioctrl,
        #pwmgen,
        mpu6050,
        imu_madgwick,
        j2tw,
    ])

