import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = "auv25_ros"
    
    mpu6050 = Node(
        package=package_name,
        executable='mpu6050',
        name='mpu6050_node',
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

    serrecv = Node(
        package=package_name,
        executable='serrecv',
        name='serrecv_node',
        namespace='auv25',
        output='log',
    )

    thctrl = Node(
        package=package_name,
        executable='thctrl',
        name='thctrl_node',
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
    

    return LaunchDescription([
        #pingsonner,
        #gpioctrl,
        #pwmgen,
        mpu6050,
        j2tw,
        serrecv,
        thctrl,
        pwmgen,
    ])

