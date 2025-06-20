from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'auv25_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='AUV ROS2 package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pingsonner = auv25_ros.pingsonner:main',
            'gpioctrl = auv25_ros.gpioctrl:main',
            'pwmgen = auv25_ros.pwmgen:main',
            'mpu6050 = auv25_ros.mpu6050:main',
            'imu_madgwick = auv25_ros.imu_madgwick:main',
            'imu_q2rpy = auv25_ros.imu_q2rpy:main',
            'j2tw = auv25_ros.j2tw:main',
        ],
    },
)
