from setuptools import setup
import os
from glob import glob

package_name = 'auv25_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
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
        ],
    },
)

