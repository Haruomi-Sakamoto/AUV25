from setuptools import setup

package_name = 'auv25_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/auv25_launch.py', 'launch/auv25_remotelaunch.py']),
    ],
    zip_safe=True,
    maintainer='haruomi',
    maintainer_email='haruomi@todo.todo',
    description='AUV sensor and thruster control package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pingsonner = auv25_ros.pingsonner:main',
            'gpioctrl = auv25_ros.gpioctrl:main',
            'pwmtest = auv25_ros.pwmtest:main',
            'mpu6050 = auv25_ros.mpu6050:main',
            'j2tw = auv25_ros.j2tw:main',
            'thctrl = auv25_ros.thctrl:main',
            'pwmgen = auv25_ros.pwmgen:main',
            'serrecv = auv25_ros.serrecv:main',
            'campub = auv25_ros.pubsub:main',
        ],
    },
)
