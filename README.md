# AUV25

## about
AUV'25 ROS2 Project

### 1. build&launch

RaspberryPi5:
cd ~/colon_ws<br>
colcon build<br>
source install/setup.bash<br>
ros2 launch auv25_ros auv25_launch.py<br>

remotePC:
wsl


cd ~/colon_ws<br>
colcon build<br>
source install/setup.bash<br>
ros2 launch auv25_ros auv25_launch.py<br>

### 2. check topic

ros2 topic echo /auv25/sensor_data<br>
