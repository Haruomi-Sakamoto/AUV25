# AUV25

## about
AUV'25 ROS2 Project

### 1. build&launch

cd ~/colon_ws<br>
colcon build<br>
source install/setup.bash<br>
ros2 launch auv25_ros auv25_launch.py<br>

### 2. remote

sudo systemctl start ssh<br>
sudo systemctl enable ssh<br>

cd ~/colon_ws<br>
colcon build<br>
source install/setup.bash<br>
ros2 launch auv25_ros auv25_remotelaunch.py<br>

### 3. check topic

ros2 topic echo /auv25/sensor_data<br>
