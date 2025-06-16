# AUV25

## about
AUV'25 ROS2 Project

### 1. launch
pi:

cd ~/colon_ws<br>
colcon build<br>
source install/setup.bash<br>
ros2 launch auv25_ros auv25_launch.py<br>

### 2. remote

sudo systemctl start ssh<br>
sudo systemctl enable ssh<br>

### 2. remotelaunch
pi:
sudo systemctl start ssh
sudo systemctl enable ssh

remote pc:

cd ~/colon_ws<br>
colcon build<br>
source install/setup.bash<br>
ros2 launch auv25_ros auv25_remotelaunch.py<br>

### 3. check topic

ros2 topic list<br>
