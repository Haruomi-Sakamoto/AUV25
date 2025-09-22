# AUV25

## about
AUV'25 ROS2 Project

### 1. local launch
pi:<br>
<br>
cd ~/colon_ws<br>
colcon build<br>
source install/setup.bash<br>
ros2 launch auv25_ros auv25_launch.py<br>
<br>
### 2. remote launch
remote pc:<br>
<br>
cd ~/colon_ws<br>
colcon build<br>
source install/setup.bash<br>
ros2 launch auv25_ros auv25_remotelaunch.py<br>
<br>
ssh haruomi@192.168.100.2<br>
cd ~/colon_ws<br>
colcon build<br>
source install/setup.bash<br>
ros2 launch auv25_ros auv25_launch.py<br>
<br>

### 3. check topic

ros2 topic list<br>
