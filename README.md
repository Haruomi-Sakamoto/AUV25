#AUV25 ROS Project

## 概要
AUV'25 ros2 project

### 1. build&launch
```bash
cd ~/colon_ws
colcon build
source install/setup.bash
ros2 launch auv25_ros auv25_launch.py

### 2. topic確認
ros2 topic echo /auv25/sensor_data

### 3. git push

git add .
git commit -m
git push origin main
