# AUV25 ROS Project

## 概要
AUV'25 ROS2 プロジェクト

### 1. build&launch

cd ~/colon_ws<br>
colcon build<br>
source install/setup.bash<br>
ros2 launch auv25_ros auv25_launch.py<br>

### 2. topic確認

ros2 topic echo /auv25/sensor_data<br>

### 3. git push

git add .<br>
git commit -m "変更内容"<br>
git push origin main<br>
<br>
