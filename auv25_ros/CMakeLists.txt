cmake_minimum_required(VERSION 3.5)
project(auv25_ros)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorPacket.msg"
)

ament_package()
