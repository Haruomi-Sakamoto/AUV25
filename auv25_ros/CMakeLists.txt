cmake_minimum_required(VERSION 3.5)
project(auv25_ros)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# install Python modules
ament_python_install_package(${PROJECT_NAME})

# install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# install resource files
install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
