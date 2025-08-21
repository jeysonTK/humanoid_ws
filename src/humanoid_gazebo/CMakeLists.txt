cmake_minimum_required(VERSION 3.5)
project(humanoid_gazebo)

# Find ament_cmake which includes launch support
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
