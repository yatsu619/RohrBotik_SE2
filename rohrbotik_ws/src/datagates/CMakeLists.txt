cmake_minimum_required(VERSION 3.8)
project(datagates)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MarkerInfo.msg"
  DEPENDENCIES sensor_msgs
)

ament_package()