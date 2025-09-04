cmake_minimum_required(VERSION 3.5)
project(check_leg)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Errormsg.msg"
  DEPENDENCIES std_msgs
)


# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

# Install Python modules
install(
  DIRECTORY src/
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()