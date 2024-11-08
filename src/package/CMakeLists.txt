cmake_minimum_required(VERSION 3.5)
project(package)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators 
REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add custom message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedPerson.msg"
  "msg/DetectedPersonArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/lidar_processing_node.py
  src/marker_visualization_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_package()

