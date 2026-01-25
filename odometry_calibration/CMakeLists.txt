cmake_minimum_required(VERSION 3.5)
project(odometry_calibration)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(visualization_msgs REQUIRED) 
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(odometry_calibration_node src/odometry_calibration_node.cpp)
ament_target_dependencies(odometry_calibration_node rclcpp sensor_msgs nav_msgs)

add_executable(ground_truth_node src/ground_truth_node.cpp)
ament_target_dependencies(ground_truth_node rclcpp sensor_msgs nav_msgs apriltag_msgs tf2_ros tf2_geometry_msgs)

add_executable(polygon_publisher src/polygon_publisher.cpp) 
ament_target_dependencies(polygon_publisher rclcpp visualization_msgs geometry_msgs )

# Install targets
install(TARGETS
  odometry_calibration_node ground_truth_node polygon_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})

ament_package()