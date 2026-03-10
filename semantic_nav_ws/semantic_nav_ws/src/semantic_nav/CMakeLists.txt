cmake_minimum_required(VERSION 3.8)
project(semantic_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# ---- Semantic Navigator Node ----
add_executable(semantic_navigator src/semantic_navigator.cpp)
ament_target_dependencies(semantic_navigator
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ---- Obstacle Monitor Node ----
add_executable(obstacle_monitor src/obstacle_monitor.cpp)
ament_target_dependencies(obstacle_monitor
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
)

install(TARGETS
  semantic_navigator
  obstacle_monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  worlds
  maps
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
