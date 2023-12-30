cmake_minimum_required(VERSION 3.5)
project(stepbot_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(stepbot_status SHARED
  src/stepbot_status.cpp
  )
target_include_directories(stepbot_status PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(stepbot_status
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  )
rclcpp_components_register_nodes(stepbot_status "stepbot_status::StepbotStatus")
install(TARGETS stepbot_status DESTINATION lib)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
