cmake_minimum_required(VERSION 3.8)
project(stepbot_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs)
find_package(nav_msgs)
find_package(rclcpp_components)
find_package(rclcpp)

add_library(pid_example SHARED src/pid_example.cpp)
ament_target_dependencies(pid_example rclcpp geometry_msgs nav_msgs rclcpp_components)
rclcpp_components_register_nodes(pid_example "pid_example::PidExample")
install(TARGETS pid_example DESTINATION lib)

ament_package()
