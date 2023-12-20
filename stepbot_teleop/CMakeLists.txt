cmake_minimum_required(VERSION 3.8)
project(stepbot_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(ramped_teleop
  src/ramped_teleop.cpp
)

ament_target_dependencies(ramped_teleop
  rclcpp
  geometry_msgs
)

install(TARGETS ramped_teleop
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
