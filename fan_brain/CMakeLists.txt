cmake_minimum_required(VERSION 3.5)
project(fan_brain)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(fan_brain src/FanBrain.cpp)
ament_target_dependencies(
  fan_brain
  rclcpp
  std_msgs
  )

install(TARGETS
  fan_brain
  DESTINATION lib/${PROJECT_NAME})

ament_package()
