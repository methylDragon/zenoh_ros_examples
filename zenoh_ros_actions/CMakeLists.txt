cmake_minimum_required(VERSION 3.5)
project(zenoh_ros_actions)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(client_main src/client.cpp)
ament_target_dependencies(client_main rclcpp rclcpp_action example_interfaces)

add_executable(server_main src/server.cpp)
ament_target_dependencies(server_main rclcpp rclcpp_action example_interfaces)

install(TARGETS
  client_main
  server_main
  DESTINATION lib/${PROJECT_NAME})

ament_package()
