cmake_minimum_required(VERSION 3.5)
project(zenoh_ros_services)

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

add_executable(client_main src/client.cpp)
ament_target_dependencies(client_main rclcpp example_interfaces)

add_executable(service_main src/service.cpp)
ament_target_dependencies(service_main rclcpp example_interfaces)

install(TARGETS
  client_main
  service_main
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
