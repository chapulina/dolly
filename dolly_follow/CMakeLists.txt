cmake_minimum_required(VERSION 3.5)

project(dolly_follow)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(target "dolly_follow")

add_executable(${target} src/${target}.cpp)

ament_target_dependencies(${target}
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
