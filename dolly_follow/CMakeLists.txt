cmake_minimum_required(VERSION 3.5)

project(dolly_follow)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(target "dolly_follow")

add_executable(${target} src/${target}.cpp)

ament_target_dependencies(${target}
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs")

install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
