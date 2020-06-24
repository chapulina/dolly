cmake_minimum_required(VERSION 3.5)

project(dolly_ignition)

# Skip if Ignition not present
find_package(ignition-gazebo3 QUIET)
if(NOT ignition-gazebo3_FOUND)
  message(WARNING "Ignition Gazebo 3 not found, proceeding without that simulator.")
  return()
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    ign
    launch
    models
    rviz
    worlds
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
