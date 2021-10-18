cmake_minimum_required(VERSION 3.5)

project(dolly_ignition)

# Fortress
if("$ENV{IGNITION_VERSION}" STREQUAL "fortress")
  find_package(ignition-gazebo6 REQUIRED)

  message(STATUS "Using Ignition Fortress")
# Default to Edifice
else()
  find_package(ignition-gazebo5 QUIET)

  if(NOT ignition-gazebo5_FOUND)
    # Skip if Ignition not present
    message(WARNING "Ignition Gazebo 5 or 6 not found, proceeding without that simulator.")
    return()
  else()
    message(STATUS "Using Ignition Edifice")
  endif()
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
