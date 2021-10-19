cmake_minimum_required(VERSION 3.5)

project(dolly_tests)

# Fortress
if("$ENV{IGNITION_VERSION}" STREQUAL "fortress")
  find_package(ignition-gazebo6 REQUIRED)
  set(IGN_GAZEBO_VER 6)

  message(STATUS "Compiling against Ignition Fortress")
# Default to Edifice
else()
  find_package(ignition-gazebo5 QUIET)
  set(IGN_GAZEBO_VER 5)

  if(NOT ignition-gazebo5_FOUND)
    # Skip if Ignition not present
    message(WARNING "Ignition Gazebo 5 or 6 not found, proceeding without that simulator.")
    return()
  else()
    message(STATUS "Compiling against Ignition Edifice")
  endif()
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_find_gtest()

  set("PROJECT_SOURCE_PATH" ${CMAKE_CURRENT_SOURCE_DIR})
  configure_file(test/constants.hh.in constants.hh @ONLY)

  set(TEST_NAME follow_ignition_TEST)
  ament_add_gtest_executable(${TEST_NAME} test/${TEST_NAME}.cpp)
  target_link_libraries(${TEST_NAME}
    ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
  )
  include_directories(${CMAKE_CURRENT_BINARY_DIR})
  install(
    TARGETS ${TEST_NAME}
    DESTINATION lib/${PROJECT_NAME}
  )
  add_launch_test(launch/${TEST_NAME}.launch.py
    TIMEOUT 200
  )
endif()

install(
  DIRECTORY
    worlds
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
