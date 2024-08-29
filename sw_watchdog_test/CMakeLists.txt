#
# Copyright (c) 2024 by Laza Medical.
# All Rights Reserved.
#
cmake_minimum_required(VERSION 3.28)
project(sw_watchdog_test)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sw_watchdog_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS})

### nodes
add_library(${PROJECT_NAME} SHARED
  src/TestNode.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  rcutils
  std_srvs
  std_msgs
  sw_watchdog_msgs
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sw_watchdog_test::TestNode"
  EXECUTABLE TestNode)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

  # This links against the gtest libraries, but does not register it
  # as a test
  ament_add_gtest_executable(watchdog_integ_test test/watchdog_integ_test.cpp)

  ament_target_dependencies(watchdog_integ_test
    rclcpp
    std_msgs
    std_srvs
    sw_watchdog_msgs
    lifecycle_msgs
  )

  install(TARGETS
    watchdog_integ_test
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  add_launch_test(test/watchdog_integ_test.py)

endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
