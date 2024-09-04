#
# Copyright (c) 2024 by Laza Medical.
# All Rights Reserved.
#
cmake_minimum_required(VERSION 3.28)
project(sw_watchdog)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sw_watchdog_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_updater)

include_directories(
  include
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS})

### nodes
add_library(${PROJECT_NAME} SHARED
  src/simple_heartbeat.cpp
  src/simple_watchdog.cpp
  src/windowed_watchdog.cpp
  src/watchdog_monitor.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  rcutils
  sw_watchdog_msgs
  std_srvs
  std_msgs
  diagnostic_updater
)
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "SW_WATCHDOG_BUILDING_DLL")
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sw_watchdog::SimpleHeartbeat"
  EXECUTABLE simple_heartbeat)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sw_watchdog::SimpleWatchdog"
  EXECUTABLE simple_watchdog)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sw_watchdog::WindowedWatchdog"
  EXECUTABLE windowed_watchdog)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sw_watchdog::WatchdogMonitor"
  EXECUTABLE watchdog_monitor)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_heartbeat_test test/heartbeat_test.cpp)
  ament_add_gtest(${PROJECT_NAME}_watchdog_test test/watchdog_test.cpp)
  ament_add_gtest(${PROJECT_NAME}_watchdog_monitor_test test/watchdog_monitor_test.cpp)
  ament_add_gtest(${PROJECT_NAME}_windowed_watchdog_test test/windowed_watchdog_test.cpp)

  target_link_libraries(${PROJECT_NAME}_heartbeat_test
    sw_watchdog
  )
  target_link_libraries(${PROJECT_NAME}_watchdog_test
    sw_watchdog
  )
  target_link_libraries(${PROJECT_NAME}_watchdog_monitor_test
    sw_watchdog
  )
  target_link_libraries(${PROJECT_NAME}_windowed_watchdog_test
    sw_watchdog
  )

#   find_package(ros_testing REQUIRED)
#   add_ros_test(
#     test/test_watchdog.py
#     TIMEOUT 60
#   )
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
