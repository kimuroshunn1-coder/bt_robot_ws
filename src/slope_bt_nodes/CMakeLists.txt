cmake_minimum_required(VERSION 3.8)
project(slope_bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(
  slope_up_detector
  src/slope_up_detector.cpp
)

ament_target_dependencies(
  slope_up_detector
  rclcpp
  sensor_msgs
  nav_msgs
)

add_library(
  slope_bt_plugin SHARED
  src/plugin.cpp
  src/slope_up_node.cpp
  src/slope_down_node.cpp
  src/move_forward_node.cpp
  src/move_back_node.cpp
  src/follow_waypoints_nav2_node.cpp
)

target_compile_definitions(
  slope_bt_plugin
  PRIVATE BT_PLUGIN_EXPORT
)

target_include_directories(
  slope_bt_plugin
  PUBLIC
  include
)

ament_target_dependencies(
  slope_bt_plugin
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  behaviortree_cpp
)

target_compile_features(
  slope_bt_plugin
  PUBLIC cxx_std_17
)

install(
  TARGETS
    slope_up_detector
    slope_bt_plugin

  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

add_executable(
  test_bt
  src/test_main.cpp
)

target_include_directories(
  test_bt
  PUBLIC include
)

install(
  TARGETS slope_bt_plugin
  LIBRARY DESTINATION lib
)

ament_target_dependencies(
  test_bt
  behaviortree_cpp
  rclcpp
)

target_link_libraries(
  test_bt
  slope_bt_plugin
)

install(
  TARGETS test_bt
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
