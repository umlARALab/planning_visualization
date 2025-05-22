cmake_minimum_required(VERSION 3.8)
project(action_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_action REQUIRED)

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

add_executable(action_server src/move_robot_server.cpp)
ament_target_dependencies(action_server
  "rclcpp"
  "rclcpp_action"
  "moveit_action"
  "geometry_msgs"
  "tf2_geometry_msgs"
  "moveit_ros_planning_interface"
  "moveit_action"
)

add_executable(action_client src/move_robot_client.cpp)
ament_target_dependencies(action_client
  "rclcpp"
  "rclcpp_action"
  "moveit_action"
  "geometry_msgs"
  "tf2_geometry_msgs"
)


install(TARGETS
  action_server
  action_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
