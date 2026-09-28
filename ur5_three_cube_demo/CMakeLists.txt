cmake_minimum_required(VERSION 3.8)
project(ur5_three_cube_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(graph_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Static Planning Scene node
add_executable(
  scene_builder
  src/scene_builder.cpp
)

target_compile_features(
  scene_builder
  PUBLIC
  cxx_std_17
)

ament_target_dependencies(
  scene_builder
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
  geometry_msgs
)

# Four-table hard-environment scene
add_executable(
  hard_scene_builder
  src/hard_scene_builder.cpp
)

target_compile_features(
  hard_scene_builder
  PUBLIC
  cxx_std_17
)

ament_target_dependencies(
  hard_scene_builder
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
  geometry_msgs
)

# Pick-and-place node
add_executable(
  three_cube_pick_place
  src/three_cube_pick_place.cpp
)

target_compile_features(
  three_cube_pick_place
  PUBLIC
  cxx_std_17
)

ament_target_dependencies(
  three_cube_pick_place
  rclcpp
  rclcpp_action
  control_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  graph_msgs
  moveit_msgs
  shape_msgs
  geometry_msgs
)

# Four-table cyclic pick-and-place task
add_executable(
  four_table_cycle
  src/four_table_cycle.cpp
)

target_compile_features(
  four_table_cycle
  PUBLIC
  cxx_std_17
)

ament_target_dependencies(
  four_table_cycle
  rclcpp
  rclcpp_action
  control_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  graph_msgs
  moveit_msgs
  shape_msgs
  geometry_msgs
)

install(
  TARGETS
    scene_builder
    three_cube_pick_place
    hard_scene_builder
    four_table_cycle
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/start_mock_gripper.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
