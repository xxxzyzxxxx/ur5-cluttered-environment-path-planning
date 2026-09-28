#include <array>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

using namespace std::chrono_literals;

moveit_msgs::msg::CollisionObject makeBox(
  const std::string & id,
  const std::string & frame_id,
  const std::array<double, 3> & size,
  const std::array<double, 3> & position)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = frame_id;
  object.id = id;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {size[0], size[1], size[2]};

  geometry_msgs::msg::Pose pose;
  pose.position.x = position[0];
  pose.position.y = position[1];
  pose.position.z = position[2];
  pose.orientation.w = 1.0;

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;

  return object;
}

moveit_msgs::msg::CollisionObject makeTable(
  const std::string & id,
  const std::string & frame_id,
  double x,
  double y,
  double depth = 0.22,
  double width = 0.30)
{
  moveit_msgs::msg::CollisionObject table;
  table.header.frame_id = frame_id;
  table.id = id;

  // Tabletop
  shape_msgs::msg::SolidPrimitive tabletop;
  tabletop.type = shape_msgs::msg::SolidPrimitive::BOX;
  // Compact tabletops leave visible corridors for the blocking walls.
  tabletop.dimensions = {width, depth, 0.05};

  geometry_msgs::msg::Pose tabletop_pose;
  tabletop_pose.position.x = x;
  tabletop_pose.position.y = y;
  tabletop_pose.position.z = 0.30;
  tabletop_pose.orientation.w = 1.0;

  // Central support
  shape_msgs::msg::SolidPrimitive support;
  support.type = shape_msgs::msg::SolidPrimitive::BOX;
  support.dimensions = {0.12, 0.12, 0.275};

  geometry_msgs::msg::Pose support_pose;
  support_pose.position.x = x;
  support_pose.position.y = y;
  support_pose.position.z = 0.1375;
  support_pose.orientation.w = 1.0;

  table.primitives.push_back(tabletop);
  table.primitive_poses.push_back(tabletop_pose);

  table.primitives.push_back(support);
  table.primitive_poses.push_back(support_pose);

  table.operation = moveit_msgs::msg::CollisionObject::ADD;

  return table;
}

moveit_msgs::msg::ObjectColor makeColor(
  const std::string & id,
  double red,
  double green,
  double blue)
{
  moveit_msgs::msg::ObjectColor color;
  color.id = id;
  color.color.r = red;
  color.color.g = green;
  color.color.b = blue;
  color.color.a = 1.0;

  return color;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("hard_scene_builder");

  moveit::planning_interface::PlanningSceneInterface planning_scene;

  const std::string frame_id = "world";

  // Remove old versions when the program is run again.
  const std::vector<std::string> object_ids = {
    "pick_table",
    "place_table",
    "table_1",
    "table_2",
    "table_3",
    "table_4",
    "ground",
    "obstacle_gap_12",
    "center_obstacle",
    "obstacle_gap_34",
    "obstacle_outer_north",
    "obstacle_outer_south",
    "barrier_12",
    "barrier_23",
    "barrier_34",
    "gate_north",
    "gate_south",
    "table_1_screen",
    "table_2_screen",
    "table_3_screen",
    "table_4_screen",
    "shelf_middle_roof",
    "shelf_middle_north",
    "shelf_middle_divider",
    "shelf_middle_south",
    "shelf_table_1_roof",
    "shelf_table_1_side",
    "shelf_table_4_roof",
    "shelf_table_4_side",
    "table_1_frame_roof",
    "table_1_frame_left",
    "table_1_frame_divider",
    "table_1_frame_right",
    "table_2_frame_roof",
    "table_2_frame_left",
    "table_2_frame_divider",
    "table_2_frame_right",
    "table_3_frame_roof",
    "table_3_frame_left",
    "table_3_frame_divider",
    "table_3_frame_right",
    "table_4_frame_roof",
    "table_4_frame_left",
    "table_4_frame_divider",
    "table_4_frame_right",
    "cube_1",
    "cube_2",
    "cube_3"
  };

  planning_scene.removeCollisionObjects(object_ids);
  std::this_thread::sleep_for(1s);

  // The mock robot starts inside the future Table 3 frame. Reposition in the
  // cleared scene before inserting it, so the new scene has a valid start.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});
  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
  move_group.setEndEffectorLink("ee_link");
  move_group.setPlanningTime(15.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setMaxVelocityScalingFactor(0.15);
  move_group.setMaxAccelerationScalingFactor(0.15);

  geometry_msgs::msg::Pose staging_pose;
  staging_pose.orientation.x = -1.0;
  staging_pose.position.x = 0.00;
  staging_pose.position.y = 0.00;
  staging_pose.position.z = 0.70;
  bool staged = false;
  for (int attempt = 0; attempt < 8 && !staged; ++attempt)
  {
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(staging_pose, "ee_link");
    moveit::planning_interface::MoveGroupInterface::Plan staging_plan;
    const bool planned =
      move_group.plan(staging_plan) == moveit::core::MoveItErrorCode::SUCCESS;
    move_group.clearPoseTargets();
    staged = planned &&
      move_group.execute(staging_plan) == moveit::core::MoveItErrorCode::SUCCESS;
    if (!staged)
    {
      std::this_thread::sleep_for(200ms);
    }
  }
  if (!staged)
  {
    RCLCPP_ERROR(node->get_logger(), "Could not reach staging pose before adding frames.");
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Staged robot in clear neutral area before adding frames.");

  std::vector<moveit_msgs::msg::CollisionObject> objects;
  std::vector<moveit_msgs::msg::ObjectColor> colors;

  // Ground plane: prevents plans from sending robot links below the workspace.
  objects.push_back(
    makeBox("ground", frame_id, {2.40, 2.40, 0.05}, {0.20, 0.00, -0.055}));
  colors.push_back(makeColor("ground", 0.20, 0.22, 0.24));

  // Four tables arranged around the front half of the UR5 workspace.
  objects.push_back(makeTable("table_1", frame_id, 0.30, 0.55));
  colors.push_back(makeColor("table_1", 0.55, 0.27, 0.08));

  objects.push_back(makeTable("table_2", frame_id, 0.50, 0.25, 0.20, 0.38));
  colors.push_back(makeColor("table_2", 0.45, 0.30, 0.12));

  objects.push_back(makeTable("table_3", frame_id, 0.50, -0.25, 0.20, 0.38));
  colors.push_back(makeColor("table_3", 0.45, 0.30, 0.12));

  objects.push_back(makeTable("table_4", frame_id, 0.30, -0.55));
  colors.push_back(makeColor("table_4", 0.55, 0.27, 0.08));

  // Barriers 12 and 34 cut the outer transfers. Barrier 23 blocks the
  // outside bypass, while the two framed tables obstruct the middle transfer.
  // The gate pillars further narrow the routes around the tables.
  objects.push_back(
    makeBox(
      "barrier_12",
      frame_id,
      {0.16, 0.08, 0.60},
      {0.34, 0.375, 0.30}));
  colors.push_back(makeColor("barrier_12", 0.70, 0.12, 0.10));

  objects.push_back(
    makeBox(
      "barrier_23",
      frame_id,
      {0.10, 0.08, 0.46},
      {0.77, 0.00, 0.23}));
  colors.push_back(makeColor("barrier_23", 0.72, 0.14, 0.10));

  objects.push_back(
    makeBox(
      "barrier_34",
      frame_id,
      {0.16, 0.08, 0.60},
      {0.34, -0.375, 0.30}));
  colors.push_back(makeColor("barrier_34", 0.70, 0.12, 0.10));

  objects.push_back(
    makeBox(
      "gate_north",
      frame_id,
      {0.07, 0.07, 0.60},
      {0.68, 0.38, 0.30}));
  colors.push_back(makeColor("gate_north", 0.92, 0.45, 0.08));

  objects.push_back(
    makeBox(
      "gate_south",
      frame_id,
      {0.07, 0.07, 0.60},
      {0.68, -0.38, 0.30}));
  colors.push_back(makeColor("gate_south", 0.92, 0.45, 0.08));

  // Open-front frames stand on Tables 2 and 3. Their roofs and side posts
  // form a central bay for the cube; all pieces are collision objects.
  struct TableFrame
  {
    std::string name;
    double x;
    double y;
  };

  const std::array<TableFrame, 2> frames = {{
    {"table_2_frame", 0.50, 0.30},
    {"table_3_frame", 0.50, -0.32}
  }};

  constexpr double frame_base_z = 0.325;
  constexpr double post_height = 0.70;
  constexpr double post_z = frame_base_z + post_height / 2.0;
  constexpr double roof_z = frame_base_z + post_height + 0.02;

  const bool include_frames = std::getenv("UR5_SKIP_TABLE_FRAMES") == nullptr;
  if (include_frames)
  {
    for (const auto & frame : frames)
    {
      const auto add_frame_piece = [&](const std::string & suffix,
        const std::array<double, 3> & size,
        const std::array<double, 3> & center)
      {
        const std::string id = frame.name + suffix;
        objects.push_back(makeBox(id, frame_id, size, center));
        colors.push_back(makeColor(id, 0.68, 0.73, 0.77));
      };

      add_frame_piece("_roof", {0.35, 0.14, 0.04},
        {frame.x, frame.y, roof_z});
      add_frame_piece("_left", {0.025, 0.035, post_height},
        {frame.x - 0.16, frame.y, post_z});
      add_frame_piece("_right", {0.025, 0.035, post_height},
        {frame.x + 0.16, frame.y, post_z});
    }
  }

  // Three cubes on table 1
  objects.push_back(
    makeBox("cube_1", frame_id, {0.05, 0.05, 0.05}, {0.24, 0.55, 0.35}));
  colors.push_back(makeColor("cube_1", 0.90, 0.10, 0.10));

  objects.push_back(
    makeBox("cube_2", frame_id, {0.05, 0.05, 0.05}, {0.30, 0.55, 0.35}));
  colors.push_back(makeColor("cube_2", 0.10, 0.80, 0.20));

  objects.push_back(
    makeBox("cube_3", frame_id, {0.05, 0.05, 0.05}, {0.36, 0.55, 0.35}));
  colors.push_back(makeColor("cube_3", 0.10, 0.30, 0.90));

  const bool success =
    planning_scene.applyCollisionObjects(objects, colors);

  if (success)
  {
    RCLCPP_INFO(
      node->get_logger(),
      "Hard scene created: 4 tables, 3 blocking walls, 2 gate pillars, "
      "%d centered tabletop frames and 3 cubes.",
      include_frames ? 2 : 0);
  }
  else
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to create the planning scene.");
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return success ? 0 : 1;
}
