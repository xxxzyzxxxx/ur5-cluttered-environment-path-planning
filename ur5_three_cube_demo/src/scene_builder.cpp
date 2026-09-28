#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
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
  double y)
{
  moveit_msgs::msg::CollisionObject table;
  table.header.frame_id = frame_id;
  table.id = id;

  // Tabletop
  shape_msgs::msg::SolidPrimitive tabletop;
  tabletop.type = shape_msgs::msg::SolidPrimitive::BOX;
  tabletop.dimensions = {0.55, 0.42, 0.05};

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

  auto node = std::make_shared<rclcpp::Node>("scene_builder");

  moveit::planning_interface::PlanningSceneInterface planning_scene;

  const std::string frame_id = "world";

  // Remove old versions when the program is run again.
  const std::vector<std::string> object_ids = {
    "pick_table",
    "place_table",
    "center_obstacle",
    "cube_1",
    "cube_2",
    "cube_3"
  };

  planning_scene.removeCollisionObjects(object_ids);
  std::this_thread::sleep_for(1s);

  std::vector<moveit_msgs::msg::CollisionObject> objects;
  std::vector<moveit_msgs::msg::ObjectColor> colors;

  // Two tables
  objects.push_back(makeTable("pick_table", frame_id, 0.50, 0.40));
  colors.push_back(makeColor("pick_table", 0.55, 0.27, 0.08));

  objects.push_back(makeTable("place_table", frame_id, 0.50, -0.40));
  colors.push_back(makeColor("place_table", 0.55, 0.27, 0.08));

  // Obstacle between the two tables
  objects.push_back(
    makeBox(
      "center_obstacle",
      frame_id,
      {0.10, 0.16, 0.55},
      {0.78, 0.00, 0.275}));

  colors.push_back(makeColor("center_obstacle", 0.25, 0.25, 0.28));

  // Three cubes on the pick table
  objects.push_back(
    makeBox("cube_1", frame_id, {0.05, 0.05, 0.05}, {0.42, 0.40, 0.35}));
  colors.push_back(makeColor("cube_1", 0.90, 0.10, 0.10));

  objects.push_back(
    makeBox("cube_2", frame_id, {0.05, 0.05, 0.05}, {0.50, 0.40, 0.35}));
  colors.push_back(makeColor("cube_2", 0.10, 0.80, 0.20));

  objects.push_back(
    makeBox("cube_3", frame_id, {0.05, 0.05, 0.05}, {0.56, 0.40, 0.35}));
  colors.push_back(makeColor("cube_3", 0.10, 0.30, 0.90));

  const bool success =
    planning_scene.applyCollisionObjects(objects, colors);

  if (success)
  {
    RCLCPP_INFO(
      node->get_logger(),
      "Scene created successfully: 2 tables, 1 obstacle and 3 cubes.");
  }
  else
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to create the planning scene.");
  }

  rclcpp::shutdown();
  return success ? 0 : 1;
}