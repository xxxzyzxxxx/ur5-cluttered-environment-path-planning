#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;

namespace rvt = rviz_visual_tools;

using GripperCommand = control_msgs::action::GripperCommand;

void publishStatus(
  moveit_visual_tools::MoveItVisualTools& visual_tools,
  const std::string& text,
  rvt::Colors color = rvt::WHITE)
{
  geometry_msgs::msg::Pose text_pose;
  text_pose.orientation.w = 1.0;
  text_pose.position.x = 0.30;
  text_pose.position.y = 0.0;
  text_pose.position.z = 1.20;

  visual_tools.publishText(
    text_pose,
    text,
    color,
    rvt::MEDIUM,
    true);

  visual_tools.trigger();
}

bool commandGripper(
  const rclcpp::Node::SharedPtr& node,
  const rclcpp_action::Client<GripperCommand>::SharedPtr& client,
  double position)
{
  if (!client->wait_for_action_server(5s))
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Gripper action server is not available.");
    return false;
  }

  GripperCommand::Goal goal;
  goal.command.position = position;
  goal.command.max_effort = 20.0;

  RCLCPP_INFO(
    node->get_logger(),
    "Sending gripper command: position = %.2f",
    position);

  auto goal_future = client->async_send_goal(goal);

  if (goal_future.wait_for(5s) != std::future_status::ready)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Timed out while sending gripper goal.");
    return false;
  }

  auto goal_handle = goal_future.get();

  if (!goal_handle)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "The gripper rejected the goal.");
    return false;
  }

  auto result_future = client->async_get_result(goal_handle);

  if (result_future.wait_for(10s) != std::future_status::ready)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Timed out while waiting for gripper result.");
    return false;
  }

  const auto wrapped_result = result_future.get();

  const bool success =
    wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
    wrapped_result.result->reached_goal;

  RCLCPP_INFO(
    node->get_logger(),
    "Gripper result: %s",
    success ? "SUCCESS" : "FAILED");

  return success;
}

bool moveToPose(
  const rclcpp::Node::SharedPtr& node,
  moveit::planning_interface::MoveGroupInterface& move_group,
  moveit_visual_tools::MoveItVisualTools& visual_tools,
  const moveit::core::JointModelGroup* joint_model_group,
  const geometry_msgs::msg::Pose& target,
  const std::string& step_name)
{
  publishStatus(
    visual_tools,
    step_name,
    rvt::WHITE);

  RCLCPP_INFO(
    node->get_logger(),
    "%s target: x=%.3f, y=%.3f, z=%.3f",
    step_name.c_str(),
    target.position.x,
    target.position.y,
    target.position.z);

  constexpr int max_retries = 8;

  for (int attempt = 1; attempt <= max_retries; ++attempt)
  {
    RCLCPP_INFO(
      node->get_logger(),
      "%s planning attempt %d/%d",
      step_name.c_str(),
      attempt,
      max_retries);

    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(target, "ee_link");

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    const bool planned =
      move_group.plan(plan) ==
      moveit::core::MoveItErrorCode::SUCCESS;

    move_group.clearPoseTargets();

    if (!planned)
    {
      RCLCPP_WARN(
        node->get_logger(),
        "%s attempt %d failed. Replanning...",
        step_name.c_str(),
        attempt);

      std::this_thread::sleep_for(200ms);
      continue;
    }

    visual_tools.publishAxisLabeled(
      target,
      step_name,
      rvt::SMALL);

    const auto* end_effector_link = move_group.getRobotModel()->getLinkModel("ee_link");

    if (end_effector_link)
    {
      visual_tools.publishTrajectoryLine(
      plan.trajectory,
      end_effector_link,
      joint_model_group,
      rvt::LIME_GREEN);
    }

    visual_tools.trigger();

    const bool executed =
      move_group.execute(plan) ==
      moveit::core::MoveItErrorCode::SUCCESS;

    if (executed)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "%s: SUCCESS",
        step_name.c_str());

      return true;
    }

    RCLCPP_WARN(
      node->get_logger(),
      "%s execution attempt %d failed.",
      step_name.c_str(),
      attempt);

    std::this_thread::sleep_for(200ms);
  }

  publishStatus(
    visual_tools,
    step_name + " FAILED",
    rvt::RED);

  RCLCPP_ERROR(
    node->get_logger(),
    "%s failed after %d attempts.",
    step_name.c_str(),
    max_retries);

  return false;
}

bool moveCartesian(
  const rclcpp::Node::SharedPtr& node,
  moveit::planning_interface::MoveGroupInterface& move_group,
  moveit_visual_tools::MoveItVisualTools& visual_tools,
  const geometry_msgs::msg::Pose& target,
  const std::string& step_name)
{
  publishStatus(
    visual_tools,
    step_name,
    rvt::YELLOW);

  const auto current_pose =
    move_group.getCurrentPose("ee_link").pose;

  std::vector<geometry_msgs::msg::Pose> waypoints = {
    current_pose,
    target
  };

  moveit_msgs::msg::RobotTrajectory trajectory;

  const double fraction = move_group.computeCartesianPath(
    waypoints,
    0.005,
    trajectory,
    false);

  RCLCPP_INFO(
    node->get_logger(),
    "%s Cartesian path: %.1f%%",
    step_name.c_str(),
    fraction * 100.0);

  visual_tools.publishPath(
    waypoints,
    rvt::YELLOW,
    rvt::SMALL,
    step_name);

  visual_tools.publishAxisLabeled(
    target,
    step_name,
    rvt::SMALL);

  visual_tools.trigger();

  if (fraction < 0.95)
  {
    publishStatus(
      visual_tools,
      step_name + " FAILED",
      rvt::RED);

    RCLCPP_ERROR(
      node->get_logger(),
      "%s Cartesian path is incomplete.",
      step_name.c_str());

    return false;
  }

  const bool executed =
    move_group.execute(trajectory) ==
    moveit::core::MoveItErrorCode::SUCCESS;

  RCLCPP_INFO(
    node->get_logger(),
    "%s: %s",
    step_name.c_str(),
    executed ? "SUCCESS" : "FAILED");

  return executed;
}

bool transferCube(
  const rclcpp::Node::SharedPtr& node,
  moveit::planning_interface::MoveGroupInterface& move_group,
  moveit_visual_tools::MoveItVisualTools& visual_tools,
  const moveit::core::JointModelGroup* joint_model_group,
  const rclcpp_action::Client<GripperCommand>::SharedPtr& gripper_client,
  const std::string& cube_id,
  double pick_x,
  double place_x)
{
  const double pick_y = 0.40;
  const double place_y = -0.40;

  const double approach_z =
    cube_id == "cube_3" ? 0.40 : 0.45;

  const double grasp_z = 0.35;

  geometry_msgs::msg::Pose target_pose;

  target_pose.orientation.x = -1.0;
  target_pose.orientation.y = 0.0;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 0.0;

  publishStatus(
    visual_tools,
    "Picking " + cube_id,
    rvt::YELLOW);

  RCLCPP_INFO(
    node->get_logger(),
    "========================================");

  RCLCPP_INFO(
    node->get_logger(),
    "Starting transfer of %s",
    cube_id.c_str());

  RCLCPP_INFO(
    node->get_logger(),
    "========================================");

  // Open gripper.
  if (!commandGripper(
      node,
      gripper_client,
      0.1))
  {
    return false;
  }

  // Move above cube.
  target_pose.position.x = pick_x;
  target_pose.position.y = pick_y;
  target_pose.position.z = approach_z;

  if (!moveToPose(
      node,
      move_group,
      visual_tools,
      joint_model_group,
      target_pose,
      "Move above " + cube_id))
  {
    return false;
  }

  // Descend to cube.
  target_pose.position.z = grasp_z;

  if (!moveCartesian(
      node,
      move_group,
      visual_tools,
      target_pose,
      "Descend to " + cube_id))
  {
    return false;
  }

  // Close gripper.
  if (!commandGripper(
      node,
      gripper_client,
      0.5))
  {
    return false;
  }

  // Attach cube.
  const std::vector<std::string> touch_links = {
    "ee_link",
    "robotiq_85_base_link",
    "robotiq_85_left_knuckle_link",
    "robotiq_85_right_knuckle_link",
    "robotiq_85_left_finger_link",
    "robotiq_85_right_finger_link",
    "robotiq_85_left_inner_knuckle_link",
    "robotiq_85_right_inner_knuckle_link",
    "robotiq_85_left_finger_tip_link",
    "robotiq_85_right_finger_tip_link"
  };

  const bool attached = move_group.attachObject(
    cube_id,
    "ee_link",
    touch_links);

  RCLCPP_INFO(
    node->get_logger(),
    "Attach %s: %s",
    cube_id.c_str(),
    attached ? "SUCCESS" : "FAILED");

  if (!attached)
  {
    return false;
  }

  publishStatus(
    visual_tools,
    cube_id + " attached",
    rvt::YELLOW);

  std::this_thread::sleep_for(1s);

  // Lift cube.
  target_pose.position.z = approach_z;

  if (!moveCartesian(
      node,
      move_group,
      visual_tools,
      target_pose,
      "Lift " + cube_id))
  {
    return false;
  }

  publishStatus(
    visual_tools,
    "Placing " + cube_id,
    rvt::WHITE);

  // Move above place table.
  target_pose.position.x = place_x;
  target_pose.position.y = place_y;
  target_pose.position.z = approach_z;

  if (!moveToPose(
      node,
      move_group,
      visual_tools,
      joint_model_group,
      target_pose,
      "Move " + cube_id + " above place table"))
  {
    return false;
  }

  // Lower cube.
  target_pose.position.z = grasp_z;

  if (!moveCartesian(
      node,
      move_group,
      visual_tools,
      target_pose,
      "Lower " + cube_id))
  {
    return false;
  }

  // Detach cube.
  const bool detached =
    move_group.detachObject(cube_id);

  RCLCPP_INFO(
    node->get_logger(),
    "Detach %s: %s",
    cube_id.c_str(),
    detached ? "SUCCESS" : "FAILED");

  if (!detached)
  {
    return false;
  }

  std::this_thread::sleep_for(1s);

  // Open gripper.
  if (!commandGripper(
      node,
      gripper_client,
      0.1))
  {
    return false;
  }

  // Retreat.
  target_pose.position.z = approach_z;

  if (!moveCartesian(
      node,
      move_group,
      visual_tools,
      target_pose,
      "Retreat from " + cube_id))
  {
    return false;
  }

  publishStatus(
    visual_tools,
    cube_id + " completed",
    rvt::GREEN);

  RCLCPP_INFO(
    node->get_logger(),
    "%s transfer: SUCCESS",
    cube_id.c_str());

  return true;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "three_cube_pick_place",
    rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  moveit::planning_interface::MoveGroupInterface move_group(
    node,
    "ur_manipulator");

  moveit::planning_interface::PlanningSceneInterface
    planning_scene_interface;

  auto gripper_client =
    rclcpp_action::create_client<GripperCommand>(
      node,
      "/gripper_controller/gripper_cmd");

  move_group.setEndEffectorLink("ee_link");
  move_group.setPlanningTime(15.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setMaxVelocityScalingFactor(0.15);
  move_group.setMaxAccelerationScalingFactor(0.15);

  const auto robot_model =
    move_group.getRobotModel();

  const auto* joint_model_group =
    robot_model->getJointModelGroup("ur_manipulator");

  if (!joint_model_group)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Joint model group ur_manipulator was not found.");

    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  moveit_visual_tools::MoveItVisualTools visual_tools(
    node,
    "world",
    "/ur5_three_cube_markers",
    robot_model);

  visual_tools.deleteAllMarkers();

  RCLCPP_INFO(
    node->get_logger(),
    "Planning frame: %s",
    move_group.getPlanningFrame().c_str());

  RCLCPP_INFO(
    node->get_logger(),
    "End-effector link: %s",
    move_group.getEndEffectorLink().c_str());

  struct CubeTask
  {
    std::string id;
    double pick_x;
    double place_x;
  };

  const std::vector<CubeTask> tasks = {
    {"cube_1", 0.42, 0.42},
    {"cube_2", 0.50, 0.50},
    {"cube_3", 0.56, 0.56}
  };

  // Publish title and all pick/place coordinate markers.
  geometry_msgs::msg::Pose text_pose;
  text_pose.orientation.w = 1.0;
  text_pose.position.x = 0.30;
  text_pose.position.y = 0.0;
  text_pose.position.z = 1.20;

  visual_tools.publishText(
    text_pose,
    "UR5 Three-Cube Pick and Place",
    rvt::WHITE,
    rvt::MEDIUM,
    true);

  for (const auto& task : tasks)
  {
    geometry_msgs::msg::Pose pick_pose;
    pick_pose.orientation.x = -1.0;
    pick_pose.orientation.w = 0.0;
    pick_pose.position.x = task.pick_x;
    pick_pose.position.y = 0.40;
    pick_pose.position.z = 0.35;

    geometry_msgs::msg::Pose place_pose = pick_pose;
    place_pose.position.x = task.place_x;
    place_pose.position.y = -0.40;

    visual_tools.publishAxisLabeled(
      pick_pose,
      task.id + "_pick",
      rvt::SMALL);

    visual_tools.publishAxisLabeled(
      place_pose,
      task.id + "_place",
      rvt::SMALL);
  }

  visual_tools.trigger();

  const auto known_objects =
    planning_scene_interface.getKnownObjectNames();

  for (const auto& task : tasks)
  {
    if (
      std::find(
        known_objects.begin(),
        known_objects.end(),
        task.id) == known_objects.end())
    {
      publishStatus(
        visual_tools,
        task.id + " not found",
        rvt::RED);

      RCLCPP_ERROR(
        node->get_logger(),
        "%s was not found. Run scene_builder first.",
        task.id.c_str());

      rclcpp::shutdown();
      spinner.join();
      return 1;
    }
  }

  bool all_success = true;

  for (std::size_t index = 0; index < tasks.size(); ++index)
  {
    const auto& task = tasks[index];

    RCLCPP_INFO(
      node->get_logger(),
      "Processing cube %zu of %zu",
      index + 1,
      tasks.size());

    const bool success = transferCube(
      node,
      move_group,
      visual_tools,
      joint_model_group,
      gripper_client,
      task.id,
      task.pick_x,
      task.place_x);

    if (!success)
    {
      publishStatus(
        visual_tools,
        task.id + " transfer FAILED",
        rvt::RED);

      RCLCPP_ERROR(
        node->get_logger(),
        "%s transfer failed. Stopping the sequence.",
        task.id.c_str());

      all_success = false;
      break;
    }
  }

  publishStatus(
    visual_tools,
    all_success
      ? "THREE-CUBE PICK AND PLACE: SUCCESS"
      : "THREE-CUBE PICK AND PLACE: FAILED",
    all_success ? rvt::GREEN : rvt::RED);

  RCLCPP_INFO(
    node->get_logger(),
    "========================================");

  RCLCPP_INFO(
    node->get_logger(),
    "THREE-CUBE PICK AND PLACE: %s",
    all_success ? "SUCCESS" : "FAILED");

  RCLCPP_INFO(
    node->get_logger(),
    "========================================");

  // Keep the final markers visible briefly.
  std::this_thread::sleep_for(3s);

  rclcpp::shutdown();
  spinner.join();

  return all_success ? 0 : 1;
}