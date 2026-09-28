#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;

namespace rvt = rviz_visual_tools;

using GripperCommand = control_msgs::action::GripperCommand;
using GetPositionIK = moveit_msgs::srv::GetPositionIK;

struct RoutePoint
{
  double x;
  double y;
  std::string label;
};

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
  const std::string& step_name,
  bool avoid_collisions = false)
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
    avoid_collisions);

  RCLCPP_INFO(
    node->get_logger(),
    "%s Cartesian path: %.1f%% (collision checking: %s)",
    step_name.c_str(),
    fraction * 100.0,
    avoid_collisions ? "ON" : "OFF");

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

  if (fraction < 0.995)
  {
    if (avoid_collisions)
    {
      moveit_msgs::msg::RobotTrajectory diagnostic_trajectory;
      const double no_collision_fraction = move_group.computeCartesianPath(
        waypoints,
        0.005,
        diagnostic_trajectory,
        false);

      RCLCPP_WARN(
        node->get_logger(),
        "%s diagnostic without collision checking: %.1f%%",
        step_name.c_str(),
        no_collision_fraction * 100.0);
    }

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

bool moveToPoseWithClosestIK(
  const rclcpp::Node::SharedPtr& node,
  moveit::planning_interface::MoveGroupInterface& move_group,
  moveit_visual_tools::MoveItVisualTools& visual_tools,
  const moveit::core::JointModelGroup* joint_model_group,
  const rclcpp::Client<GetPositionIK>::SharedPtr& ik_client,
  const geometry_msgs::msg::Pose& target,
  const std::string& step_name)
{
  publishStatus(visual_tools, step_name, rvt::WHITE);

  if (!ik_client->wait_for_service(5s))
  {
    RCLCPP_ERROR(node->get_logger(), "MoveIt IK service is not available.");
    return false;
  }

  constexpr int max_retries = 48;

  for (int attempt = 1; attempt <= max_retries; ++attempt)
  {
    const auto current_state = move_group.getCurrentState(2.0);

    if (!current_state)
    {
      RCLCPP_WARN(node->get_logger(), "%s: current state unavailable.", step_name.c_str());
      continue;
    }

    auto request = std::make_shared<GetPositionIK::Request>();
    request->ik_request.group_name = "ur_manipulator";
    request->ik_request.ik_link_name = "ee_link";
    request->ik_request.pose_stamped.header.frame_id = move_group.getPlanningFrame();
    request->ik_request.pose_stamped.header.stamp = node->now();
    request->ik_request.pose_stamped.pose = target;
    request->ik_request.avoid_collisions = true;
    request->ik_request.timeout.sec = 1;
    request->ik_request.timeout.nanosec = 0;
    request->ik_request.robot_state.is_diff = true;

    // The same nearby seed returns the same failed IK branch on every retry.
    // Sample alternative branches while always planning from the real state.
    moveit::core::RobotState ik_seed(*current_state);
    if (attempt > 1)
    {
      ik_seed.setToRandomPositions(joint_model_group);
      ik_seed.update();
    }
    // Seed IK with arm variables only; the planning scene retains its
    // current gripper and attached-cube state.
    auto& seed_joints = request->ik_request.robot_state.joint_state;
    seed_joints.name = joint_model_group->getVariableNames();
    seed_joints.position.reserve(seed_joints.name.size());
    for (const auto& variable_name : seed_joints.name)
    {
      seed_joints.position.push_back(ik_seed.getVariablePosition(variable_name));
    }
    request->ik_request.robot_state.is_diff = true;

    RCLCPP_INFO(
      node->get_logger(),
      "%s nearest-IK attempt %d/%d",
      step_name.c_str(),
      attempt,
      max_retries);

    auto future = ik_client->async_send_request(request);

    if (future.wait_for(3s) != std::future_status::ready)
    {
      RCLCPP_WARN(node->get_logger(), "%s: IK service timed out.", step_name.c_str());
      continue;
    }

    const auto response = future.get();

    if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_WARN(
        node->get_logger(),
        "%s: IK failed with code %d.",
        step_name.c_str(),
        response->error_code.val);
      continue;
    }

    move_group.setStartStateToCurrentState();
    move_group.clearPoseTargets();

    std::vector<double> group_target;
    group_target.reserve(joint_model_group->getVariableCount());

    bool complete_group_target = true;
    for (const auto& variable_name : joint_model_group->getVariableNames())
    {
      const auto joint_it = std::find(
        response->solution.joint_state.name.begin(),
        response->solution.joint_state.name.end(),
        variable_name);

      if (joint_it == response->solution.joint_state.name.end())
      {
        complete_group_target = false;
        break;
      }

      const auto joint_index = std::distance(
        response->solution.joint_state.name.begin(),
        joint_it);
      group_target.push_back(response->solution.joint_state.position[joint_index]);
    }

    std::vector<double> current_group_values;
    current_state->copyJointGroupPositions(
      joint_model_group,
      current_group_values);

    double largest_joint_change = 0.0;
    const auto& variable_names = joint_model_group->getVariableNames();

    for (std::size_t index = 0; index < group_target.size(); ++index)
    {
      const double raw_target = group_target[index];
      double nearest_target = raw_target;
      double nearest_distance = std::abs(raw_target - current_group_values[index]);
      const auto& bounds = move_group.getRobotModel()->getVariableBounds(variable_names[index]);

      // UR joints can represent the same angle one full turn apart. Choose the
      // equivalent value nearest the current state so the arm never spins 2*pi.
      for (int turns = -2; turns <= 2; ++turns)
      {
        const double candidate =
          raw_target + static_cast<double>(turns) * 2.0 * M_PI;

        if (
          candidate < bounds.min_position_ - 1e-6 ||
          candidate > bounds.max_position_ + 1e-6)
        {
          continue;
        }

        const double distance =
          std::abs(candidate - current_group_values[index]);

        if (distance < nearest_distance)
        {
          nearest_target = candidate;
          nearest_distance = distance;
        }
      }

      group_target[index] = nearest_target;
      largest_joint_change = std::max(largest_joint_change, nearest_distance);
    }

    RCLCPP_INFO(
      node->get_logger(),
      "%s largest nearest-IK joint change: %.3f rad",
      step_name.c_str(),
      largest_joint_change);

    if (!complete_group_target || !move_group.setJointValueTarget(group_target))
    {
      RCLCPP_WARN(node->get_logger(), "%s: IK joint target is outside limits.", step_name.c_str());
      continue;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned =
      move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;

    if (!planned)
    {
      RCLCPP_WARN(node->get_logger(), "%s: joint-space planning failed.", step_name.c_str());
      continue;
    }

    // Reject any OMPL result that sends the carried cube below the table-safe
    // workspace or rotates a joint through an unnecessary large detour.
    moveit::core::RobotState trajectory_state(*current_state);
    double minimum_ee_height = std::numeric_limits<double>::infinity();
    std::vector<double> joint_path_motion(
      plan.trajectory.joint_trajectory.joint_names.size(),
      0.0);
    std::vector<double> previous_positions;

    for (const auto& point : plan.trajectory.joint_trajectory.points)
    {
      trajectory_state.setVariablePositions(
        plan.trajectory.joint_trajectory.joint_names,
        point.positions);
      trajectory_state.updateLinkTransforms();

      minimum_ee_height = std::min(
        minimum_ee_height,
        trajectory_state.getGlobalLinkTransform("ee_link").translation().z());

      if (!previous_positions.empty())
      {
        for (std::size_t joint = 0; joint < point.positions.size(); ++joint)
        {
          joint_path_motion[joint] +=
            std::abs(point.positions[joint] - previous_positions[joint]);
        }
      }

      previous_positions = point.positions;
    }

    const double largest_path_motion = joint_path_motion.empty()
      ? 0.0
      : *std::max_element(joint_path_motion.begin(), joint_path_motion.end());

    RCLCPP_INFO(
      node->get_logger(),
      "%s trajectory check: minimum ee z=%.3f m, largest joint path=%.3f rad",
      step_name.c_str(),
      minimum_ee_height,
      largest_path_motion);

    if (minimum_ee_height < 0.42 || largest_path_motion > M_PI + 0.15)
    {
      RCLCPP_WARN(
        node->get_logger(),
        "%s: rejected unnatural trajectory; replanning.",
        step_name.c_str());
      continue;
    }

    const auto* end_effector_link = move_group.getRobotModel()->getLinkModel("ee_link");
    if (end_effector_link)
    {
      visual_tools.publishTrajectoryLine(
        plan.trajectory,
        end_effector_link,
        joint_model_group,
        rvt::LIME_GREEN);
    }
    visual_tools.publishAxisLabeled(target, step_name, rvt::SMALL);
    visual_tools.trigger();

    const bool executed =
      move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;

    if (executed)
    {
      RCLCPP_INFO(node->get_logger(), "%s: SUCCESS", step_name.c_str());
      return true;
    }

    RCLCPP_WARN(node->get_logger(), "%s: execution failed.", step_name.c_str());
  }

  publishStatus(visual_tools, step_name + " FAILED", rvt::RED);
  RCLCPP_ERROR(node->get_logger(), "%s failed after nearest-IK retries.", step_name.c_str());
  return false;
}

bool transferCube(
  const rclcpp::Node::SharedPtr& node,
  moveit::planning_interface::MoveGroupInterface& move_group,
  moveit_visual_tools::MoveItVisualTools& visual_tools,
  const moveit::core::JointModelGroup* joint_model_group,
  const rclcpp_action::Client<GripperCommand>::SharedPtr& gripper_client,
  const rclcpp::Client<GetPositionIK>::SharedPtr& ik_client,
  const std::string& cube_id,
  double pick_x,
  double pick_y,
  double place_x,
  double place_y,
  const std::string& source_table,
  const std::string& destination_table,
  const std::vector<RoutePoint>& route_points)
{
  const double approach_z = 0.48;
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
    "Starting transfer of %s: %s -> %s",
    cube_id.c_str(),
    source_table.c_str(),
    destination_table.c_str());

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
      "Move above " + source_table))
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
      "Descend on " + source_table))
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

  // For routed transfers, measure the original source-to-destination straight
  // path before taking the safe corridor. This demonstrates that the obstacle
  // changes the planning problem instead of merely decorating the scene.
  if (!route_points.empty())
  {
    geometry_msgs::msg::Pose direct_destination = target_pose;
    direct_destination.position.x = place_x;
    direct_destination.position.y = place_y;
    direct_destination.position.z = approach_z;

    const std::vector<geometry_msgs::msg::Pose> full_direct_waypoints = {
      move_group.getCurrentPose("ee_link").pose,
      direct_destination
    };
    moveit_msgs::msg::RobotTrajectory checked_full_direct;
    moveit_msgs::msg::RobotTrajectory geometric_full_direct;

    const double checked_full_fraction = move_group.computeCartesianPath(
      full_direct_waypoints,
      0.005,
      checked_full_direct,
      true);
    const double geometric_full_fraction = move_group.computeCartesianPath(
      full_direct_waypoints,
      0.005,
      geometric_full_direct,
      false);

    RCLCPP_INFO(
      node->get_logger(),
      "%s to %s full direct-path test: collision-aware %.1f%%, without obstacles %.1f%%",
      source_table.c_str(),
      destination_table.c_str(),
      checked_full_fraction * 100.0,
      geometric_full_fraction * 100.0);

    if (geometric_full_fraction >= 0.995 && checked_full_fraction < 0.995)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "%s to %s: direct route is blocked; using planned corridor.",
        source_table.c_str(),
        destination_table.c_str());
    }
  }

  const auto move_transport_leg =
    [&](double x, double y, const std::string& label)
    {
      target_pose.position.x = x;
      target_pose.position.y = y;
      target_pose.position.z = approach_z;

      const auto current_pose = move_group.getCurrentPose("ee_link").pose;
      const std::vector<geometry_msgs::msg::Pose> direct_waypoints = {
        current_pose,
        target_pose
      };
      moveit_msgs::msg::RobotTrajectory checked_direct_path;
      moveit_msgs::msg::RobotTrajectory geometric_direct_path;

      const double collision_aware_fraction = move_group.computeCartesianPath(
        direct_waypoints,
        0.005,
        checked_direct_path,
        true);
      const double geometric_fraction = move_group.computeCartesianPath(
        direct_waypoints,
        0.005,
        geometric_direct_path,
        false);

      RCLCPP_INFO(
        node->get_logger(),
        "%s direct-path test: collision-aware %.1f%%, without obstacles %.1f%%",
        label.c_str(),
        collision_aware_fraction * 100.0,
        geometric_fraction * 100.0);

      if (
        geometric_fraction >= 0.995 &&
        collision_aware_fraction < 0.995)
      {
        RCLCPP_INFO(
          node->get_logger(),
          "%s: blocking obstacle detected; invoking OMPL detour.",
          label.c_str());
      }

      if (collision_aware_fraction >= 0.995)
      {
        RCLCPP_INFO(
          node->get_logger(),
          "%s: verified collision-free corridor; executing Cartesian segment.",
          label.c_str());

        const bool executed =
          move_group.execute(checked_direct_path) ==
          moveit::core::MoveItErrorCode::SUCCESS;

        RCLCPP_INFO(
          node->get_logger(),
          "%s Cartesian corridor: %s",
          label.c_str(),
          executed ? "SUCCESS" : "FAILED");

        return executed;
      }

      return moveToPoseWithClosestIK(
        node,
        move_group,
        visual_tools,
        joint_model_group,
        ik_client,
        target_pose,
        label);
    };

  // Long transfers can use intermediate safe points above empty tables.
  for (const auto& route_point : route_points)
  {
    if (!move_transport_leg(
        route_point.x,
        route_point.y,
        "Move via " + route_point.label))
    {
      return false;
    }
  }

  // Move above the destination table.
  const bool transfer_succeeded = move_transport_leg(
    place_x,
    place_y,
    "Move to " + destination_table);

  if (!transfer_succeeded)
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
      "Lower on " + destination_table))
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
      "Retreat from " + destination_table))
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
    "four_table_cycle",
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

  auto ik_client =
    node->create_client<GetPositionIK>("/compute_ik");

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

  struct TransferStep
  {
    std::string source_table;
    std::string destination_table;
    double pick_x;
    double pick_y;
    double place_x;
    double place_y;
    std::vector<RoutePoint> route_points;
  };

  const std::string cube_id = "cube_1";

  // The cubes on Tables 2 and 3 sit within the central frame openings:
  // x=0.53 lies between the side posts (x=0.34 and x=0.66), and
  // y=+/-0.28 sits beneath each roof beside the posts at y=+0.30 and -0.32.
  const std::vector<TransferStep> steps = {
    {"Table 1", "Table 2", 0.24, 0.55, 0.53, 0.28,
      {{0.18, 0.20, "upper inner lane"},
       {0.18, 0.12, "front of Table 2 frame"},
       {0.53, 0.12, "Table 2 bay entrance"}}},
    {"Table 2", "Table 3", 0.53, 0.28, 0.53, -0.28,
      {{0.53, 0.12, "exit Table 2 bay"},
       {0.18, 0.12, "upper inner lane"},
       {0.18, -0.12, "lower inner lane"},
       {0.53, -0.12, "Table 3 bay entrance"}}},
    {"Table 3", "Table 4", 0.53, -0.28, 0.24, -0.55,
      {{0.53, -0.12, "exit Table 3 bay"},
       {0.18, -0.12, "front of Table 3 frame"},
       {0.18, -0.20, "lower inner lane"}}},
    {"Table 4", "Table 1", 0.24, -0.55, 0.24, 0.55,
      {{0.18, -0.20, "lower inner lane"},
       {0.18, 0.20, "upper inner lane"}}}
  };

  // Publish title and all pick/place coordinate markers.
  geometry_msgs::msg::Pose text_pose;
  text_pose.orientation.w = 1.0;
  text_pose.position.x = 0.30;
  text_pose.position.y = 0.0;
  text_pose.position.z = 1.20;

  visual_tools.publishText(
    text_pose,
    "UR5 Four-Table Cycle",
    rvt::WHITE,
    rvt::MEDIUM,
    true);

  for (const auto& step : steps)
  {
    geometry_msgs::msg::Pose pick_pose;
    pick_pose.orientation.x = -1.0;
    pick_pose.orientation.w = 0.0;
    pick_pose.position.x = step.pick_x;
    pick_pose.position.y = step.pick_y;
    pick_pose.position.z = 0.35;

    geometry_msgs::msg::Pose place_pose = pick_pose;
    place_pose.position.x = step.place_x;
    place_pose.position.y = step.place_y;

    visual_tools.publishAxisLabeled(
      pick_pose,
      step.source_table,
      rvt::SMALL);

    visual_tools.publishAxisLabeled(
      place_pose,
      step.destination_table,
      rvt::SMALL);
  }

  visual_tools.trigger();

  const auto known_objects =
    planning_scene_interface.getKnownObjectNames();

  if (
    std::find(
      known_objects.begin(),
      known_objects.end(),
      cube_id) == known_objects.end())
  {
    publishStatus(
      visual_tools,
      cube_id + " not found",
      rvt::RED);

    RCLCPP_ERROR(
      node->get_logger(),
      "%s was not found. Run hard_scene_builder first.",
      cube_id.c_str());

    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  bool all_success = true;

  for (std::size_t index = 0; index < steps.size(); ++index)
  {
    const auto& step = steps[index];

    RCLCPP_INFO(
      node->get_logger(),
      "Processing transfer %zu of %zu: %s -> %s",
      index + 1,
      steps.size(),
      step.source_table.c_str(),
      step.destination_table.c_str());

    const bool success = transferCube(
      node,
      move_group,
      visual_tools,
      joint_model_group,
      gripper_client,
      ik_client,
      cube_id,
      step.pick_x,
      step.pick_y,
      step.place_x,
      step.place_y,
      step.source_table,
      step.destination_table,
      step.route_points);

    if (!success)
    {
      publishStatus(
        visual_tools,
        step.source_table + " to " + step.destination_table + " FAILED",
        rvt::RED);

      RCLCPP_ERROR(
        node->get_logger(),
        "%s to %s failed. Stopping the sequence.",
        step.source_table.c_str(),
        step.destination_table.c_str());

      all_success = false;
      break;
    }
  }

  if (!all_success)
  {
    const auto attached_objects =
      planning_scene_interface.getAttachedObjects({cube_id});

    if (!attached_objects.empty())
    {
      RCLCPP_WARN(
        node->get_logger(),
        "Failure recovery: detaching %s.",
        cube_id.c_str());

      move_group.detachObject(cube_id);
      std::this_thread::sleep_for(1s);
    }

    commandGripper(node, gripper_client, 0.1);
  }

  publishStatus(
    visual_tools,
    all_success
      ? "FOUR-TABLE CYCLE: SUCCESS"
      : "FOUR-TABLE CYCLE: FAILED",
    all_success ? rvt::GREEN : rvt::RED);

  RCLCPP_INFO(
    node->get_logger(),
    "========================================");

  RCLCPP_INFO(
    node->get_logger(),
    "FOUR-TABLE CYCLE: %s",
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
