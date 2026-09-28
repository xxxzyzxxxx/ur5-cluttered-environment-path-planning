"""Launch MoveIt 2 for a UR5 fitted with a Robotiq 2F-85 gripper.

This is the ROS 2 Jazzy replacement for the original Galactic-era launch file.
The UR arm configuration comes from the installed Universal Robots packages,
while the combined robot description and SRDF come from this workspace.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    del context, args, kwargs

    ur_type = LaunchConfiguration("ur_type")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    prefix = LaunchConfiguration("prefix")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    description_share = get_package_share_directory("ur5_gripper_description")
    config_share = get_package_share_directory("ur5_gripper_moveit_config")
    ur_moveit_share = get_package_share_directory("ur_moveit_config")

    robot_xacro = os.path.join(
        description_share, "urdf", "ur5_robotiq85_gripper.urdf.xacro"
    )
    semantic_xacro = os.path.join(config_share, "srdf", "ur_gripper.srdf.xacro")
    controller_yaml = os.path.join(
        ur_moveit_share, "config", "moveit_controllers.yaml"
    )

    robot_mappings = {
        "name": "ur5",
        "ur_type": ur_type,
        "tf_prefix": prefix,
        "use_mock_hardware": use_mock_hardware,
        "mock_sensor_commands": "false",
        "robot_ip": "127.0.0.1",
    }

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description(file_path=robot_xacro, mappings=robot_mappings)
        .robot_description_semantic(
            file_path=semantic_xacro,
            mappings={"robot_name": "ur5", "name": "ur", "prefix": prefix},
        )
        .robot_description_kinematics()
        .joint_limits()
        .trajectory_execution(
            file_path=controller_yaml,
            moveit_manage_controllers=False,
        )
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl"],
            load_all=False,
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    # The Jazzy driver starts the non-scaled controller for mock hardware.
    controller_manager = moveit_config.trajectory_execution[
        "moveit_simple_controller_manager"
    ]
    controller_manager["scaled_joint_trajectory_controller"]["default"] = False
    controller_manager["joint_trajectory_controller"]["default"] = True

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
    )

    rviz_config = os.path.join(ur_moveit_share, "config", "moveit.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        condition=IfCondition(launch_rviz),
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            {"use_sim_time": use_sim_time},
        ],
    )

    return [move_group, rviz]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur5"),
            DeclareLaunchArgument("use_mock_hardware", default_value="true"),
            DeclareLaunchArgument("prefix", default_value=""),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            # Retained for compatibility with the repository's old command line.
            DeclareLaunchArgument("launch_servo", default_value="false"),
            OpaqueFunction(function=launch_setup),
        ]
    )
