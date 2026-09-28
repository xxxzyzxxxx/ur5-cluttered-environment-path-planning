"""Launch the repository's service node with the Jazzy UR5 + gripper model."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    description_share = get_package_share_directory("ur5_gripper_description")
    config_share = get_package_share_directory("ur5_gripper_moveit_config")

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description(
            file_path=os.path.join(
                description_share, "urdf", "ur5_robotiq85_gripper.urdf.xacro"
            ),
            mappings={
                "name": "ur5",
                "ur_type": "ur5",
                "tf_prefix": "",
                "use_mock_hardware": "true",
                "mock_sensor_commands": "false",
                "robot_ip": "127.0.0.1",
            },
        )
        .robot_description_semantic(
            file_path=os.path.join(config_share, "srdf", "ur_gripper.srdf.xacro"),
            mappings={"robot_name": "ur5", "name": "ur", "prefix": ""},
        )
        .robot_description_kinematics()
        .to_moveit_configs()
    )

    service_node = Node(
        package="moveit_ur5_interface",
        executable="moveit_srv_node",
        name="moveit_srv_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([service_node])
