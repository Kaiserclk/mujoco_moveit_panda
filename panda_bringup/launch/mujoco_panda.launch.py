#!/usr/bin/env python3

"""
Usage:
    ros2 launch mujoco_ros2_control_demos mujoco_panda.launch.py
    ros2 launch mujoco_ros2_control_demos mujoco_panda.launch.py headless:=true

Control the arm:
    ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]"

Control the gripper:
    ros2 topic pub /gripper_controller/commands std_msgs/msg/Float64MultiArray "data: [0.04]"
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare("panda_bringup")
    panda_description_share=FindPackageShare("panda_description")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([panda_description_share, "urdf", "panda.urdf"]),
            " headless:=",
            LaunchConfiguration("headless"),
        ]
    )

    robot_description_str = robot_description_content.perform(context)
    robot_description = {"robot_description": ParameterValue(value=robot_description_str, value_type=str)}

    parameters_file = PathJoinSubstitution([pkg_share, "config", "ros2_controllers.yaml"])

    nodes = []

    nodes.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"use_sim_time": True}],
        )
    )

    nodes.append(
        Node(
            package="mujoco_ros2_control",
            executable="ros2_control_node",
            emulate_tty=True,
            output="both",
            parameters=[
                {"use_sim_time": True},
                ParameterFile(parameters_file),
            ],
            remappings=(
                [("~/robot_description", "/robot_description")] if os.environ.get("ROS_DISTRO") == "humble" else []
            ),
            on_exit=Shutdown(),
        )
    )

    controllers_to_spawn = ["joint_state_broadcaster", "panda_arm_controller", "panda_hand_controller"]
    for controller in controllers_to_spawn:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--param-file", parameters_file],
                output="both",
            )
        )

    return nodes


def generate_launch_description():
    headless = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run simulation without visualization window",
    )

    return LaunchDescription(
        [
            headless,
            OpaqueFunction(function=launch_setup),
        ]
    )
