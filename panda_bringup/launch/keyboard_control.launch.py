#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    # Build MoveIt config
    moveit_config = (
        MoveItConfigsBuilder("panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .joint_limits(file_path="config/hard_joint_limits.yaml")
        .robot_description_kinematics()
        .to_moveit_configs()
    )

    # Servo parameters
    servo_params = {
        "servo_control": {
            "use_gazebo": False,
            "status_topic": "~/status",
            "command_out_topic": "/panda_arm_controller/joint_trajectory",
            "command_out_type": "trajectory_msgs/JointTrajectory",
            "publish_joint_positions": True,
            "publish_joint_velocities": False,
            "publish_joint_accelerations": False,
            "publish_link_dimensions": False,
            "publish_end_effector": False,
            "monitored_planning_scene_topic": "monitored_planning_scene",
            "joint_topic": "/joint_states",
            "check_octomap_collisions": False,
            "override_velocity_scaling_factor": 0.0,
            "planning_frame": "panda_link0",
            "ee_frame": "panda_link8",
            "use_smoothing": True,
            "smoothing_filter_plugin_name": "online_signal_smoothing::ButterworthFilterPlugin",
            "low_latency_mode": False,
            "lower_singularity_threshold": 17.0,
            "hard_stop_singularity_threshold": 30.0,
            "leaving_singularity_threshold_multiplier": 2.0,
            "move_group_name": "panda_arm",
            "planning_group_name": "panda_arm",
        }
    }

    # Acceleration limiting filter parameters
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "panda_arm"}

    # Servo node
    servo_node = Node(
        package="servo_control",
        executable="servo_node",
        parameters=[
            servo_params,
            acceleration_filter_update_period,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("panda_bringup"), "rviz", "panda.rviz")],
    )
    hand_tf=Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--roll", "0", "--pitch", "-1.5708", "--yaw", "3.14",
                   "--frame-id", "panda_hand",
                   "--child-frame-id", "gripper_control_frame"],
    )



    return [servo_node,rviz_node,hand_tf]


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
