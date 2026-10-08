# Copyright (c) 2026 Kinova inc. All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD 3-Clause license. Refer to the LICENSE file for details.
"""
MoveIt for the KIMA arm.

Brings up the arm with the regular bringup (real hardware over EtherCAT, mock
hardware, or Gazebo) and adds move_group plus RViz with the MotionPlanning panel.
The hardware side is not duplicated here: kortex_bringup stays the single source
of truth for the driver, the controllers and their configuration.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    sim = LaunchConfiguration("sim").perform(context) == "true"
    # Resolved now, before the bringup is included: the included launch files
    # receive launch_rviz:=false, and include arguments are not scoped to the
    # included file on their own.
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context)
    use_sim_time = {"use_sim_time": sim}

    bringup_launch_dir = os.path.join(get_package_share_directory("kortex_bringup"), "launch")
    if sim:
        # Gazebo Sim with gz_ros2_control in place of the EtherCAT driver.
        robot_bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_launch_dir, "kortex_sim_control.launch.py")
            ),
            launch_arguments={"launch_rviz": "false"}.items(),
        )
    else:
        # Real arm over EtherCAT, or mock hardware with use_fake_hardware:=true.
        robot_bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_launch_dir, "kima.launch.py")),
            launch_arguments={
                "robot_ip": LaunchConfiguration("robot_ip"),
                "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                "launch_rviz": "false",
            }.items(),
        )

    # move_group only needs the kinematics and collision geometry, which are the
    # same for every bringup variant, so it always loads the real-arm description.
    moveit_config = (
        MoveItConfigsBuilder("kima", package_name="kima_moveit_config")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("kortex_description"), "robots", "kima.xacro"
            )
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        # robot_description is already published by robot_state_publisher from the
        # bringup, and the controller_manager subscribes to that topic. Publishing
        # this copy too would race it, and this copy always selects the real
        # EtherCAT driver, so a mock or sim run could pick up the wrong hardware.
        .planning_scene_monitor(
            publish_robot_description=False, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), use_sim_time],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("kima_moveit_config"), "config", "moveit.rviz"
            ),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            use_sim_time,
        ],
        condition=IfCondition(launch_rviz),
    )

    # RViz must not start before the arm's joint states exist. The MotionPlanning
    # panel initializes its start state when it loads and does not refresh it on
    # its own; loaded too early, it starts from the URDF default (all zeros), so it
    # shows the wrong pose and plans are rejected at execution because their start
    # does not match the real arm. Wait for the first /joint_states message first.
    wait_for_joint_states = ExecuteProcess(
        cmd=["ros2", "topic", "echo", "--once", "/joint_states", "--field", "header.stamp"],
        name="wait_for_joint_states",
        output="log",
        condition=IfCondition(launch_rviz),
    )
    start_rviz_after_joint_states = RegisterEventHandler(
        OnProcessExit(target_action=wait_for_joint_states, on_exit=[rviz_node])
    )

    # Scope the bringup so the arguments passed to it (launch_rviz:=false in
    # particular) do not overwrite this file's launch configurations.
    return [
        GroupAction([robot_bringup], scoped=True),
        move_group_node,
        wait_for_joint_states,
        start_rviz_after_joint_states,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_ip",
            default_value="0.0.0.0",
            description="Forwarded to kima.launch.py, which requires it. The EtherCAT "
            "driver does not use it.",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Run on mock hardware instead of the real arm.",
        ),
        DeclareLaunchArgument(
            "sim",
            default_value="false",
            choices=["true", "false"],
            description="Run in Gazebo Sim instead of on the real arm (ignores use_fake_hardware).",
        ),
        DeclareLaunchArgument(
            "launch_rviz", default_value="true", description="Launch RViz with MoveIt?"
        ),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
