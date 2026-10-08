# Copyright (c) 2026 Kinova inc. All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD 3-Clause license. Refer to the LICENSE file for details.
"""
Integration test of MoveIt on the KIMA arm with mock hardware.

Launches kima_moveit_config robot.launch.py with use_fake_hardware:=true (no arm,
no EtherCAT, no RViz) and plans and executes motions with OMPL and Pilz through
the move_group action.
"""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
import pytest
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

JOINTS = [f"joint_{i}" for i in range(1, 8)]
BENT_POSE = [0.5, 0.4, 0.0, 0.8, 0.0, 0.6, 0.3]
HOME = [0.0] * 7


@pytest.mark.launch_test
def generate_test_description():
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("kima_moveit_config"), "launch", "robot.launch.py"
            )
        ),
        launch_arguments={"use_fake_hardware": "true", "launch_rviz": "false"}.items(),
    )
    return LaunchDescription([moveit, launch_testing.actions.ReadyToTest()])


class TestMoveItMock(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_moveit_mock")
        cls.joint_state = None
        cls.node.create_subscription(JointState, "/joint_states", cls._on_joint_state, 10)
        cls.move_group = ActionClient(cls.node, MoveGroup, "/move_action")
        cls.jtc_action = ActionClient(
            cls.node, FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory"
        )
        cls.list_srv = cls.node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_joint_state(cls, msg):
        cls.joint_state = dict(zip(msg.name, msg.position))

    def spin_until(self, predicate, timeout):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def wait_until_ready(self, timeout=180.0):
        # The bringup spawns its controllers in the background: move_group can be
        # up long before joint_trajectory_controller is, and execution fails with
        # CONTROL_FAILED until it is active and its action server is reachable.
        self.assertTrue(self.move_group.wait_for_server(timeout_sec=timeout), "move_group missing")
        self.assertTrue(self.list_srv.wait_for_service(timeout_sec=timeout))
        end = time.time() + timeout
        while time.time() < end:
            future = self.list_srv.call_async(ListControllers.Request())
            self.spin_until(future.done, 10.0)
            states = {c.name: c.state for c in future.result().controller} if future.done() else {}
            if states.get("joint_trajectory_controller") == "active":
                break
            time.sleep(0.5)
        else:
            self.fail(f"joint_trajectory_controller never became active: {states}")
        self.assertTrue(self.jtc_action.wait_for_server(timeout_sec=30.0))
        self.assertTrue(self.spin_until(lambda: self.joint_state is not None, 30.0))
        # Give move_group's own trajectory action client time to connect too.
        self.spin_until(lambda: False, 2.0)

    def plan_and_execute(self, positions, pipeline, planner=""):
        self.wait_until_ready()

        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = "manipulator"
        request.pipeline_id = pipeline
        request.planner_id = planner
        request.num_planning_attempts = 1
        request.allowed_planning_time = 10.0
        request.max_velocity_scaling_factor = 0.5
        request.max_acceleration_scaling_factor = 0.5
        request.goal_constraints = [
            Constraints(
                joint_constraints=[
                    JointConstraint(
                        joint_name=name,
                        position=value,
                        tolerance_above=0.01,
                        tolerance_below=0.01,
                        weight=1.0,
                    )
                    for name, value in zip(JOINTS, positions)
                ]
            )
        ]
        goal.planning_options.plan_only = False

        send = self.move_group.send_goal_async(goal)
        self.assertTrue(self.spin_until(send.done, 30.0))
        handle = send.result()
        self.assertTrue(handle.accepted, "MoveGroup goal rejected")
        result = handle.get_result_async()
        self.assertTrue(self.spin_until(result.done, 60.0), "MoveGroup goal timed out")
        self.assertEqual(result.result().result.error_code.val, MoveItErrorCodes.SUCCESS)

        self.spin_until(lambda: False, 0.5)
        for name, value in zip(JOINTS, positions):
            self.assertAlmostEqual(self.joint_state[name], value, delta=0.02, msg=name)

    def test_1_ompl_joint_goal(self):
        self.plan_and_execute(BENT_POSE, "ompl")

    def test_2_pilz_ptp_home(self):
        self.plan_and_execute(HOME, "pilz_industrial_motion_planner", "PTP")
