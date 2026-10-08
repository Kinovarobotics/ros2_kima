# Copyright (c) 2026 Kinova inc. All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD 3-Clause license. Refer to the LICENSE file for details.
"""
Integration test of the KIMA bringup on mock hardware.

Launches kima.launch.py with use_fake_hardware:=true (no arm, no EtherCAT) and
checks that the controllers come up as expected, that a joint trajectory is
executed, and that the Cartesian motion controller tracks a target pose.
"""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import PoseStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = [f"joint_{i}" for i in range(1, 8)]
# A bent pose, away from the straight-up kinematic singularity.
BENT_POSE = [0.0, 0.5, 0.0, 1.2, 0.0, 0.8, 0.0]


@pytest.mark.launch_test
def generate_test_description():
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("kortex_bringup"), "launch", "kima.launch.py")
        ),
        launch_arguments={
            "robot_ip": "0.0.0.0",
            "use_fake_hardware": "true",
            "launch_rviz": "false",
        }.items(),
    )
    return LaunchDescription([bringup, launch_testing.actions.ReadyToTest()])


class TestKimaMockBringup(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_kima_mock_bringup")
        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(cls.tf_buffer, cls.node)
        cls.joint_state = None
        cls.node.create_subscription(JointState, "/joint_states", cls._on_joint_state, 10)
        cls.list_srv = cls.node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        cls.switch_srv = cls.node.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_joint_state(cls, msg):
        cls.joint_state = dict(zip(msg.name, msg.position))

    # ---------------------------------------------------------------- helpers
    def spin_until(self, predicate, timeout):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def call(self, client, request, timeout=10.0):
        self.assertTrue(client.wait_for_service(timeout_sec=timeout), f"{client.srv_name} missing")
        future = client.call_async(request)
        self.assertTrue(self.spin_until(future.done, timeout), f"{client.srv_name} timed out")
        return future.result()

    def controller_states(self):
        return {
            c.name: c.state for c in self.call(self.list_srv, ListControllers.Request()).controller
        }

    def wait_for_controllers(self, expected, timeout=120.0):
        states = {}
        end = time.time() + timeout
        while time.time() < end:
            states = self.controller_states()
            if all(states.get(name) == state for name, state in expected.items()):
                return states
            time.sleep(0.5)
        self.fail(f"controllers not in expected states {expected}: {states}")

    def switch(self, activate, deactivate):
        request = SwitchController.Request(
            activate_controllers=activate,
            deactivate_controllers=deactivate,
            strictness=SwitchController.Request.STRICT,
        )
        self.assertTrue(self.call(self.switch_srv, request).ok, f"switch {activate}/{deactivate}")

    def tool_position(self, timeout=10.0):
        result = []

        def lookup():
            try:
                t = self.tf_buffer.lookup_transform("base_link", "tool_frame", rclpy.time.Time())
                result.append(t.transform)
                return True
            except Exception:
                return False

        self.assertTrue(self.spin_until(lookup, timeout), "no base_link -> tool_frame transform")
        return result[-1]

    def send_trajectory(self, positions, seconds=2):
        client = ActionClient(
            self.node,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )
        self.assertTrue(client.wait_for_server(timeout_sec=10.0))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        goal.trajectory.points = [
            JointTrajectoryPoint(positions=positions, time_from_start=Duration(sec=seconds))
        ]
        send = client.send_goal_async(goal)
        self.assertTrue(self.spin_until(send.done, 10.0))
        handle = send.result()
        self.assertTrue(handle.accepted, "trajectory goal rejected")
        result = handle.get_result_async()
        self.assertTrue(self.spin_until(result.done, seconds + 10.0), "trajectory timed out")
        self.assertEqual(
            result.result().result.error_code, FollowJointTrajectory.Result.SUCCESSFUL
        )

    # ------------------------------------------------------------------ tests
    def test_1_controllers_come_up(self):
        states = self.wait_for_controllers(
            {
                "joint_state_broadcaster": "active",
                "joint_trajectory_controller": "active",
                "cartesian_motion_controller": "inactive",
                "motion_control_handle": "inactive",
            }
        )
        # fault_controller needs the real driver's reset_fault interfaces and is
        # deliberately not spawned on mock hardware.
        self.assertNotIn("fault_controller", states)
        self.assertTrue(self.spin_until(lambda: self.joint_state is not None, 10.0))
        self.assertEqual(sorted(self.joint_state), JOINTS)

    def test_2_joint_trajectory(self):
        self.wait_for_controllers({"joint_trajectory_controller": "active"})
        self.send_trajectory(BENT_POSE)
        self.spin_until(lambda: False, 0.5)
        for name, target in zip(JOINTS, BENT_POSE):
            self.assertAlmostEqual(self.joint_state[name], target, delta=0.01, msg=name)

    def test_3_cartesian_motion(self):
        self.wait_for_controllers({"joint_trajectory_controller": "active"})
        self.send_trajectory(BENT_POSE)
        self.switch(["cartesian_motion_controller"], ["joint_trajectory_controller"])
        try:
            start = self.tool_position()
            pub = self.node.create_publisher(
                PoseStamped, "/cartesian_motion_controller/target_frame", 10
            )
            target = PoseStamped()
            target.header.frame_id = "base_link"
            target.pose.position.x = start.translation.x + 0.05
            target.pose.position.y = start.translation.y
            target.pose.position.z = start.translation.z + 0.03
            target.pose.orientation = start.rotation
            self.assertTrue(self.spin_until(lambda: pub.get_subscription_count() > 0, 10.0))
            pub.publish(target)
            self.spin_until(lambda: False, 4.0)
            end = self.tool_position()
            self.assertAlmostEqual(end.translation.x - start.translation.x, 0.05, delta=0.002)
            self.assertAlmostEqual(end.translation.y - start.translation.y, 0.0, delta=0.002)
            self.assertAlmostEqual(end.translation.z - start.translation.z, 0.03, delta=0.002)
        finally:
            self.switch(["joint_trajectory_controller"], ["cartesian_motion_controller"])

    def test_4_motion_handle_wiring(self):
        # The handle's output is remapped straight onto the controller's input.
        self.wait_for_controllers({"motion_control_handle": "inactive"})
        self.switch(["motion_control_handle"], [])
        try:
            publishers = self.node.get_publishers_info_by_topic(
                "/cartesian_motion_controller/target_frame"
            )
            self.assertIn("motion_control_handle", [p.node_name for p in publishers])
        finally:
            self.switch([], ["motion_control_handle"])
