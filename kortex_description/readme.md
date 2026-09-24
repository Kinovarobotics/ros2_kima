<!--
* KINOVA (R) KINOVA KORTEX™ (TM)
*
* Copyright (c) 2018 Kinova inc. All rights reserved.
*
* This software may be modified and distributed
* under the terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
* -->

# KIMA Description
This package contains the URDF (Unified Robot Description Format), STL and configuration files for the KIMA robotic arm.

## Usage

To load the description of the arm, load **robots/kima.xacro**. The ros2_control configuration (EtherCAT hardware parameters, network topology and controllers) lives under **arms/kima/7dof/**.

For example, to generate the URDF with mock hardware:

<code>xacro $(ros2 pkg prefix --share kortex_description)/robots/kima.xacro use_fake_hardware:=true</code>

## Tool frame

The `tool_frame` link refers to the tool frame used by the arm when it reports end effector position feedback.

## Xacro Parameters for `load_robot` macro

### Parameter Table
Param | Description | Default |
:---- | :---------- | :------ |
`parent` | Parent link in the URDF the arm should be attached to | - |
`origin` | Origin of the robot relative to the specified parent link | - |
`prefix` | This is an optional prefix for all joint and link names in the kortex_description. It is used to allow differentiating between different arms in the same URDF. | - |
`arm` | Name of your robot arm model. | - |
`dof` | Number of DOFs of your robot. | - |
`vision` | Boolean value to indicate if your arm has a Vision Module. This argument only affects the visual representation of the arm in RViz. | - |
`robot_ip` | The IP address of the robot you're connection to. | - |
`username` | The username for the robot connection. | - |
`password` | The password for the robot connection. | - |
`port` | Port for KINOVA KORTEX™ hardware driver | - |
`port_realtime` | Realtime port for KINOVA KORTEX™ hardware driver | - |
`session_inactivity_timeout_ms` | The duration after which the robot will clean the client session if the client hangs up the connection brutally (should not happen with the ROS driver). | - |
`connection_inactivity_timeout_ms` | The duration after which a connection is destroyed by the robot if no communication is detected between the client and the robot. | - |
`use_fake_hardware` | Boolean value to indicate whether or not the hardware components will be mocked. If true the hardware params will be ignored and the hardware components will be mocked. | false |
`fake_sensor_commands` | Boolean value. If set to true will create fake command interfaces for faking sensor measurements with an external command. | false |
`sim_gazebo` | Boolean value to indicate whether or not the gazebo_ros2_control/GazeboSystem plugin will be loaded. | false |
`sim_ignition` | Boolean value to indicate whether or not the ign_ros2_control/IgnitionSystem plugin will be loaded. | false |
`sim_isaac` | Boolean value to indicate whether or not the topic_based_ros2_control/TopicBasedSystem plugin will be loaded and the "joint_commands_topic" and "joint_states_topic" parameters will be set to the `isaac_joint_commands` and `isaac_joint_states` values respectively. | false |
`isaac_joint_commands` | Name of the joint commands topic to be used by Isaac Sim. | /isaac_joint_commands |
`isaac_joint_states` | Name of the joint states topic to be used by Isaac Sim. | /isaac_joint_states |
`use_external_cable` | Boolean value that sets joint limits to avoid wrapping of external cables if true. | false |
`initial_positions` | Dictionary of initial joint positions. | {joint_1: 0.0, joint_2: 0.0, joint_3: 0.0, joint_4: 0.0, joint_5: 0.0, joint_6: 0.0, joint_7: 0.0} |

### Example Usage
#### KIMA arm
```
<robot ...>
...
  <xacro:property name="initial_positions" value="${dict(joint_1=0.0, joint_2=0.0, joint_3=0.0, joint_4=0.0, joint_5=0.0, joint_6=0.0, joint_7=0.0)}"/>
  <xacro:load_robot
    parent="world"
    arm="kima"
    dof="7"
    vision="false"
    robot_ip="192.168.1.10"
    sim_gazebo="false"
    sim_ignition="false"
    sim_isaac="false"
    prefix=""
    use_fake_hardware="false"
    initial_positions="${initial_positions}"
    use_external_cable="false"
    fake_sensor_commands="false">
    <origin xyz="0 0 0.0" rpy="0 0 0" />
  </xacro:load_robot>
...
</robot>
```
