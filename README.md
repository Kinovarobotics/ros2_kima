# ROS 2 KINOVA KORTEX™ (KIMA ADAPTATION)
> ROS 2 driver, description and bringup for the KIMA robotic arm.

This repository provides the ros2_control hardware interface, robot description and launch files for the KIMA robotic arm. The driver talks to the arm over EtherCAT through Kinova's Robot Control Library (RCL), vendored by the `kinova_rcl_vendor` package.


## Getting started

1. Install ROS 2.

   This repository targets ROS 2 Jazzy on Ubuntu 24.04. The vendored Robot Control Library is only published for Linux x86_64 with the gcc-13 ABI.

   Stable LTS Release: [Install ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

   After installing ROS2, source the setup.bash, which will set the `$ROS_DISTRO` environment variable.


## Contributing to this repository or building from source

If you want to build this repository from source or contribute back to the repository read on.

1. Make sure that `colcon` and its extensions are installed:
   ```
   sudo apt install python3-colcon-common-extensions
   ```

2. Create a new ROS2 workspace:
   ```
   export COLCON_WS=~/workspace/ros2_kima_ws
   mkdir -p $COLCON_WS/src
   ```

3. Pull relevant packages:
   ```
   cd $COLCON_WS
   git clone https://github.com/Kinovarobotics/ros2_kima.git src/ros2_kima
   ```

   All other dependencies (ros2_control, the controllers, gz_ros2_control, ros_gz) are released for Jazzy and are installed by `rosdep` in the next step.


4. Install dependencies, compile, and source the workspace:
   ```
   rosdep install --ignore-src --from-paths src -y -r
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

   By default, colcon will use as much resources as possible to build the ROS2 workspace. This can temporarily freeze or even crash your machine. You can limit the number of threads used to avoid this issue, we found a good tradeoff between build time and resource utilisation by setting it to 3 :
   ```
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 3
   ```
5. Source the previously built workspace using the following command:
   ```
   echo 'source ~/workspace/ros2_kortex_ws/install/setup.bash' >> ~/.bashrc
   ```

6. Make sure to bring up the Ethernet connection to the robotic arm using:
   ```
   sudo ip link set <ip_link_name> up
   ```
   where the ip_link_name can be identified using:
   ```
   ip link
   ```

## Usage

### KIMA Robots

The `kima.launch.py` launch file is designed to be used for KIMA arms. The typical use case to bringup and visualize the KIMA robotic arm with mock hardware on Rviz:

```bash
ros2 launch kortex_bringup kima.launch.py \
  robot_ip:=yyy.yyy.yyy.yyy \
  use_fake_hardware:=true
```

Alternatively, for a physical robot:

```bash
ros2 launch kortex_bringup kima.launch.py robot_ip:=192.168.1.10 
```
You can specify the following arguments if you wish to change your arm configuration:

* `robot_type`: Your robot model. Default value (and only one) is `kima`.

* `robot_ip` : IP address by which the robot can be reached. No default is specified, this is a required argument. All arms are shipped with address `192.168.1.10`, but if you have reassigned your physical arm's robot IP address, then you will need to assign that ip address.

* `use_fake_hardware` : Start robot with fake hardware mirroring command to its states. Default value is `false`.

* `fake_sensor_commands` : Enable fake command interfaces for sensors used for simple simulations. Used only if 'use_fake_hardware' parameter is true. Default value is `false`.

* `robot_controller` : Robot controller to start. Possible values are `twist_controller` and `joint_trajectory_controller`.Default value is `joint_trajectory_controller`.

* `controllers_file` : Ros 2 control configuration file to use. Default value is `ros2_controllers.yaml`

* `launch_rviz` : Start an Rviz window to visualize the robot. Default value is `true`.

### Simulation (Gazebo)

To run the arm in Gazebo Sim (Harmonic) instead of on the hardware:

```bash
ros2 launch kortex_bringup kortex_sim_control.launch.py
```

This starts Gazebo, spawns the arm and activates `joint_state_broadcaster` and `joint_trajectory_controller`, so the arm can be commanded exactly as described in [Commanding the arm](#commanding-the-arm). Useful arguments:

* `launch_rviz` : Start RViz. Default value is `true`.

* `gz_args` : Arguments passed to `gz sim`. Default value is `" -r -v 3 empty.sdf"`. Add `-s` to run the server headless, without the Gazebo GUI.

* `controllers_file` : Controllers configuration, in `kortex_description/arms/kima/7dof/config/`. Default value is `ros2_controllers_sim.yaml`.

In simulation, `gz_ros2_control` replaces the EtherCAT driver, so the fault controller and the twist controller are not available. They need interfaces that only the real driver provides.

### MoveIt

The `kima_moveit_config` package adds MoveIt (`move_group`) and RViz with the MotionPlanning panel on top of the regular bringup:

```bash
# Real arm
ros2 launch kima_moveit_config robot.launch.py

# Mock hardware
ros2 launch kima_moveit_config robot.launch.py use_fake_hardware:=true

# Gazebo simulation
ros2 launch kima_moveit_config robot.launch.py sim:=true
```

Arguments:

* `use_fake_hardware` : Run on mock hardware instead of the real arm. Default value is `false`.

* `sim` : Run in Gazebo instead of on the real arm. It takes precedence over `use_fake_hardware`. Default value is `false`.

* `launch_rviz` : Start RViz with the MotionPlanning panel. Default value is `true`.

* `robot_ip` : Forwarded to `kima.launch.py`. The EtherCAT driver does not use it. Default value is `0.0.0.0`.

The planning group is `manipulator` (`base_link` to `tool_frame`), with a `home` named state (all joints at 0). The OMPL (default) and Pilz (`PTP`, `LIN`, `CIRC`) planners are available. Motions execute through `joint_trajectory_controller`.

**Velocity and acceleration are scaled to 10% by default** (`config/joint_limits.yaml`). Raise them in the MotionPlanning panel, or in the planning request, once motions have been validated on the arm.

The disabled collision pairs in `config/kima.srdf` were generated by sampling. If the meshes change, regenerate them with:

```bash
ros2 run moveit_setup_assistant collisions_updater \
  --urdf src/ros2_kima/kortex_description/robots/kima.xacro \
  --srdf src/ros2_kima/kima_moveit_config/config/kima.srdf \
  --output src/ros2_kima/kima_moveit_config/config/kima.srdf \
  --default --always --trials 100000
```

The package can also be opened in the MoveIt Setup Assistant (`ros2 launch moveit_setup_assistant setup_assistant.launch.py`), which picks up `.setup_assistant`.

## Clearing arm faults

Actuator faults stay latched in the actuators across restarts: stopping the launch does not clear them. There are two ways to clear them.

### While the arm is running: fault controller

If the launch is up, clear the fault through the `fault_controller`:

```bash
ros2 service call /fault_controller/reset_fault example_interfaces/srv/Trigger
```

On success the driver re-enters real-time mode from the arm's current position. When a fault latches, the driver logs its causes and the per-actuator fault banks (`Arm fault latched. ... Actuator fault banks:`).

### When the launch cannot start: RCL directly

If a fault is still latched when the launch starts, the hardware may fail to activate and `ros2_control_node` exits with `SetArmMode: 'SelectMode' is not allowed in state 'Fault'`. In that case, stop the launch and clear the fault with the standalone RCL tool, which talks to the arm without ROS:

```bash
ros2 run kortex_driver clear_faults \
  "$(ros2 pkg prefix --share kortex_description)/arms/kima/7dof/config/network_topology_1_arm.yaml"
```

The tool scans the bus, prints the arm state, the latched causes and the fault banks, clears the faults and prints the state again. A successful run ends with:

```
[after] arm state: Idle
[after]   (no actuator reports a non-zero fault bank)
```

Notes:
* Only one program can hold the EtherCAT master at a time, so `ros2_control_node` must not be running.
* The tool never enables the arm: it stays in `Idle` with its brakes engaged.
* RCL sometimes reports `Fault clear incomplete: N actuator(s) still in fault` even though every fault bank already reads zero. The tool, like `~/reset_fault`, retries the clear up to 3 times in that case. If it still ends in `Fault`, run it again.
* The expected arm identity (`ARM-L3M000-001`, revision `A`, serial `0001`, 7 actuators, slaves 0-13) is fixed in `kortex_driver/tools/clear_faults.cpp` and matches the defaults in `kortex.ros2_control.xacro`. The scan fails if the arm on the bus does not match.
* RCL numbers actuators from 0 in its own messages (`Actuator 1 self-test triggered`), while the fault-bank lines number them from 1 (`actuator 2: a=0x2000`). Both refer to the same actuator.



## Commanding the arm
You can command the arm by publishing Joint Trajectory messages directly to the joint trajectory controller with joint positions are in **radians**:

```bash
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
  joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
  points: [
    { positions: [0, 0, 0, 0, 0, 0, 0], time_from_start: { sec: 10 } },
  ]
}" -1
```

You can also command the arm using Twist messages. Before doing so, you must active the `twist_controller` and deactivate the `joint_trajectory_controller`:
```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{
  activate_controllers: [twist_controller],
  deactivate_controllers: [joint_trajectory_controller],
  strictness: 1,
  activate_asap: true,
}"
```

Once the `twist_controller` is activated, you can publish Twist messages on the `/twist_controller/commands` topic to command the arm.

For example, you can jog the arm using [Teleop Twist Keyboard](https://index.ros.org/p/teleop_twist_keyboard/github-ros2-teleop_twist_keyboard/) with the following command:

**WARNING: you are responsible for collision checking, including self collisions when in this mode.**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/twist_controller/commands
```

If you wish to use the `joint_trajectory_controller` again to command the arm using JointTrajectory messages, run the following:
```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{
  activate_controllers: [joint_trajectory_controller],
  deactivate_controllers: [twist_controller],
  strictness: 1,
  activate_asap: true,
}"
```