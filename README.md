# ROS 2 KINOVA KORTEX™ (KIMA ADAPTATION)

> ROS 2 driver, description and bringup for the KIMA robotic arm.

This repository provides the ros2_control hardware interface, robot description and launch files for the KIMA robotic arm. The driver talks to the arm over EtherCAT through Kinova's Robot Control Library (RCL), vendored by the `kinova_rcl_vendor` package.

## Getting started

1. Install ROS 2.

   This repository targets ROS 2 Jazzy on Ubuntu 24.04. The vendored Robot Control Library is only published for Linux x86_64 with the gcc-13 ABI.

   Stable LTS release: [Install ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

   After installing ROS 2, source its `setup.bash`, which sets the `$ROS_DISTRO` environment variable.

## Contributing to this repository or building from source

If you want to build this repository from source or contribute back to it, read on.

1. Make sure that `colcon`, its extensions, and `vcs` are installed:

   ```bash
   sudo apt install python3-colcon-common-extensions python3-vcstool
   ```

2. Create a new ROS 2 workspace:

   ```bash
   export COLCON_WS=~/workspace/ros2_kima_ws
   mkdir -p $COLCON_WS/src
   ```

3. Pull the relevant packages:

   ```bash
   cd $COLCON_WS
   git clone https://github.com/Kinovarobotics/ros2_kima.git src/ros2_kima
   vcs import src --skip-existing --input src/ros2_kima/ros2_kima.jazzy.repos
   ```

   `ros2_kima.jazzy.repos` only pulls the FZI `cartesian_controllers`, which are not released for Jazzy. All other dependencies (ros2_control, the controllers, gz_ros2_control, ros_gz, MoveIt) are released for Jazzy and are installed by `rosdep` in the next step.

4. Install the dependencies and compile the workspace:

   ```bash
   rosdep install --ignore-src --from-paths src -y -r
   colcon build --packages-skip cartesian_controller_simulation cartesian_controller_tests --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

   `cartesian_controller_simulation` (MuJoCo) and `cartesian_controller_tests` are part of the FZI repository but are not used here.

   By default, colcon uses as many resources as possible to build the workspace, which can temporarily freeze or even crash your machine. To avoid this, limit the number of parallel workers; 3 is a good trade-off between build time and resource use:

   ```bash
   colcon build --packages-skip cartesian_controller_simulation cartesian_controller_tests --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 3
   ```

5. Source the workspace, and have every new terminal source it:

   ```bash
   source $COLCON_WS/install/setup.bash
   echo 'source ~/workspace/ros2_kima_ws/install/setup.bash' >> ~/.bashrc
   ```

6. Bring up the Ethernet interface connected to the robotic arm:

   ```bash
   sudo ip link set <ip_link_name> up
   ```

   where `<ip_link_name>` can be identified with:

   ```bash
   ip link
   ```

## Usage

### Bringup

The `kima.launch.py` launch file brings up the KIMA arm. To bring up and visualize the arm with mock hardware in RViz:

```bash
ros2 launch kortex_bringup kima.launch.py \
  robot_ip:=yyy.yyy.yyy.yyy \
  use_fake_hardware:=true
```

For the physical arm:

```bash
ros2 launch kortex_bringup kima.launch.py robot_ip:=192.168.1.10
```

Arguments:

* `robot_type`: Robot model. Default value (and only one) is `kima`.

* `robot_ip`: Required by the launch file. The EtherCAT driver does not use it, so any address works.

* `use_fake_hardware`: Start the robot with mock hardware that mirrors commands to its states. Default value is `false`.

* `fake_sensor_commands`: Enable fake command interfaces for sensors, for simple simulations. Only used if `use_fake_hardware` is `true`. Default value is `false`.

* `robot_controller`: Robot controller to start. Default value is `joint_trajectory_controller`.

* `controllers_file`: ros2_control configuration file to use. Default value is `ros2_controllers.yaml`.

* `launch_rviz`: Start RViz to visualize the robot. Default value is `true`.

### Simulation (Gazebo)

To run the arm in Gazebo Sim (Harmonic) instead of on the hardware:

```bash
ros2 launch kortex_bringup kortex_sim_control.launch.py
```

This starts Gazebo, spawns the arm and activates `joint_state_broadcaster` and `joint_trajectory_controller`, so the arm can be commanded exactly as described in [Commanding the arm](#commanding-the-arm).

Arguments:

* `launch_rviz`: Start RViz. Default value is `true`.

* `gz_args`: Arguments passed to `gz sim`. Default value is `" -r -v 3 empty.sdf"`. Add `-s` to run the server headless, without the Gazebo GUI.

* `controllers_file`: Controllers configuration, in `kortex_description/arms/kima/7dof/config/`. Default value is `ros2_controllers_sim.yaml`.

In simulation, `gz_ros2_control` replaces the EtherCAT driver, so the fault controller is not available. It needs interfaces that only the real driver provides.

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

* `use_fake_hardware`: Run on mock hardware instead of the real arm. Default value is `false`.

* `sim`: Run in Gazebo instead of on the real arm. It takes precedence over `use_fake_hardware`. Default value is `false`.

* `launch_rviz`: Start RViz with the MotionPlanning panel. Default value is `true`.

* `robot_ip`: Forwarded to `kima.launch.py`. The EtherCAT driver does not use it. Default value is `0.0.0.0`.

The planning group is `manipulator` (`base_link` to `tool_frame`), with a `home` named state (all joints at 0). The OMPL (default) and Pilz (`PTP`, `LIN`, `CIRC`) planners are available. Motions execute through `joint_trajectory_controller`.

**Velocity and acceleration are scaled to 10% by default** (`config/joint_limits.yaml`). They can be raised in the MotionPlanning panel.

## Commanding the arm

### Joint trajectory controller

Command the arm by publishing `JointTrajectory` messages to the joint trajectory controller. Joint positions are in **radians**:

```bash
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
  joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
  points: [
    { positions: [0, 0, 0, 0, 0, 0, 0], time_from_start: { sec: 10 } },
  ]
}" -1
```

### Cartesian motion controller

`cartesian_motion_controller` ([FZI cartesian_controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers)) moves `tool_frame` to a target pose. It is loaded inactive by `kima.launch.py` and `kortex_sim_control.launch.py`, together with `motion_control_handle`, an interactive marker in RViz whose pose is fed to the controller.

The controller tracks the target pose it receives on `/cartesian_motion_controller/target_frame` (`geometry_msgs/msg/PoseStamped`). It keeps tracking the last target until a new one arrives. That target can come from the RViz handle or from a topic, but only one source should be active at a time.

#### With the RViz handle

Switch from the trajectory controller to Cartesian control with the handle:

```bash
ros2 control switch_controllers \
  --activate cartesian_motion_controller motion_control_handle \
  --deactivate joint_trajectory_controller
```

Then drag the `CartesianTarget` marker in RViz. The handle starts at the current tool pose, so the arm does not jump when it is activated.

The marker is shown by the `CartesianTarget` display (Interactive Markers, namespace `/motion_control_handle`), which is already included in the RViz started by `kima.launch.py`, `kortex_sim_control.launch.py` and `kima_moveit_config robot.launch.py`. If RViz was launched with `launch_rviz:=false`, open it separately with the same configuration:

```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share kortex_description)/rviz/view_robot.rviz
```

With any other RViz configuration, add the display manually: **Add** > **By display type** > `rviz_default_plugins/InteractiveMarkers`, then set **Interactive Markers Namespace** to `/motion_control_handle`.

The marker appears as soon as the handle is loaded, even while it is inactive. It only moves to the current tool pose, and only commands the arm, once `motion_control_handle` and `cartesian_motion_controller` are active.

#### From a ROS 2 topic

1. Switch to the Cartesian controller, and leave `motion_control_handle` inactive so it does not publish targets too:

   ```bash
   ros2 control switch_controllers --activate cartesian_motion_controller --deactivate joint_trajectory_controller
   ```

2. Read the current tool pose:

   ```bash
   ros2 run tf2_ros tf2_echo base_link tool_frame
   ```

   Note the `Translation` and the quaternion (xyzw).

3. Publish the target pose. Edit the position, and paste the orientation read in step 2 unless you want the tool to rotate:

   ```bash
   ros2 topic pub --once -w 1 /cartesian_motion_controller/target_frame geometry_msgs/msg/PoseStamped \
   "{header: {frame_id: base_link}, pose: {position: {x: <x>, y: <y>, z: <z>}, orientation: {x: <x>, y: <y>, z: <z>, w: <w>}}}"
   ```

   * `frame_id` must be exactly `base_link`. A target in any other frame is ignored, and the only sign is a warning in the controller's log.
   * Positions are in metres, relative to `base_link`.
   * `-w 1` waits until the controller is subscribed before sending, so the single message is not lost. `--once` is enough because the controller keeps tracking the last target.

#### Switching back

Switch back to the trajectory controller (needed for MoveIt and joint trajectories) with:

```bash
ros2 control switch_controllers \
  --activate joint_trajectory_controller \
  --deactivate cartesian_motion_controller motion_control_handle
```

Notes:

* The controller has no velocity limit of its own: how fast the arm moves depends on how far the target is and on the gains in `kortex_description/arms/kima/7dof/config/ros2_controllers.yaml` (`solver.error_scale`, `pd_gains`). Start with small target steps on the real arm.
* Activate it away from the straight-up zero pose, which is a kinematic singularity. Move the arm into a bent pose with the trajectory controller first.
* The orientation in the target is followed too: a quaternion different from the current one rotates the tool.

## Clearing arm faults

Actuator faults stay latched in the actuators across restarts: stopping the launch does not clear them. There are two ways to clear them.

### While the arm is running: fault controller

If the launch is up, clear the fault through the `fault_controller`:

```bash
ros2 service call /fault_controller/reset_fault example_interfaces/srv/Trigger
```

On success, the driver re-enters real-time mode from the arm's current position. When a fault latches, the driver logs its causes and the per-actuator fault banks (`Arm fault latched. ... Actuator fault banks:`).

### When the launch cannot start: RCL directly

If a fault is still latched when the launch starts, the hardware may fail to activate and `ros2_control_node` exits with `SetArmMode: 'SelectMode' is not allowed in state 'Fault'`. In that case, stop the launch and clear the fault with the standalone RCL tool, which talks to the arm without ROS:

```bash
ros2 run kortex_driver clear_faults \
  "$(ros2 pkg prefix --share kortex_description)/arms/kima/7dof/config/network_topology_1_arm.yaml"
```

The tool scans the bus, prints the arm state, the latched causes and the fault banks, clears the faults and prints the state again. A successful run ends with:

```text
[after] arm state: Idle
[after]   (no actuator reports a non-zero fault bank)
```

Notes:

* Only one program can hold the EtherCAT master at a time, so `ros2_control_node` must not be running.
* The tool never enables the arm: it stays in `Idle` with its brakes engaged.
* RCL sometimes reports `Fault clear incomplete: N actuator(s) still in fault` even though every fault bank already reads zero. The tool, like `~/reset_fault`, retries the clear up to 3 times in that case. If it still ends in `Fault`, run it again.
* The expected arm identity (`ARM-L3M000-001`, revision `A`, serial `0001`, 7 actuators, slaves 0-13) is fixed in `kortex_driver/tools/clear_faults.cpp` and matches the defaults in `kortex.ros2_control.xacro`. The scan fails if the arm on the bus does not match.
