# ROS 2 KINOVA KORTEX™ (KIMA ADAPTATION)
> Kinova® KINOVA KORTEX™ is the common software platform behind all of the products in the Gen3 family (Gen3 and Gen3 lite). It unifies the inner workings of the various robots and their related external tools, like the API. This repository is an adaptation that supports KIMA robotic arm as well. <br />
> https://www.kinovarobotics.com/product/gen3-robots

ROS2 KINOVA KORTEX™ is the official ROS2 package to interact with KINOVA KORTEX™ and its related products. It is built upon the KINOVA KORTEX™ API, documentation for which can be found in the [GitHub Kortex repository](https://github.com/Kinovarobotics/kortex).


## Getting started

1. Install ROS 2.

   For this branch, ROS2 Humble has to be installed on Ubuntu 22.04.

   Stable LTS Release: [Install ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

   After installing ROS2, source the setup.bash, which will set the `$ROS_DISTRO` environment variable.


## Contributing to this repository or building from source

If you want to build this repository from source or contribute back to the repository read on.

1. Make sure that `colcon`, its extensions, and `vcs` are installed:
   ```
   sudo apt install python3-colcon-common-extensions python3-vcstool
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
   vcs import src --skip-existing --input src/ros2_kima/ros2_kortex.$ROS_DISTRO.repos
   vcs import src --skip-existing --input src/ros2_kima/ros2_kortex-not-released.$ROS_DISTRO.repos
   ```


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