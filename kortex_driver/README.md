# ROS 2 KINOVA KORTEX™ Driver
The ROS 2 KINOVA KORTEX™ driver implements the `ros2_control` hardware interface for a `SystemInterface`.

### Command interfaces
This driver exports a position command interface for each joint defined in the URDF (the KIMA arm is commanded in real-time joint position mode through RCL).
Additional interfaces are exported for fault management: `reset_fault/command` and `reset_fault/async_success`.

### State interfaces
This driver exports position, velocity, and effort state interfaces for each joint defined in the URDF.

Additionally, one state interface `reset_fault/internal_fault` is used for determining the robot's fault state.
