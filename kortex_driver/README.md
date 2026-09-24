# ROS 2 KINOVA KORTEX™ Driver
The ROS 2 KINOVA KORTEX™ driver implements the `ros2_control` hardware interface for a `SystemInterface`.

### Command interfaces
This driver exports commands interfaces for position, velocity, and effort interfaces for each joint defined in the URDF.
Additionally, twist interfaces are exported for the end effector for operational space control.
Additional interfaces are exported for fault management: `reset_fault/command` and `reset_fault/async_success`.

### State interfaces
This driver exports position and velocity state interfaces for joint defined in the URDF.

Additionally, one state interface `reset_fault/internal_fault` is used for determining the robot's fault state.
