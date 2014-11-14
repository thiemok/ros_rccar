ros_rccar
=========
An RC Car based on ROS indigo and the Raspberry Pi.

The Project uses the Raspberry Pi Servo board v3, which is available here http://electronics.chroma.se/

Currently driving is possible via ros *geometry_msgs/Twist* to *rc_car/cmd_vel* or via joystick using *joystick_control_node*.

### Control via rc_car/cmd_vel
Use linear.x for throttle (to drive in reverse use negative values), linear.y is used for braking and angular.z for steering.

### Control via joystick_control_node
Configure the node for your joystick via the launch file.
The node uses two trigger axes for throttle and brake, one axis for steering and a button to switch between gears
and another button to switch between foward and backward driving directions.
Gear switching is done between HI and LOW gears, with configurable ratios.
The gears are simply a modifier applied to the throttle value for finer control.

**To configure the nodes use the included launch files.**

*The Readme will be extended as the project progresses.*
