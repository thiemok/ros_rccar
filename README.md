ros_rccar
=========
An RC Car based on ROS indigo and the Raspberry Pi.

The Project uses the Raspberry Pi Servo board v3, which is available here http://electronics.chroma.se/

Currently only driving is possible via ros *geometry_msgs/Twist* to *rc_car/cmd_vel*.
Use linear.x for throttle (to drive in reverse use negative values), linear.y is used for braking and angular.z for steering.
**To configure the node to your cars paramters use the included launch file.**

*The Readme will be extended as the project progresses.*
