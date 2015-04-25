/*
The MIT License (MIT)

Copyright (c) 2014 Thiemo Krause

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include "rc_car/joystick_control_node.h"

#define GEAR_HI 1
#define GEAR_LOW 0
#define DIRECTION_FORWARD -1
#define DIRECTION_BACKWARD 1

/**
 * Constructor
 */
JoystickControlNode::JoystickControlNode() {
	/* Initialisation from launch file */
	ros::NodeHandle privateNodeHandle("~");
	privateNodeHandle.param<int>("throttle_axis", this->throttleAxis, 0);
	privateNodeHandle.param<int>("brake_axis", this->brakeAxis, 2);
	privateNodeHandle.param<int>("gear_switch_button", this->gearSwitchBtn, 0);
	privateNodeHandle.param<int>("direction_switch_button", this->directionSwitchBtn, 1);
	privateNodeHandle.param<int>("steering_axis", this->steeringAxis, 1);
	privateNodeHandle.param<double>("HI_gear_mod", this->hiGearMod, 0.75);
	privateNodeHandle.param<double>("LOW_gear_mod", this->lowGearMod, 0.3);

    this->cmdPub = nodeHandle.advertise<geometry_msgs::Twist>("/rc_car/cmd_vel", 1);
    //ros::Rate loop_rate(10);
    this->sub = nodeHandle.subscribe("joy", 1, &JoystickControlNode::joyCallback, this);

    /* Set default values */
    this->gear = GEAR_LOW;
    this->driveDirection = DIRECTION_FORWARD;
    this->gearBtnLastState = 0;
    this->directionBtnLastState = 0;
}

/**
 * Destructor
 */
JoystickControlNode::~JoystickControlNode() {}

/**
 * Main node loop.
 */
void JoystickControlNode::mainNodeLoop() {
	/* ******************************************
	 * Sleep till new messages arrives
	 * ******************************************/
    ros::spin();
}

/**
 * This function prints a short message which informs the user
 * how to use this node.
 */
void JoystickControlNode::printUsageMessage(void) {
	ROS_INFO("*** Usage of JoystickControl Node ***\nTo use this node configure your joystick in the included launch file");
}

/**
 * Subscriber callback to the sensor_msgs/joy topic.
 *
 * @param msg The message
 */
void JoystickControlNode::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {

    if (msg->buttons[gearSwitchBtn] == 1 && gearBtnLastState != 1) { /* Switch gears */
		gear = ((gear == GEAR_HI) ? GEAR_LOW : GEAR_HI);
    }

    if (msg->buttons[directionSwitchBtn] == 1 && directionBtnLastState != 1) { /* Switch direction */
    	driveDirection = ((driveDirection == DIRECTION_FORWARD) ? DIRECTION_BACKWARD : DIRECTION_FORWARD);
    }

    /* Store buttons last state for reference */
    this->gearBtnLastState = msg->buttons[gearSwitchBtn];
    this->directionBtnLastState = msg->buttons[directionSwitchBtn];

    double gearMod = ((gear == GEAR_HI) ? hiGearMod : lowGearMod);

    /* correct throttle and brake range */
    double throttle = (fmin(msg->axes[throttleAxis], 0)) * driveDirection;
    double brake = (fmax(-msg->axes[brakeAxis], 0)); 

    geometry_msgs::Twist pub_msg;
    pub_msg.linear.x = throttle * gearMod;
    pub_msg.linear.y = brake;
    pub_msg.linear.z = 0;
    pub_msg.angular.x = 0;
    pub_msg.angular.y = 0;
    pub_msg.angular.z = msg->axes[steeringAxis];

    cmdPub.publish(pub_msg);
}

/**
 * Main Method
 *
 * @param argc Number of Arguments given to this method
 * @param argv Pointer to the list of arguments (char streams)
 * @return zero if method finished successfully
 */
int main(int argc, char** argv) {
	
	// Register node in the ros environment
	ros::init(argc, argv, "joystick_control_node");

	JoystickControlNode instance;
	instance.printUsageMessage();
	instance.mainNodeLoop();
}

