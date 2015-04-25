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
#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

/**
 * A joystick teleop node for the rc_car.
 * Allows driving forwards and backwards, steering, braking and switching between HI and LOW gears.
 * Publishes on /rc_car/cmd_vel
 */
class JoystickControlNode 
{

public:

	/**
	 * Constructor
	 */
	JoystickControlNode();

	/**
	 * Destructor
	 */
	~JoystickControlNode();

	/**
	 * Main node loop.
	 */
	void mainNodeLoop();
	
	/**
	 * This function prints a short message which informs the user
	 * how to use this node.
	 */
	void printUsageMessage(void);

private:

	/** The node handle **/
    ros::NodeHandle nodeHandle;

	/** The command publisher **/
    ros::Publisher cmdPub;

	/** The sensor subscriber **/
    ros::Subscriber sub;

    /** The throttle axis */
    int throttleAxis;

    /** The brake axis */
    int brakeAxis;

    /** The gear switch button */
    int gearSwitchBtn;

    /** The direction switch button */
    int directionSwitchBtn;

    /** The steering axis */
    int steeringAxis;

    /** The current gear */
    int gear;

    /** The current drive direction */
    int driveDirection;

    /** The modificator value for the HI gear */
    double hiGearMod;

    /** The modificator value for the LOW gear */
    double lowGearMod;

    /** The gear switch buttons last state */
    int gearBtnLastState;

    /** The direction switch buttons last state */
    int directionBtnLastState;

	/**
	 * Subscriber callback to the sensor_msgs/joy topic.
	 *
	 * @param msg The message
	 */
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
};
