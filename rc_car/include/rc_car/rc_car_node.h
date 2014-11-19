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
#include <geometry_msgs/Twist.h>
#include <math.h>
#include "rc_car/chroma_rpi_servoboard.h"
#include <unistd.h>

/**
 * This node controls the rc car.
 */
class RcCar 
{

public:

	/**
	 * Constructor
	 */
	RcCar();

	/**
	 * Destructor
	 */
	~RcCar();

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

	/**
	 * Displays state of the engine
	 */
	enum engineState {
	 	state_FORWARD,
	 	state_NEUTRAL,
	 	state_REVERSE
	 };

	/**
	 * The controller that is used to control the motors
	 */
	ServoBoard* motorController;

	/** The node handle **/
	ros::NodeHandle nodeHandle;

	/** The subscriber for velocity messages**/
	ros::Subscriber velocitySub;

	/** The servo ID of the engine's esc **/
	int engineID;

	/** The engines reverseing Timeout **/
	ros::Duration reversingTimeout;

	/** The servo ID of the steering servo **/
	int steeringServoID;

	/** The steering servos's right limit **/
	int steeringServoRightLimit;

	/** The steering servo's left limit */
	int steeringServoLeftLimit;

	/** The currently active command **/
	geometry_msgs::Twist currentCMD;

	/** The cars current driving state **/
	RcCar::engineState currentEngineState;

	/** Time at which braking was executed, will be 0 if not breaking **/
	ros::Time brakePushedAt;

	/**
	 * Sets the steering to the given position.
	 * @param angle The steering angle in percent of the full lock. Positive values are interpreted as right.
	 */
	void steer(double angle);

	/**
	 * Sets the engine with the given throttle or brake.
	 * @param throttle The throttle, which should be applied to the engine, to reverse use negative value.
	 * @param brake The brake, which should be applied. Will override throttle
	 */
	void setEngine(double throttle, double brake);

	/**
 	 * Subscriber callback to the velocity topic.
	 * Stores the instructions send via RC_CAR_VEL_CMD_CHANNEL for execution.
	 * Use msg.linear.x for throttle, msg.linear.y for braking and msg.angular.z for steering.
	 * @param msg The velocity message
	 */
	void velocityCallback(const geometry_msgs::Twist& msg);

	/**
	 * Manages the brake timestamp.
	 * @param braking Flag if the brake has is active.
	 */
	void manageBrakeTimer(bool braking);

	/**
	 * Checks if the reversing timeout is elapsed.
	 * @return true if the reversing timeout is elapsed, false if not.
	 */
	bool checkReversingTimeout();
};
