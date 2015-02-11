/*
The MIT License (MIT)

Copyright (c) 2015 Thiemo Krause

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
#include "rc_car/tlc59711.h"

/**
 * This node is used to control led lights on the rc car.
 */
class RcCarLights 
{

public:

	/**
	 * Constructor
	 */
	RcCarLights();

	/**
	 * Destructor
	 */
	~RcCarLights();

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

	/** The velocity messages subscriber **/
	ros::Subscriber velocitySub;

	/** The led driver */
	TLC59711* ledDriver;

	bool isBreaking;

	/** LED port assignment */
	static uint8_t allLights[];
	static uint8_t frontLights[];
	static uint8_t backLights[];

	/**
	 * Subscriber callback to the velocity topic.
	 * Uses the velocity messages to set the state of the cars lights
	 * @param msg The Message
	 */
	void velocityCallback(const geometry_msgs::Twist& msg);

	/**
	 * Sets the given lights to the given pwm level.
	 * @param lights The lights to set.
	 * @param size The size of the lights array.
	 * @param level The level.
	 */
	void setLights(uint8_t lights[], size_t size, float level);

};