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
#define RC_CAR_VEL_CMD_CHANNEL "rc_car/cmd_vel"

#include "rc_car_node.h"

RcCar::RcCar() 
{
	//TODO replace this with parameterized initialisation
	this->engineID = 0;
	this->steeringServoID = 1;
	this->steeringServoRightLimit = 775;
	this->steeringServoLeftLimit = 475;
	this->reversingTimeout = ros::Duration(5.0);
	//TODO

	/* Set current command to 0 */
	this->currentCMD.linear.x = 0;
	this->currentCMD.linear.y = 0;
	this->currentCMD.linear.z = 0;
	this->currentCMD.angular.x = 0;
	this->currentCMD.angular.y = 0;
	this->currentCMD.angular.z = 0;

	/* Init Servo Board */
	this->motorController = new ServoBoard();

	/* Subscribe to cmd msgs */
	this->velocitySub = nodeHandle.subscribe(RC_CAR_VEL_CMD_CHANNEL, 1, &RcCar::velocityCallback, this);

	/* Set default values for variables */
	this->inReverse = false;
	this->brakePushedAt = ros::Time(0.0);
}

RcCar::~RcCar() 
{
	delete motorController;
}

void RcCar::mainNodeLoop() 
{
	/* set the loop rate (in hertz) */
	ros::Rate loop_rate(25);
	
	while (ros::ok()) {
	
		/* do your node business here */
		steer(currentCMD.angular.z);
		setEngine(currentCMD.linear.x, currentCMD.linear.y);
	
		/* spin the node once */
		ros::spinOnce();
	
		/* go to sleep for the rest of the node's processing period */
		loop_rate.sleep();
	}
}

/**
 * Sets the steering to the given position.
 * @param angle The steering angle in percent of the full lock. Positive values are interpreted as right.
 */
void RcCar::steer(double angle) {
	if (angle >= 0) {
		motorController->setServo(steeringServoID, steeringServoRightLimit * fmin(angle, 1), 0, false);
	} else {
		motorController->setServo(steeringServoID, steeringServoLeftLimit * fmax(angle, -1), 0, false);
	}
}

/**
 * Sets the engine with the given throttle or brake.
 * @param throttle The throttle, which should be applied to the engine, to reverse use negative value.
 * @param brake The brake, which should be applied. Will override throttle
 */
void RcCar::setEngine(double throttle, double brake) {

	double b = brake;

	/* Check if breake value is not positive */
	if (b < 0) {
		/* Make it positive */
		b = -b;
	}

	/* Check if braking */
	if (b > 0) {
		/* Set breake timestamp if not already done */
		if(brakePushedAt == ros::Time(0.0)) {
			brakePushedAt = ros::Time::now();
		}
		/* Apply Brake */
		if(!inReverse) {
			motorController->setServo(engineID, -1000 * fmin(brake, 1), 0, false);
		} else {
			motorController->setServo(engineID, 1000 * fmin(brake, 1), 0, false);
		}
	} else {
		
		if (throttle >= 0 && !inReverse) { /* move forward */
			/* Check if brake was applied priviously */
			if(brakePushedAt != ros::Time(0.0)) {
				brakePushedAt == ros::Time(0.0);
			}
			/* Apply throttle */
			motorController->setServo(engineID, 1000 * fmin(throttle, 1), 0, false);
		} else if (throttle <= 0 && inReverse) { /* reversing */
			/* Check if brake was applied priviously */
			if(brakePushedAt != ros::Time(0.0)) {
				brakePushedAt == ros::Time(0.0);
			}
			/* Apply throttle */
			motorController->setServo(engineID, 1000 * fmax(throttle, -1), 0, false);
		} else if (throttle < 0 && !inReverse) { /* starting to reverse */
			/* Check if reversing timeout needs to be applied */
			if (brakePushedAt == ros::Time(0.0) || (ros::Time::now() - brakePushedAt) < reversingTimeout) {
				setEngine(0, 1);
			} else {
				inReverse = true;
				setEngine(throttle, 0);
			} 
		} else if (throttle > 0 && inReverse) { /* staring to move forward from reversing */
			/* Check if brake was applied before and applie reversing timeout proactivly if not to protect the gearbox */
			if (brakePushedAt == ros::Time(0.0) || (ros::Time::now() - brakePushedAt) < reversingTimeout) {
				setEngine(0.1, 0);
			} else {
				inReverse = false;
				setEngine(throttle, 0);
			} 
		}
	}
}

/**
 * Subscriber callback to the velocity topic.
 * Stores the instructions send via RC_CAR_VEL_CMD_CHANNEL for execution.
 * Use msg.linear.x for throttle, msg.linear.y for braking and msg.angular.z for steering.
 * @param msg The velocity message
 */
void RcCar::velocityCallback(const geometry_msgs::Twist& msg) {
	currentCMD = msg;
}

void RcCar::printUsageMessage(void)
{
	ROS_INFO("*** Usage of sample node ***\nTo use this node...");
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
	ros::init(argc, argv, "Rc Car");

	RcCar instance;
	instance.printUsageMessage();
	instance.mainNodeLoop();
}

