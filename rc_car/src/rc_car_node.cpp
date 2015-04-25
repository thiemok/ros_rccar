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

#include "rc_car/rc_car_node.h"

RcCar::RcCar() 
{
	/* Initialisation from launch file */
	ros::NodeHandle privateNodeHandle("~");
	privateNodeHandle.param<int>("engine_id", this->engineID, 0);
	privateNodeHandle.param<int>("steering_servo_id", this->steeringServoID, 1);
	privateNodeHandle.param<int>("steering_servo_right_limit", this->steeringServoRightLimit, 200);
	privateNodeHandle.param<int>("steering_servo_left_limit", this->steeringServoLeftLimit, 200);
	double t;
	privateNodeHandle.param<double>("reversing_timeout", t, 5.0);
	this->reversingTimeout = ros::Duration(t);
	privateNodeHandle.param<double>("esc_reverse_mode_timeout", t, 1.0);
	this->escReversingTimeout = ros::Duration(t);
	this->reverseSwitchTimer = nodeHandle.createTimer(this->escReversingTimeout, &RcCar::switchToReverseTimerCallback,
													  this, true, false);


	/* Set current command to 0 */
	this->currentCMD.linear.x = 0;
	this->currentCMD.linear.y = 0;
	this->currentCMD.linear.z = 0;
	this->currentCMD.angular.x = 0;
	this->currentCMD.angular.y = 0;
	this->currentCMD.angular.z = 0;

	/* Init Servo Board */
	this->motorController = new ServoBoard();
	this->motorController->changeBps(2);

	/* Subscribe to cmd msgs */
	this->velocitySub = nodeHandle.subscribe(RC_CAR_VEL_CMD_CHANNEL, 1, &RcCar::velocityCallback, this);

	/* Set default values for variables */
	this->currentEngineState = state_NEUTRAL;
	this->brakePushedAt = ros::Time(0.0);
	this->lockEngine = false;
}

RcCar::~RcCar() 
{
	delete motorController;
}

void RcCar::mainNodeLoop() 
{

	/* ******************************************
	 * Sleep till new messages arrives
	 * ******************************************/
    //ros::spin();

	/* set the loop rate (in hertz) */
	ros::Rate loop_rate(50);
	
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
		motorController->setServo(steeringServoID, steeringServoRightLimit * fmax(angle, -1), 0, false);
	} else {
		motorController->setServo(steeringServoID, steeringServoLeftLimit * fmin(angle, 1), 0, false);
	}
}

/**
 * Manages the brake timestamp.
 * @param braking Flag if the brake has is active.
 */
void RcCar::manageBrakeTimer(bool braking) {
	if (braking) {	//braking?
		if (brakePushedAt == ros::Time(0.0)) {	//Brake not already pushed?
			brakePushedAt = ros::Time::now();
		}
	} else {	//not braking
		if (brakePushedAt != ros::Time(0.0)) { //Been braking before?
			brakePushedAt = ros::Time(0.0);
		}
	}
}

/**
 * Checks if the reversing timeout is elapsed.
 * @return true if the reversing timeout is elapsed, false if not.
 */
bool RcCar::checkReversingTimeout() {
	return (brakePushedAt != ros::Time(0.0)) && ((ros::Time::now() - brakePushedAt) >  reversingTimeout);
}

/**
 * Switches the ESC to Reverse Mode
 * @param e The Timer event.
 */
void RcCar::switchToReverseTimerCallback(const ros::TimerEvent& e) {
	motorController->setServo(engineID, currentCMD.linear.x * 1000, 0, false);
	currentEngineState=state_REVERSE;
	lockEngine = false;
}

/**
 * Sets the engine with the given throttle or brake.
 * @param throttle The throttle, which should be applied to the engine, to reverse use negative value.
 * @param brake The brake, which should be applied. Will override throttle
 */
void RcCar::setEngine(double throttle, double brake) {

	if(!lockEngine) {

		double b = brake;
		
		/* The actually executed engine speed */
		double cmd = 0;

		/* Check if breake value is not positive */
		if (b < 0) {
			/* Make it positive */
			b = -b;
		}

		/* determine engine state */
		switch (currentEngineState) {
			case state_FORWARD:
				if (b > 0) {	//braking?
					cmd = -1 * b;
					manageBrakeTimer(true);
				} else {	//not braking
					/* Check for state change */
					if(throttle <= 0) {	//reverse or neutral?
						/* Apply reversing timeout */
						if (checkReversingTimeout()) { //reversing timeout elapsed?
							/* Change to corresponding engine state */
							if (throttle < 0) {	//change to reverse
								cmd = 0;
								lockEngine = true;
								reverseSwitchTimer.setPeriod(escReversingTimeout);
								reverseSwitchTimer.start();

							} else {	//change to neutral
								currentEngineState = state_NEUTRAL;
								cmd = throttle;
							}
							manageBrakeTimer(false);
						} else { //reversing timeout not elapsed - brake
							cmd = -1;
							manageBrakeTimer(true);
						}
					} else {	//no state change - forward
						cmd = throttle;
						manageBrakeTimer(false);
					}
				}
				break;

			case state_NEUTRAL:
				if (b > 0) {	//braking?
					cmd = 0;
					manageBrakeTimer(true);
				} else {	//not braking
					/* Check for state change */
					if (throttle > 0) {	//forward?
						currentEngineState = state_FORWARD;
					} else if (throttle < 0) { //reverse?
						currentEngineState = state_REVERSE;
					}

					cmd = throttle;
					manageBrakeTimer(false);
				}
				break;

			case state_REVERSE:
				if (b > 0) {	//braking?
					cmd = 0;
					manageBrakeTimer(true);
				} else {	//not braking
					/* Check for state change */
					if (throttle >= 0) {	//forward or neutral?
						/* Apply reversing timeout proactivly to protect the gearbox */
						if (checkReversingTimeout()) {	//reversing timeout elapsed?
							/* Change to corresponding engine state */
							currentEngineState = (throttle > 0) ? state_FORWARD : state_NEUTRAL;

							cmd = throttle;
							manageBrakeTimer(false);
						} else {	//reversing timeout not elapsed - brake
							cmd = 0.01;
							manageBrakeTimer(true);
						}
					} else { //no state change - reverse
						cmd = throttle;
						manageBrakeTimer(false);
					}
				}
				break;
		}

		motorController->setServo(engineID, 1000 * cmd, 0, false);
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
	ROS_INFO("*** Usage of rc car node ***\nTo use this node configure your cars parameters in the provided launch file.");
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
	ros::init(argc, argv, "rc_car");

	RcCar instance;
	instance.printUsageMessage();
	instance.mainNodeLoop();
}

