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
#include "rc_car/rc_car_lights_node.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define PWM_LEVEL_DEFAULT_FULL 1.0
#define PWM_LEVEL_DEFAULT_ON 0.2
#define PWM_LEVEL_DEFAULT_OFF 0.0
#define BREAKING_DEADZONE 0.3
#define RC_CAR_VEL_CMD_CHANNEL "rc_car/cmd_vel"

/**
 * Led port assignment.
 * The TLC59711 hast ports 1 to 12 available
 * Mapping: R0 G0 B0 R1 G1 B1 R2 G2 B2 R3 G3 B3
 *          0  1  2  3  4  5  6  7  8  9  10 11
 */
uint8_t RcCarLights::allLights[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
uint8_t RcCarLights::frontLights[] = {0, 1};
uint8_t RcCarLights::backLights[] = {3, 4};

RcCarLights::RcCarLights() {
	this->ledDriver = new TLC59711(1);

	//Disable LEDs
	setLights(allLights, ARRAY_SIZE(allLights),PWM_LEVEL_DEFAULT_OFF);
	
	//Setting default values
	setLights(frontLights, ARRAY_SIZE(frontLights), PWM_LEVEL_DEFAULT_ON);
	setLights(backLights, ARRAY_SIZE(backLights), PWM_LEVEL_DEFAULT_ON);
	this->isBreaking = false;

	/* Subscribe to cmd msgs */
	this->velocitySub = this->nodeHandle.subscribe(
			RC_CAR_VEL_CMD_CHANNEL,
			1,
			&RcCarLights::velocityCallback,
			this
		);
}

RcCarLights::~RcCarLights() {
	setLights(allLights, ARRAY_SIZE(allLights), PWM_LEVEL_DEFAULT_OFF);
	delete ledDriver;
}

void RcCarLights::mainNodeLoop() {
	//setLights(frontLights, ARRAY_SIZE(frontLights), PWM_LEVEL_DEFAULT_ON);
	/* ******************************************
	 * Sleep till new messages arrives
	 * ******************************************/
	ros::spin();
	
}

/**
 * Subscriber callback to the velocity topic.
 * Uses the velocity messages to set the state of the cars lights
 * @param msg The Message
 */
void RcCarLights::velocityCallback(const geometry_msgs::Twist& msg) {
	
	//Brake lights
	if((msg.linear.y > BREAKING_DEADZONE) && !isBreaking) {
		this->isBreaking = true;
		setLights(backLights, ARRAY_SIZE(backLights), PWM_LEVEL_DEFAULT_FULL);
	} else if ((msg.linear.y <= BREAKING_DEADZONE) && isBreaking){
		this->isBreaking = false;
		setLights(backLights, ARRAY_SIZE(backLights), PWM_LEVEL_DEFAULT_ON);
	}
}

/**
 * Sets the given lights to the given pwm level.
 * @param lights The lights to set.
 * @param size The size of the lights array.
 * @param level The level.
 */
void RcCarLights::setLights(uint8_t lights[], size_t size, float level) {

 	for(int i = 0; i < size; i++) {
 		this->ledDriver->setLED(lights[i], level);
 		ROS_INFO("Set LED: %d to Level: %f\n", lights[i], level);
 	}
 	this->ledDriver->writeValues();
}

void RcCarLights::printUsageMessage(void) {
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
	ros::init(argc, argv, "rc_car_lights");

	RcCarLights instance;
	instance.printUsageMessage();
	instance.mainNodeLoop();
}
