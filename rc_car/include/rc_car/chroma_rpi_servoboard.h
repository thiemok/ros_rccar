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

/**
 * Interface to the Raspberry Pi Servo Board v3
 * See http://electronics.chroma.se/rpisbv3.php
 */

#pragma once


#include <stdio.h>
#include <string>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdbool.h>

class ServoBoard {
 
	public:

		/**
		 * Initialises the interface with the given serial port
		 * @param port The serial port that should be used
		 */
		ServoBoard(std::string port);

		/**
		 * Initialises the interface with the rpi's default serial port /dev/ttyAMA0
		 */
		ServoBoard();

		/**
		 * Deinitialises the interface
		 */
		~ServoBoard();

		 /**
		 * Changes to used baud rate
		 * @param bps The baud rate that should be used
		 *	0: 9600bps
		 *	1: 19200bps
		 *	2: 38400bps
		 */
		void changeBps(int bps);

		/**
		 * Tests the servos by slowly moving them between -100% and 100%
		 */
		void servoTest();

		 /**
		  * Sets speed at which all servos should move
		  * params: One value per servo ranging from 1 to 255 or 0 for as fast as possible.
		  */
		void setAllVelocity(unsigned int servo0, unsigned int servo1, unsigned int servo2,
							unsigned int servo3, unsigned int servo4, unsigned int servo5,
							unsigned int servo6, unsigned int servo7);

		/**
		 * Sets the position for all servos.
		 * params: One value per servo ranging from -1000 to 1000
		 * or -2500 to 1900 if rangeOverride is true
		 */
		void setAllPosition(int servo0, int servo1, int servo2, int servo3,
							int servo4, int servo5, int servo6, int servo7,
							bool rangeOverride);

		/**
		 * Sets the position of the specified servo at the given speed.
		 * @param servo The servo that should be set
		 * @param pos The position to which the servo should be set
		 * 			  Ranging from -1000 to 1000
		 *			  or with rangeOverride from -2500 to 1900
		 * @param speed The speed at which the servo should be set
		 *				Ranging from 1 to 255 and 0 for as fast as possible
		 * @param rangeOverride Specifies if the extended position range should be used
		 */
		void setServo(unsigned int servo, int pos, unsigned int speed, bool rangeOverride);

		/**
		 * Sets the initial servo positions.
		 * At power up the servos are moved to the specified position.
		 * Positions are persistent across reboots.
		 * params: One value per servo ranging from -1000 to 1000
		 * or -2500 to 1900 if rangeOverride is true
		 */
		void setInitialServoPositions(int servo0, int servo1, int servo2, int servo3,
									  int servo4, int servo5, int servo6, int servo7,
									  bool rangeOverride);

		/**
		 * Enables servos
		 * Default state is enabled
		 */
		void enableServos();

		/**
		 * Disables servos
		 * Default state is enabled
		 */
		void disableServos();

	private:

		/**
		 * File descriptor for the serial port
		 */
		int fd;

		/**
		 * The actually initialising function. Is called by the constructor.
		 * @param port The serial port that should be used.
		 */
		void init(std::string port);

		/**
		 * Checks if the velocity is between 0 and 255, if its greater 0 (as fast as possible)
		 * is returned,
		 */ 
		unsigned int checkVelocityRange(unsigned int vel);

		/**
		 * Checks if the position is in the posiotion range, between -1000 and 1000
		 * and returns the position.
		 * If the position is outside the bounds the neares value within bounds will be returned.
		 * @para pos The position
		 */
		int checkPositionRange(int pos);

		/**
		 * Checks if the position is in the extended position range, between -2500 and 1900
		 * and returns the position.
		 * If the position is outside the bounds the neares value within bounds will be returned.
		 * @param pos The position
		 */
		int checkPositionWithExtendedRange(int pos);
};