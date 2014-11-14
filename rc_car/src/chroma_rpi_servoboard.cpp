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
 * @see http://electronics.chroma.se/rpisbv3.php
 *
 * Some code taken from http://electronics.chroma.se/servo.c
 */

#include "rc_car/chroma_rpi_servoboard.h"

#define uint unsigned int

#define RPI_DEFAULT_SERIAL_PORT "/dev/ttyAMA0"

/**
 * Changes to used baud rate
 * @param bps The baud rate that should be used
 *	0: 9600bps
 *	1: 19200bps
 *	2: 38400bps
 */
void ServoBoard::changeBps(int bps) {
	struct termios options;

	/* Get the current options for the port */
	tcgetattr(fd, &options);

	/* Set baud rates */
	switch(bps) {
		case 0:
			write(fd, "sbr 0\r", 6);
			cfsetispeed(&options, B9600);
			cfsetospeed(&options, B9600);
			break;
		case 1:
			write(fd, "sbr 1\r", 6);
			cfsetispeed(&options, B19200);
			cfsetospeed(&options, B19200);
			break;
		case 2:
			write(fd, "sbr 2\r", 6);
			cfsetispeed(&options, B38400);
			cfsetospeed(&options, B38400);
			break;
		default:
			perror("Servo Board: Trying to set undefined baud rate");
			break;
	}

	/* Enable the reciever and set local mode */
	options.c_cflag |= (CLOCAL | CREAD);

	/* Set the new options for the port */
	tcsetattr(fd, TCSADRAIN, &options);
}

/**
 * Checks if the velocity is between 0 and 255, if its greater 0 (as fast as possible)
 * is returned,
 */ 
uint ServoBoard::checkVelocityRange(uint vel) {
	return ((vel <= 255)? vel : 0);
}

/**
 * Checks if the position is in the posiotion range, between -1000 and 1000
 * and returns the position.
 * If the position is outside the bounds the neares value within bounds will be returned.
 * @para pos The position
 */
int ServoBoard::checkPositionRange(int pos) {
	int p;
	if (pos < -1000) {
		p = -1000;
	} else if (pos > 1000) {
		p = 1000;
	} else {
		p = pos;
	}
	return pos;
}

/**
 * Checks if the position is in the extended position range, between -2500 and 1900
 * and returns the position.
 * If the position is outside the bounds the neares value within bounds will be returned.
 * @param pos The position
 */
int ServoBoard::checkPositionWithExtendedRange(int pos) {
	int p;
	if (pos < -2500) {
		p = -2500;
	} else if (pos > 1900) {
		p = 1900;
	} else {
		p = pos;
	}
	return pos;
}

/**
 * Initialises the interface with the given serial port
 * @param port The serial port that should be used
 * Returns: -1 if initialisation fails
 */
ServoBoard::ServoBoard(std::string port) {
	char cport[80];
	strcpy(cport, port.c_str());

	fd = open(cport, O_RDWR | O_NOCTTY | O_NDELAY);

	/* Check if port could be opened */
	if (fd== -1) {
		char err[80];
		strcpy(err, "Init Servo Board: Unable to open ");
		strcat(err, cport);
		perror(err);
	} else {
		fcntl(fd, F_SETFL, 0);

		/* Set baud raute to boards default */
		changeBps(0);
	}
}

/**
 * Initialises the interface with the rpi's default serial port /dev/ttyAMA0
 * Returns: -1 if initialisation fails
 */
ServoBoard::ServoBoard() {
	ServoBoard(RPI_DEFAULT_SERIAL_PORT);
}

/**
 * Deinitialises the interface
 */
ServoBoard::~ServoBoard() {
 	write(fd, "sbr 0\r", 6);
}

/**
 * Tests the servos by slowly moving them between -100% and 100%
 */
void ServoBoard::servoTest() {
	write(fd, "st\r", 3);
}

 /**
  * Sets speed at which all servos should move
  * params: One value per servo ranging from 1 to 255 or 0 for as fast as possible.
  */
void ServoBoard::setAllVelocity(uint servo0, uint servo1, uint servo2, uint servo3,
					uint servo4, uint servo5, uint servo6, uint servo7) {

	uint s0, s1, s2, s3, s4, s5, s6, s7;

	/* Check for range */
	s0 = checkVelocityRange(servo0);
	s1 = checkVelocityRange(servo1);
	s2 = checkVelocityRange(servo2);
	s3 = checkVelocityRange(servo3);
	s4 = checkVelocityRange(servo4);
	s5 = checkVelocityRange(servo5);
	s6 = checkVelocityRange(servo6);
	s7 = checkVelocityRange(servo7);

	/* Write serial command */
	char cmd[80];
	int  n= sprintf(cmd, "sav %u %u %u %u %u %u %u %u\r", s0, s1, s2, s3, s4, s5, s6, s7);
	write(fd, cmd, n);
}

/**
 * Sets the position for all servos.
 * params: One value per servo ranging from -1000 to 1000
 * or -2500 to 1900 if rangeOverride is true
 */
void ServoBoard::setAllPosition(int servo0, int servo1, int servo2, int servo3,
					int servo4, int servo5, int servo6, int servo7,
					bool rangeOverride) {

	int s0, s1, s2, s3, s4, s5, s6, s7;

	/* Check for range */
	if (rangeOverride) {
		s0 = checkPositionWithExtendedRange(servo0);
		s1 = checkPositionWithExtendedRange(servo1);
		s2 = checkPositionWithExtendedRange(servo2);
		s3 = checkPositionWithExtendedRange(servo3);
		s4 = checkPositionWithExtendedRange(servo4);
		s5 = checkPositionWithExtendedRange(servo5);
		s6 = checkPositionWithExtendedRange(servo6);
		s7 = checkPositionWithExtendedRange(servo7);
	} else {
		s0 = checkPositionRange(servo0);
		s1 = checkPositionRange(servo1);
		s2 = checkPositionRange(servo2);
		s3 = checkPositionRange(servo3);
		s4 = checkPositionRange(servo4);
		s5 = checkPositionRange(servo5);
		s6 = checkPositionRange(servo6);
		s7 = checkPositionRange(servo7);
	}

	/* Write serial command */
	char cmd[80];
	int n = sprintf(cmd, "sa %i %i %i %i %i %i %i %i\r", s0, s1, s2, s3, s4, s5, s6, s7);
	write(fd, cmd, n);
}

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
void ServoBoard::setServo(uint servo, int pos, uint speed, bool rangeOverride) {

	uint s;
	int p;

	/* Check parameters */
	if (servo < 8) {

		s = checkVelocityRange(speed);
		if(rangeOverride) {
			p = checkPositionWithExtendedRange(pos);
		} else {
			p = checkPositionRange(pos);
		}

		/* Write serial command */
		char cmd[80];
		int n = sprintf(cmd, "s%u %i %u\r", servo, p, s);
		write(fd, cmd, n);

	} else {
		perror("Servo Board: Trying to set undefined servo");
	}
}

/**
 * Sets the initial servo positions.
 * At power up the servos are moved to the specified position.
 * Positions are persistent across reboots.
 * params: One value per servo ranging from -1000 to 1000
 * or -2500 to 1900 if rangeOverride is true
 */
void ServoBoard::setInitialServoPositions(int servo0, int servo1, int servo2, int servo3,
						 int servo4, int servo5, int servo6, int servo7,
						 bool rangeOverride) {

	int s0, s1, s2, s3, s4, s5, s6, s7;

	/* Check for range */
	if (rangeOverride) {
		s0 = checkPositionWithExtendedRange(servo0);
		s1 = checkPositionWithExtendedRange(servo1);
		s2 = checkPositionWithExtendedRange(servo2);
		s3 = checkPositionWithExtendedRange(servo3);
		s4 = checkPositionWithExtendedRange(servo4);
		s5 = checkPositionWithExtendedRange(servo5);
		s6 = checkPositionWithExtendedRange(servo6);
		s7 = checkPositionWithExtendedRange(servo7);
	} else {
		s0 = checkPositionRange(servo0);
		s1 = checkPositionRange(servo1);
		s2 = checkPositionRange(servo2);
		s3 = checkPositionRange(servo3);
		s4 = checkPositionRange(servo4);
		s5 = checkPositionRange(servo5);
		s6 = checkPositionRange(servo6);
		s7 = checkPositionRange(servo7);
	}

	/* Write serial command */
	char cmd[80];
	int n = sprintf(cmd, "sia %i %i %i %i %i %i %i %i\r", s0, s1, s2, s3, s4, s5, s6, s7);
	write(fd, cmd, strlen(cmd));
}

/**
 * Enables servos
 * Default state is enabled
 */
void ServoBoard::enableServos() {

	write(fd, "se\r", 3);

}

/**
 * Disables servos
 * Default state is enabled
 */
void ServoBoard::disableServos() {

	write(fd, "sd\r", 3);

}