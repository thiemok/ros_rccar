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

/**
 * Interface to the Adafruit TLC59711 PWM LED Driver for the raspberry pi
 * @see http://www.adafruit.com/datasheets/tlc59711.pdf
 */
 #pragma once

 #include <bcm2835.h>
 #include <stdlib.h>

 class TLC59711 {

 public:

 	/**
 	 * Initialises the driver with the given number of drivers.
 	 * @param drivers The number of drivers chained together.
	 */
 	TLC59711(uint8_t drivers);

 	~TLC59711();

 	/**
 	 * Sets the pwm value for the given monochrome led.
 	 * @param led The led to set.
 	 * @param pwmVal The pwm value for the given led.
 	 */
 	void setLED(uint8_t led, uint16_t pwmVal);

 	/**
 	 * Sets the pwm value for the given monochrome led as percantage of its maximum.
 	 * @param led The led to set.
 	 * @param pwmVal The pwm percentage for the given led.
 	 */
 	void setLED(uint8_t led, float pwmVal);

 	/**
 	 * Sets the rgb Value for the given rgb led.
 	 * @param led The led to set.
 	 * @param rVal The r channel value.
 	 * @param gVal The g channel value.
 	 * @param bVal The b channel value.
 	 */
 	void setRGB(uint8_t led, uint16_t rVal, uint16_t gVal, uint16_t bVal);

 	/**
 	 * Writes the led settings to the driver
 	 */
 	void writeValues();

 private:

 	uint16_t* pwmbuffer;

 	uint8_t BCr;
 	uint8_t BCg;
 	uint8_t BCb;

 	uint8_t drivers;

 };