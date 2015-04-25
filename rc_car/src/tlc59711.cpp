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
 #include "rc_car/tlc59711.h"

#ifdef STANDALONE
 #include <stdio.h>
 #include <unistd.h>
#endif

 #define PWM_LED_MAX 65535

 /**
  * Initialises the driver with the given number of drivers.
  * @param drivers The number of drivers chained together.
  */
 TLC59711::TLC59711(uint8_t drivers) {

 	this->drivers = drivers;

 	BCr = BCg = BCb = 0x7F;

 	this->pwmbuffer = (uint16_t *) calloc(2, 12 * drivers);
 	
 	bcm2835_init();

 	bcm2835_spi_begin();

 	bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, 0);
	bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS1, 0);

	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
 	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_128);
 	bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);

 	bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
 }

 TLC59711::~TLC59711() {
 	bcm2835_spi_end();
 	bcm2835_close();
 }

 /**
  * Sets the pwm value for the given monochrome led.
  * @param led The led to set.
  * @param pwmVal The pwm value for the given led.
  */
 void TLC59711::setLED(uint8_t led, uint16_t pwmVal) {
 	
 	if(led < 12 * this->drivers) {
 		this->pwmbuffer[led] = pwmVal;
 	}
 }

 /**
  * Sets the pwm value for the given monochrome led as percantage of its maximum.
  * @param led The led to set.
  * @param pwmVal The pwm percentage for the given led.
  */
 void TLC59711::setLED(uint8_t led, float pwmVal) {
 	setLED(led, (uint16_t)(PWM_LED_MAX * pwmVal));
 }

 /**
  * Sets the rgb Value for the given rgb led.
  * @param led The led to set.
  * @param rVal The r channel value.
  * @param gVal The g channel value.
  * @param bVal The b channel value.
  */
 void TLC59711::setRGB(uint8_t led, uint16_t rVal, uint16_t gVal, uint16_t bVal) {
 	setLED(led * 3, rVal);
 	setLED(led * 3 + 1, gVal);
 	setLED(led * 3 + 2, bVal);
 }

 /**
  * Writes the led settings to the driver
  */
 void TLC59711::writeValues() {

 	uint32_t cmd;

 	//Write command
 	cmd = 0x25;

 	cmd <<= 5;
	//OUTMG = 1, EXTGCK = 0, TMGRST = 1, DSPRPT = 1, BLANK = 0 -> 0x16
 	cmd |= 0x16;

 	cmd <<= 7;
 	cmd |= BCr;

 	cmd <<= 7;
 	cmd |= BCg;

 	cmd <<= 7;
 	cmd |= BCb;

 	for(uint8_t n = 0; n < this->drivers; n++) {
 		bcm2835_spi_transfer(cmd >> 24);
 		bcm2835_spi_transfer(cmd >> 16);
 		bcm2835_spi_transfer(cmd >> 8);
 		bcm2835_spi_transfer(cmd);

 		for (int8_t l = 11; l >= 0; l--) {
 			bcm2835_spi_transfer(this->pwmbuffer[n * 12 + l] >> 8);
 			bcm2835_spi_transfer(this->pwmbuffer[n * 12 + l]);
 		}
 	}
 }

#ifdef STANDALONE
int main() {
	
	printf("Testing LED driver\n");

	printf("...Init\n");
	TLC59711 instance = TLC59711(1);

	printf("...Setting Values\n");
	instance.setLED(0, 65535);
	instance.setLED(1, 65535);
	instance.setLED(2, 65535);
	instance.setLED(3, 65535);
	instance.setLED(4, 65535);
	instance.setLED(5, 65535);
	instance.setLED(6, 65535);
	instance.setLED(7, 65535);
	instance.setLED(8, 65535);
	instance.setLED(9, 65535);
	instance.setLED(10, 65535);
	instance.setLED(11, 65535);

	printf("...Writing Values\n");
	instance.writeValues();

	printf("...Waiting\n");
	usleep(1000000);

	printf("...Disabling\n");
	instance.setLED(0, 0);
	instance.setLED(1, 0);
	instance.setLED(2, 0);
	instance.setLED(3, 0);
	instance.setLED(4, 0);
	instance.setLED(5, 0);
	instance.setLED(6, 0);
	instance.setLED(7, 0);
	instance.setLED(8, 0);
	instance.setLED(9, 0);
	instance.setLED(10, 0);
	instance.setLED(11, 0);

	printf("...Writing Values\n");
	instance.writeValues();

	printf("...Done!\n");
}
#endif

