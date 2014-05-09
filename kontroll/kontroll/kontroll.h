/*
 * kontroll.h
 *
 * Created: 2014-05-09 11:56:39
 *  Author: erbe0036
 */ 

#include <avr/io.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

/*Define axis for joystick*/
#define X_AXIS PA0
#define Y_AXIS PA1

/*Define testlights*/
#define LED_H PA7
#define LED_V PA6
#define LED_U PA5
#define LED_N PA4


/*Arbitrary thresholds*/
#define ADC_THRESHOLDH 768
#define ADC_THRESHOLDL 256

/*Define testbuttons*/
#define SW1 PA2
#define SW2 PA3

#ifndef KONTROLL_H_
#define KONTROLL_H_

#endif /* KONTROLL_H_ */

int main();
uint16_t adc_read(uint8_t adcx);