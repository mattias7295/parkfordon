/*
 * pwm.c
 *
 * Created: 2014-05-09 10:36:03
 *  Author: masc0058
 */ 

#include "pwm.h"

uint8_t prevSpeedR = 130;
uint8_t prevSpeedL = 130;

/*
* Function: init_pwm
* Input: -
* Output: -
* Description: init two 8-bit timers and set some outputs
*/
void init_pwm() {
	
	/* Turn on two 8-bit timers. */
	TCCR0B = (1<<CS00);
	TCCR2B = (1<<CS20);
	
	/* Set OC0A and 0C0B as outputs. */
	DDRB |= (1<<PB3) | (1<<PB4);
	
	/* Set OC2A and 0C2B as outputs. */
	DDRD |= (1<<PD6) | (1<<PD7);
	
	OCR0A = 255;
	OCR0B = 255;
	OCR2A = 255;
	OCR2B = 255;
}

/* Use OCROA to compare with the timer counter */
void initEngineRightForward() {
	TCCR0A = (1<<COM0A0)|(1<<COM0A1)|(1<<WGM00);
}

/* Use OCROB to compare with the timer counter */
void initEngineRightBackward() {
	TCCR0A = (1<<COM0B0)|(1<<COM0B1)|(1<<WGM00);
}

/* Use OCR2A to compare with the timer counter */
void initEngineLeftForward() {
	TCCR2A = (1<<COM2A0)|(1<<COM2A1)|(1<<WGM20);
}

/* Use OCR2B to compare with the timer counter */
void initEngineLeftBackward() {
	TCCR2A = (1<<COM2B0)|(1<<COM2B1)|(1<<WGM20);
}