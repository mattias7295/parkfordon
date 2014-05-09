/*
 * pwm.c
 *
 * Created: 2014-05-09 10:36:03
 *  Author: masc0058
 */ 

#include "pwm.h"

void init_pwm() {
	//turn on two 8-bit timers
	TCCR0B = (1<<CS00);
	TCCR2B = (1<<CS20);
	
	//set OC0A and 0C0B as outputs
	DDRB |= (1<<PB3) | (1<<PB4);
	
	//set OC2A and 0C2B as outputs
	DDRD |= (1<<PD6) | (1<<PD7);
}


void initEngineRightForward(unsigned char speed) {
	TCCR0A = (1<<COM0A0)|(1<<COM0A1)|(1<<WGM00);
	OCR0A = speed;
}

void initEngineRightBackward(unsigned char speed) {
	TCCR0A = (1<<COM0B0)|(1<<COM0B1)|(1<<WGM00);
	OCR0B = speed;
}

void initEngineLeftForward(unsigned char speed) {
	TCCR2A = (1<<COM2A0)|(1<<COM2A1)|(1<<WGM20);
	OCR2A = speed;
}

void initEngineLeftBackward(unsigned char speed) {
	TCCR2A = (1<<COM2B0)|(1<<COM2B1)|(1<<WGM20);
	OCR2B = speed;
}