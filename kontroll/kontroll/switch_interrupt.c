/*
 * switch_interrupt.c
 *
 * Created: 2014-05-12 13:05:27
 *  Author: anjo0409
 */ 

#include "switch_interrupt.h"

/* Global steer and power on/off flags. */
power_mode power;
steer_mode steer;

void initOnInterrupt() {
	
	/* Disable the external interrupt on port INT0. */
	EIMSK &= (0<<ON_OFF_SWITCH);
	
	/* Low level generates interrupt. */
	EICRA &= (0<<ISC00)|(0<<ISC01);
	
	/* Enable the external interrupt on port INT0. */
	EIMSK |= (1<<ON_OFF_SWITCH);
	
	/* Set the global interrupt flag. */
//	sei();
}

void initOffInterrupt() {
	
	/* Disable the external interrupt on port INT0. */
	EIMSK &= (0<<ON_OFF_SWITCH);
	
	/* Rising edge generates interrupt. */
	EICRA |= (1<<ISC00)|(1<<ISC01);
	
	/* Enable the external interrupt on port INT0. */
	EIMSK |= (1<<ON_OFF_SWITCH);
	
	/* Set the global interrupt flag. */
//	sei();
}

void initSteerInterrupt() {
	
	/* Disable the external interrupt on port INT1. */
	EIMSK &= (0<<STEER_SWITCH);
	
	/* Any edge generates interrupt. */
	EICRA |= (1<<ISC10);
	EICRA &= (0<<ISC11);
	
	/* Enable the external interrupt on port INT1. */
	EIMSK |= (1<<STEER_SWITCH);
	
	/* Set the global interrupt flag. */
//	sei();
}

/*
* Function: ISR
* Input:	INT0_vect
* Output:	-
* Description:	Interrupt routine for an external interrupt
*				on port INT0, does nothing but waking the MCU
*				up after entering sleep mode and changing the
*				power mode flag correctly.
*/
ISR(INT0_vect) {
	
	/* Change power mode flag. */
	if (power == OFF) {
		power = ON;
		PORTB |= _BV(POWER_CONTROL);
	} else {
		power = OFF;
		PORTB &= ~_BV(POWER_CONTROL);
	}
}

ISR(INT1_vect) {
	
	/* Change steering mode flag. */
	if (steer == MAN) {
		steer = AUTO;
		PORTB |= _BV(STEER_CONTROL);
	} else {
		steer = MAN;
		PORTB &= ~_BV(STEER_CONTROL);
	}
	
}
