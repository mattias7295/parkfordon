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


/*
* Function: initOnInterrupt
* Input:	-
* Output:	-
* Description:	Initializes interrupt for the on/off switch
*				on port INT1 for the low signal in order to
*				wake the MCU from sleep mode.
*/
void initOnInterrupt() {
	
	/* Disable the external interrupt on port INT1. */
	EIMSK &= ~(1<<ON_OFF_SWITCH);
	
	/* Low level generates interrupt. */
	EICRA &= ~(1<<ISC10)|(1<<ISC11);
	
	/* Enable the external interrupt on port INT1. */
	EIMSK |= (1<<ON_OFF_SWITCH);
}

/*
* Function: initOffInterrupt
* Input:	-
* Output:	-
* Description:	Initializes interrupt for the on/off switch
*				on port INT1 for the rising edge in order to
*				signal sleep.
*/
void initOffInterrupt() {
	
	/* Disable the external interrupt on port INT1. */
	EIMSK &= ~(1<<ON_OFF_SWITCH);
	
	/* Rising edge generates interrupt. */
	EICRA |= (1<<ISC10)|(1<<ISC11);
	
	/* Enable the external interrupt on port INT1. */
	EIMSK |= (1<<ON_OFF_SWITCH);
}

/*
* Function: initSteerInterrupt
* Input:	-
* Output:	-
* Description:	Initializes interrupt for the steer mode switch
*				on port INT2 for both rising and falling edge.
*/
void initSteerInterrupt() {
	
	/* Disable the external interrupt on port INT2. */
	EIMSK &= ~(1<<STEER_SWITCH);
	
	/* Any edge generates interrupt. */
	EICRA |= (1<<ISC20);
	EICRA &= ~(1<<ISC21);
	
	/* Enable the external interrupt on port INT2. */
	EIMSK |= (1<<STEER_SWITCH);
}

/*
* Function: ISR
* Input:	INT1_vect
* Output:	-
* Description:	Interrupt routine for an external interrupt
*				on port INT1, does nothing but waking the MCU
*				up after entering sleep mode and changing the
*				power mode flag correctly.
*/
ISR(INT1_vect) {
	
	/* Change power mode flag. */
	if (power == OFF) {
		power = ON;
	} else {
		power = OFF;
	}
}

/*
* Function: ISR
* Input:	INT2_vect
* Output:	-
* Description:	Interrupt routine for an external interrupt
*				on port INT2, does nothing but changing the
*				steering mode flag correctly and change the
*				mode of the led (indicating auto steer when
*				lit).
*/
ISR(INT2_vect) {
	
	/* Change steering mode flag. */
	if (steer == MAN) {
		steer = AUTO;
		PORTB |= _BV(STEER_CONTROL);
	} else {
		steer = MAN;
		PORTB &= ~_BV(STEER_CONTROL);
	}
	
}

