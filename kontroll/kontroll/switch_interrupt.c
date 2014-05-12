/*
 * button_interrupt.c
 *
 * Created: 2014-05-12 13:05:27
 *  Author: anjo0409
 */ 

#include "switch_interrupt.h"

void switchInit() {
	
	/* Low level generates interrupt. */
	EICRA = (0<<ISC00)|(0<<ISC01);
	
	/* Enable the external interrupt on port INT0. */
	EIMSK = (1<<ON_OFF_SWITCH);
	
	/* Set the global interrupt flag. */
	sei();
}

/*
* Function: ISR
* Input:	INT0_vect
* Output:	-
* Description:	Interrupt routine for an external interrupt
*				on port INT0, does nothing but waking the MCU
*				up after entering sleep mode.
*/
ISR(INT0_vect) {
}
