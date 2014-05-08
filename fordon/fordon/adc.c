/*
 * adc.c
 *
 * Created: 2014-05-08 20:11:51
 *  Author: Mattias
 */ 

#include "adc.h"

uint8_t adc_read(uint8_t adcx) {
	/* adcx is the analog pin we want to use. ADMUX's first few bits are
	* the binary representations of the numbers of the pins so we can
	* just 'OR' the pin's number with ADMUX to select that pin.
	* We first zero the four bits by setting ADMUX equal to its higher
	* four bits. */
	ADMUX	&=	0xf0;
	ADMUX	|=	adcx;
	
	/* This starts the conversion. */
	ADCSRA |= _BV(ADSC);
	
	/* This is an idle loop that just wait around until the conversion
	* is finished. It constantly checks ADCSRA's ADSC bit, which we just
	* set above, to see if it is still set. This bit is automatically
	* reset (zeroed) when the conversion is ready so if we do this in
	* a loop the loop will just go until the conversion is ready. */
	while ( (ADCSRA & _BV(ADSC)) );
	
	/* Finally, we return the converted value to the calling function. */
	ADMUX |= (1 << ADLAR);
	return ADCH;
}