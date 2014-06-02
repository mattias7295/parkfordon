/*
 * adc.c
 *
 * Created: 2014-05-09 09:24:15
 *  Author: masc0058
 */ 

#include "adc.h"

/*
* Function: adc_init
* Input:	-
* Output:	-
* Description: Initialize the ADC.
*/
void adc_init() {
	
	/* Enable the ADC */
	ADCSRA |= _BV(ADEN);
}

/*
* Function: adc_read
* Input:	adcx: uint8_t - The pin to read from.
* Output:	uint8_t - The digital value converted from analogue.
* Description: Convert analogue to digital value from a port.
*/
uint8_t adc_read(uint8_t adcx) {
	
	ADMUX	&=	0xf0;
	ADMUX	|=	adcx;
	
	ADCSRA |= _BV(ADSC);
	
	while ((ADCSRA & _BV(ADSC)));
	
	ADMUX |= (1 << ADLAR);
	
	return ADCH;
}

