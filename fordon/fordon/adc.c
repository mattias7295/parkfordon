/*
 * adc.c
 *
 * Created: 2014-05-09 09:24:15
 *  Author: masc0058
 */ 

#include "adc.h"



void adc_init() {
	/* Enable the ADC */
	ADCSRA |= _BV(ADEN);
}

uint8_t adc_read(uint8_t adcx) {
	
	ADMUX	&=	0xf0;
	ADMUX	|=	adcx;
	
	
	ADCSRA |= _BV(ADSC);
	
	
	while ( (ADCSRA & _BV(ADSC)) );
	
	ADMUX |= (1 << ADLAR);
	return ADCH;
}
