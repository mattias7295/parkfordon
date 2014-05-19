/*
 * adc.c
 *
 * Created: 2014-05-09 09:24:15
 *  Author: masc0058
 */ 

#include "adc.h"

static void put_char(uint8_t c, FILE* stream);
static FILE mystdout = FDEV_SETUP_STREAM(put_char, NULL, _FDEV_SETUP_WRITE);

void adc_init() {
	stdout = &mystdout;
	/* Enable the ADC */
	ADCSRA |= _BV(ADEN);
	printf("testing %d\n\r", i);
}

uint8_t adc_read(uint8_t adcx) {
	
	ADMUX	&=	0xf0;
	ADMUX	|=	adcx;
	
	
	ADCSRA |= _BV(ADSC);
	
	
	while ( (ADCSRA & _BV(ADSC)) );
	
	ADMUX |= (1 << ADLAR);
	return ADCH;
}

static void put_char(uint8_t c, FILE* stream)
{
	if (c == '\n') put_char('\r', stream);
	while(!(UCSR0A & (1 << UDRE0)));
	UDR0 = c;
}