/*
 * kontroll.c
 *
 * Created: 2014-05-09 09:20:57
 *  Author: masc0058
 */ 


#include "kontroll.h"



int main(void)
{
	/* Enable the ADC */
	ADCSRA |= _BV(ADEN);

	/*Set buttons as input*/
	DDRA &= ~_BV(SW1);
	DDRA &= ~_BV(SW2);
	
	/* Set the LED pin as an output. */
	DDRA |= _BV(LED_H);
	DDRA |= _BV(LED_V);
	DDRA |= _BV(LED_U);
	DDRA |= _BV(LED_N);
	
    while(1) {
		
		if ((PINA & _BV(SW1))){
			
			PORTA |= _BV(LED_H);
			PORTA |= _BV(LED_V);
		}
	    else if (adc_read(X_AXIS) > ADC_THRESHOLDH) {
			
		    PORTA |= _BV(LED_H);
	    } else if (adc_read(X_AXIS) < ADC_THRESHOLDL) {
			
		    PORTA |= _BV(LED_V);
	    }
		else{
			
			PORTA &= ~_BV(LED_H);
			PORTA &= ~_BV(LED_V);
		}			
	    
		
		if ((PINA & _BV(SW2))){
			
			PORTA |= _BV(LED_U);
			PORTA |= _BV(LED_N);
		}
		else if (adc_read(Y_AXIS) > ADC_THRESHOLDH) {
			
			PORTA |= _BV(LED_N);
		} else if (adc_read(Y_AXIS) < ADC_THRESHOLDL) {
			
			PORTA |= _BV(LED_U);
		}
		else{
			
			PORTA &= ~_BV(LED_U);
			PORTA &= ~_BV(LED_N);
		}			
						
	}    
}

uint16_t adc_read(uint8_t adcx) {
	/* adcx is the analog pin we want to use. ADMUX's first few bits are
	* the binary representations of the numbers of the pins so we can
	* just 'OR' the pin's number with ADMUX to select that pin.
	* We first zero the four bits by setting ADMUX equal to its higher
	* four bits. */
	//ADMUX &= 0xf0;
	ADMUX &= 0b01100000; 
	ADMUX |= adcx;

	/* This starts the conversion. */
	ADCSRA |= _BV(ADSC);

	/* This is an idle loop that just wait around until the conversion
	* is finished. It constantly checks ADCSRA's ADSC bit, which we just
	* set above, to see if it is still set. This bit is automatically
	* reset (zeroed) when the conversion is ready so if we do this in
	* a loop the loop will just go until the conversion is ready. */
	while ( (ADCSRA & _BV(ADSC)) );

	/* Finally, we return the converted value to the calling function. */
	return ADC;
}