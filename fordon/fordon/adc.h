/*
 * adc.h
 *
 * Created: 2014-05-09 09:24:07
 *  Author: masc0058
 */ 


#ifndef ADC_H_
#define ADC_H_

#include <avr/io.h>

#define ADC_PIN			5

void adc_init();

uint8_t adc_read(uint8_t adcx);



#endif /* ADC_H_ */