/*
 * adc.h
 *
 * Created: 2014-05-08 20:11:36
 *  Author: Mattias
 */ 

#include <avr/io.h>

#ifndef ADC_H_
#define ADC_H_

#define ADC_PIN			5

uint8_t adc_read(uint8_t adcx);

#endif /* ADC_H_ */