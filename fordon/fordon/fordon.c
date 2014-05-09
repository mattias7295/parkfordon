/*
 * fordon.c
 *
 * Created: 2014-05-09 09:20:07
 *  Author: masc0058
 */ 


#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"
#include "adc.h"
#include "pwm.h"
#include "spi.h"
#include "twi.h"
int main(void)
{
	USART_Init(MYUBRR);
	adc_init();
	char cdata;
	twi_init();
    while(1)
    {
		compas_update();
		_delay_ms(100);
        
    }
}