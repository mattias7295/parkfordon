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

int main(void)
{
	USART_Init(MYUBRR);
	SPI_MasterInit();
	adc_init();
    while(1)
    {
		USART_Transmit();
		_delay_ms(100);
        
    }
}