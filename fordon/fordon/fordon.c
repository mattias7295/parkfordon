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

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

void parseBluetooth(unsigned char command);

int main(void)
{
	DDRB |= (1<<PB0);
	PORTB |= (1<<PB0);
	USART_Init(MYUBRR);
	init_pwm();
	//adc_init();
	PORTC |= (1<<PC0)|(1<<PC1);
    while(1)
    {
		parseBluetooth(USART_Receive());
		//compas_update();
		//_delay_ms(100);
        
    }
}

void parseBluetooth(unsigned char command) {

	uint8_t speed = 0;
	if (CHECK_BIT(command, 7))
	{
		speed = (command >> 4) & 0x7;
		if (speed == 0)
		{
			speed = 0xff;
		}
		else {
			speed = 255-(speed * 36);		
		}
		initEngineLeftForward((unsigned char)speed);
	}
	else {
		speed = (command >> 4) & 0x7;
		if (speed == 0)
		{
			speed = 0xff;
		}
		else {
			speed = 255-(speed * 36);
		}
		initEngineLeftBackward((unsigned char)speed);
	}
	if (CHECK_BIT(command, 3)) {
		speed = command & 0x7;
		if (speed == 0)
		{
			speed = 0xff;
		}
		else {
			speed = 255-(speed * 36);
		}
		initEngineRightForward((unsigned char)speed);
	}
	else {
		speed = command & 0x7;
		if (speed == 0)
		{
			speed = 0xff;
		}
		else {
			speed = 255-(speed * 36);
		}
		initEngineRightBackward((unsigned char)speed);
	}
}