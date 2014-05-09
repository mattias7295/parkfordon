/*
 * twi.c
 *
 * Created: 2014-05-09 12:43:18
 *  Author: johe0179
 */ 
#include<avr/io.h> 
#include "twi.h"
#include <util/delay.h>
#include "usart.h"

void compas_update()
{
	//Start
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
	
	
	//Send address
	TWDR = 0x42;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
	
	
	//Send Data
	TWDR = 0x41;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
	
	//Stop
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);	
	_delay_ms(70);
	
	//Start
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
	
	
	//Send address
	TWDR = 0x43;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
	
	//Read data
	TWCR = (1<<TWINT) | (1<<TWEA);
	uint8_t  high = TWDR;
	USART_Transmit(high);
	
	TWCR = (1<<TWINT);
	uint8_t low = TWDR;
	USART_Transmit(low);
	
	
	//Stop
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
}