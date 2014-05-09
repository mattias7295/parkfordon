/*
 * twi.c
 *
 * Created: 2014-05-09 12:43:18
 *  Author: johe0179
 */ 
#include<avr/io.h> 
#include "twi.h"
void twi_init()
{
	TWBR=72;
}

void compas_update()
{
	TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWSTA);
	while(!((TWCR)&(1<<TWINT)));
	
	TWDR=0x42; 
	
	TWCR=(1<<TWINT)|(1<<TWEN);
	while(!((TWCR)&(1<<TWINT)));
	
	TWDR=0x41;
	TWCR=(1<<TWINT)|(1<<TWEN);
	while(!((TWCR)&(1<<TWINT)));
	USART_Transmit(TWDR);
}