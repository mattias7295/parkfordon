/*
 * spi.c
 *
 * Created: 2014-05-09 11:14:56
 *  Author: masc0058
 */ 

#include "spi.h"
#include <avr/io.h>

void SPI_MasterInit(void)
{
	/* Set MOSI and SCK output, all others input */
	DDRB = (1<<PB5)|(1<<PB7);
	PORTB |= (1<<PB5)|(1<<PB7);
	/* Enable SPI, Master, set clock rate fck/16 */
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
}
void SPI_MasterTransmit(char cData)
{
	/* Start transmission */
	SPDR = cData;
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)))	
	;
	cData = SPDR;
	
	//USART_Transmit(cData >> 7);
	//USART_Transmit(6);
}