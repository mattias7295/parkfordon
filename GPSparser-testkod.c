/*
 * GPSparser.c
 *
 * Created: 2014-05-13 14:34:42
 *  Author: masc0058
 */ 

#include "GPSparser.h"
#include "usart.h"
#include <string.h>

void setupGpsParser(unsigned int baud)
{
	DDRD |= (1<<PD3);
	/* Set baud rate */
	UBRR1H = (unsigned char)(baud>>8);
	UBRR1L = (unsigned char)baud;
	/* Enable receiver and transmitter */
	UCSR1B = (1<<RXEN1)|(1<<TXEN1);
	/* Set frame format: 8data, 2stop bit */
	UCSR1C = (1<<USBS1)|(3<<UCSZ10);

}

void parseGPS()
{
	/*char temp;
	char word[6];
	while (temp != '$')
	{
		temp = USART_ReceiveGPS();
	}
	for (int i = 0; i < 5; i++)
	{
		word[i] = USART_ReceiveGPS();
	}
	word[5] = '\0';
	if (strcmp(word,"GPRMC") == 0)
	{
		USART_Transmit(word[0]);
		USART_Transmit(word[1]);
		USART_Transmit(word[2]);
		USART_Transmit(word[3]);
		USART_Transmit(word[4]);
		
		for (int i = 0; i < 12; i++)
		{
			USART_ReceiveGPS();
		}
		for (int i = 0; i < 24; i++)
		{
			USART_Transmit(USART_ReceiveGPS());
		}
		
		USART_Transmit(0x0D);
	}*/
	//USART_Transmit(USART_ReceiveGPS());
	
}


unsigned char USART_ReceiveGPS( void )
{
	/* Wait for data to be received */
	while ( !(UCSR1A & (1<<RXC1)) )
	;
	/* Get and return received data from buffer */
	return UDR1;
}