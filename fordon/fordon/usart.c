/*
 * usart.c
 *
 * Created: 2014-05-09 09:23:55
 *  Author: masc0058
 */ 

#include "usart.h"

/*
* Function: USART_Init
* Input:	baud: unsigned int  - Value to put to the UBRR register.
* Output:	-
* Description: Initialize the UART for the bluetooth communication.
*/
void USART_Init(unsigned int baud) {
	
	/* Set the pin as output. */
	DDRD |= (1<<PD1);
	
	/* Set baud rate */
	UBRR0H = (unsigned char)(baud>>8);
	UBRR0L = (unsigned char)baud;
	
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

/*
* Function: USART_Transmit
* Input:	data: uint8_t - Data to send with bluetooth.
* Output:	-
* Description: Send a byte of data with bluetooth.
*/
void USART_Transmit(uint8_t data) {
	
	/* Wait for empty transmit buffer */
	while (!(UCSR0A & (1<<UDRE0)));
	
	/* Put data into buffer, sends the data. */
	UDR0 = data;
}

/*
* Function: USART_Receive
* Input:	-
* Output:	unsigned char - The received data.
* Description: Get and return received data from buffer.
*/
unsigned char USART_Receive(void) {
	
	/* Wait for data to be received */
	while (!(UCSR0A & (1<<RXC0)));
	
	/* Get and return received data from buffer */
	return UDR0;
}