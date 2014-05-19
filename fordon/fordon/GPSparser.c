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
	//DDRD |= (1<<PD3);
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
	char temp = 'O';
	char word[6];
	char sentence[45];
	char delim = ',';
	char *GPSStatus;
	char *latitude;
	char *longitude;
	while (strcmp(word, "GPRMC") != 0) {
		
		do {
			temp = USART_ReceiveGPS();
		} while (temp != '$');
		
		for (int i = 0; i < 5; i++) {
			word[i] = USART_ReceiveGPS();
		}
		
		word[5] = '\0';
	}
	
	for (int i = 0; i < 45; i++) {
		sentence[i] = USART_ReceiveGPS();
	}
	
	sentence[44] = '\0';
	
	strtok(sentence, &delim);
	GPSStatus = strtok(NULL, &delim);
	
	if (*GPSStatus != 'V') {
		latitude = strtok(NULL, &delim);
		strtok(NULL, &delim);
		longitude = strtok(NULL, &delim);
	}
}


unsigned char USART_ReceiveGPS( void )
{
	/* Wait for data to be received */
	while ( !(UCSR1A & (1<<RXC1)) )
	;
	/* Get and return received data from buffer */
	return UDR1;
}

