/*
 * GPSparser.c
 *
 * Created: 2014-05-13 14:34:42
 *  Author: masc0058
 */ 

#include "GPS_parser.h"

char latitude[9];
char longitude[10];

void initGPSParser(unsigned int ubrr) {
	
//	DDRD |= (1<<PD3);

	/* Set baud rate */
	UBRR1H = (unsigned char)(ubrr>>8);
	UBRR1L = (unsigned char)ubrr;

	/* Enable receiver and transmitter */
	UCSR1B = (1<<RXEN1)|(1<<TXEN1);

	/* Set frame format: 8data, 2stop bit */
	UCSR1C = (1<<USBS1)|(3<<UCSZ10);

}

void parseGPS() {
		
	char temp = 'O';
	char word[6];
	char sentence[37];
	char delim = ',';
	char *GPSStatus;
	
	while (strcmp(word, "GPRMC") != 0) {
		
		do {
			temp = USART_ReceiveGPS();
		} while (temp != '$');
		
		for (int i = 0; i < 5; i++) {
			word[i] = USART_ReceiveGPS();
		}
		
		word[5] = '\0';
	}
		
	for (int i = 0; i < 36; i++) {
		sentence[i] = USART_ReceiveGPS();
	}
	
	//sentence[36] = '\0';
	
	strtok(sentence, &delim);
	GPSStatus = strtok(NULL, &delim);
	
	if (*GPSStatus != 'V') {
		//strtok(sentence, NULL);
		strcpy(latitude, strtok(NULL, &delim));
		strtok(NULL, &delim);
		strcpy(longitude, strtok(NULL, &delim));
	}
}


unsigned char USART_ReceiveGPS(void) {
	
	/* Wait for data to be received */
	while (!(UCSR1A & (1<<RXC1)));
	
	/* Get and return received data from buffer */
	return UDR1;
}