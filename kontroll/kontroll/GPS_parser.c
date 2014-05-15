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
	
	char temp;
	char word[6];
	
	for (int j = 0; j < 10; j++) {
	
	while (temp != '$') {
		temp = USART_ReceiveGPS();
	}
	
	for (int i = 0; i < 5; i++) {
		word[i] = USART_ReceiveGPS();
	}
	
	word[5] = '\0';

	if (strcmp(word, "GPRMC") == 0) {
		
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
	}
	
	}	
	
	//USART_Transmit(USART_ReceiveGPS());
	
/*	char temp = 'O';
	char word[6];
	char sentence[37];
	char delim = ',';
	char *GPSStatus;
	
	
	while (temp != '$') {	
		temp = USART_ReceiveGPS();
	}
	
	for (int i = 0; i < 5; i++) {
		word[i] = USART_ReceiveGPS();
	}
	
	word[5] = '\0';
	
	if (strcmp(word, "GPRMC") == 0) {
		
		for (int i = 0; i < 36; i++) {
			sentence[i] = USART_ReceiveGPS();
		}
		
		
		strtok(sentence, &delim);
		GPSStatus = strtok(sentence, NULL);
		
		if (*GPSStatus != 'V') {
			strtok(sentence, NULL);
			strcpy(latitude, strtok(sentence, NULL));
			strtok(sentence, NULL);
			strcpy(longitude, strtok(sentence, NULL));
		}

	}
	*/
	/*
	latitude[0] = 'h';
	latitude[1] = 'e';
	latitude[2] = 'j';
	longitude[0] = 'd';
	longitude[1] = 'å';
	*/
}


unsigned char USART_ReceiveGPS(void) {
	
	/* Wait for data to be received */
	while (!(UCSR1A & (1<<RXC1)));
	
	/* Get and return received data from buffer */
	return UDR1;
}