/*
 * GPSparser.c
 *
 * Created: 2014-05-13 14:34:42
 *  Author: masc0058
 */ 

#include "GPS_parser.h"

char latitude[10];
char longitude[11];

static void put_char(uint8_t c, FILE* stream);

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
	char word[6] = "";
	char sentence[45] = "";
	char delim = ',';
	char GPSStatus[2] = "";

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
	strncpy(GPSStatus,strtok(NULL, &delim),1);
	GPSStatus[1] = '\0';
	
	if (strcmp(GPSStatus,"V")) {
		strncpy(latitude, strtok(NULL, &delim),10);
		strtok(NULL, &delim);
		strncpy(longitude, strtok(NULL, &delim),11);
	} else {
		strncpy(latitude, "0000.0000", 9);
		strncpy(longitude, "00000.0000", 10);
	}
	
}

static void put_char(uint8_t c, FILE* stream)
{
	if (c == '\n') put_char('\r', stream);
	while(!(UCSR0A & (1 << UDRE0)));
	UDR0 = c;
}


unsigned char USART_ReceiveGPS(void) {
	
	/* Wait for data to be received */
	while (!(UCSR1A & (1<<RXC1)));
	
	/* Get and return received data from buffer */
	return UDR1;
}