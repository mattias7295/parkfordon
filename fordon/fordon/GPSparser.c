/*
 * GPSparser.c
 *
 * Created: 2014-05-13 14:34:42
 *  Author: masc0058
 */ 

#include "GPSparser.h"
#include "usart.h"
#include <string.h>
#include "util/delay.h"
static void put_char(uint8_t c, FILE* stream);

char latitude[10];
char longitude[11];

double lat;
double lon;

void setupGpsParser(unsigned int baud)
{
	//stdout = &mystdout;
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
		//printf("%c",sentence[i]);
	}
	
	sentence[44] = '\0';
	
	strtok(sentence, &delim);
	strncpy(GPSStatus,strtok(NULL, &delim),1);
	GPSStatus[1] = '\0';
	
	if (strcmp(GPSStatus,"V")) {
		strncpy(latitude, strtok(NULL, &delim), 10);
		strtok(NULL, &delim);
		strncpy(longitude, strtok(NULL, &delim), 11);

		//printf("\n\nLatitude:%s\n", latitude);
		//printf("Longitude:%s\n", longitude);
	
		char degA[3];
		strncpy(degA, latitude,2);
		degA[2] = '\0';

		char minA[8];
		strncpy(minA,latitude+2,7);
		minA[7] = '\0';

		double d,e;
		d = strtod(degA,NULL);
		e = strtod(minA,NULL);
		//printf("degA:%lf minA:%lf\n",d,e);
		lat = (d + e/60);
	
		strncpy(degA,longitude+1,2);
		degA[2] = '\0';
		strncpy(minA, longitude+3,7);
		minA[7] = '\0';
		d = strtod(degA,NULL);
		e = strtod(minA,NULL);
		//printf("degO:%lf minO:%lf\n",d,e);
		lon = (d + e/60);
	
	} else {
		lat = 0;
		lon = 0;
	}
	
}

static void put_char(uint8_t c, FILE* stream)
{
	if (c == '\n') put_char('\r', stream);
	while(!(UCSR0A & (1 << UDRE0)));
	UDR0 = c;
}


unsigned char USART_ReceiveGPS( void )
{
	/* Wait for data to be received */
	while ( !(UCSR1A & (1<<RXC1)) )
	;
	/* Get and return received data from buffer */
	return UDR1;
}

