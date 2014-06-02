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

/* Global variables for containment of the vehicle coordinates in double decimal degree form. */
double lat;
double lon;

/* Global variables for containment of the vehicle coordinates in text form. */
char latitude[10];
char longitude[11];

/*
* Function: setupGpsParser
* Input:	baud: unsigned int - The value to put in the UBBR register.
* Output:	-
* Description: Setup the second UART which communicates with the GPS.
*/
void setupGpsParser(unsigned int baud) {
	
	/* Set baud rate */
	UBRR1H = (unsigned char)(baud>>8);
	UBRR1L = (unsigned char)baud;
	
	/* Enable receiver and transmitter */
	UCSR1B = (1<<RXEN1)|(1<<TXEN1);
	
	/* Set frame format: 8data, 2stop bit */
	UCSR1C = (1<<USBS1)|(3<<UCSZ10);
}

/*
* Function: parseGPS
* Input:	-
* Output:	-
* Description: Read data from GPS, parse data and convert to doubles.
*/
void parseGPS() {

	/* Char containers. */	
	char temp = 'O';
	char word[6] = "";
	char sentence[45] = "";
	char delim = ',';
	char GPSStatus[2] = "";

	/* Find the GPRMC data string. */
	while (strcmp(word, "GPRMC") != 0) {
	
		do {
			temp = USART_ReceiveGPS();
		} while (temp != '$');
	
		for (int i = 0; i < 5; i++) {
			word[i] = USART_ReceiveGPS();
		}
		
		word[5] = '\0';
	}
	
	/* Read the rest of the string to a temporary container. */
	for (int i = 0; i < 45; i++) {
		sentence[i] = USART_ReceiveGPS();
	}
	
	sentence[44] = '\0';
	
	/* Get the status of the GPS data.*/
	strtok(sentence, &delim);
	strncpy(GPSStatus, strtok(NULL, &delim), 1);
	GPSStatus[1] = '\0';
	
	/* When the status is "A" the data is ok, and we read it.
	 * Else, the data is ignored and we set the values to 0. */
	if (strcmp(GPSStatus,"V")) {
		
		/* Read text representation of latitude and longitude in degree minute format. */
		strncpy(latitude, strtok(NULL, &delim), 10);
		strtok(NULL, &delim);
		strncpy(longitude, strtok(NULL, &delim), 11);
		
		/* Parse latitude data to prepare for conversion. */
		char degA[3];
		strncpy(degA, latitude,2);
		degA[2] = '\0';

		char minA[8];
		strncpy(minA,latitude+2,7);
		minA[7] = '\0';
		
		/* Convert latitude from text to double in decimal degree format. */
		double d,e;
		d = strtod(degA,NULL);
		e = strtod(minA,NULL);
		lat = (d + e/60);
		
		/* Parse longitude data to prepare for conversion. */
		strncpy(degA,longitude+1,2);
		degA[2] = '\0';
		strncpy(minA, longitude+3,7);
		minA[7] = '\0';
		d = strtod(degA,NULL);
		e = strtod(minA,NULL);
		
		/* Convert longitude from text to double in decimal degree format. */
		lon = (d + e/60);
	
	} else {
		
		/* Default values in case the GPS loses its fix. */
		lat = 0;
		lon = 0;
	}
	
}

/*
* Function:	USART_ReceiveGPS
* Input:	-
* Output:	unsigned char - Received GPS data.
* Description: Get and return received data from buffer.
*/
unsigned char USART_ReceiveGPS(void) {
	
	/* Wait for data to be received */
	while (!(UCSR1A & (1<<RXC1)));
	
	/* Get and return received data from buffer */
	return UDR1;
}

