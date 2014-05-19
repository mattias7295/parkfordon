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

void parseGPS(double *lat, double *lon)
{
	char temp = 'O';
	char word[6] = "";
	char sentence[45];
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
	for (int i = 0; i < 45; i++) {
		sentence[i] = USART_ReceiveGPS();
		printf("%c",sentence[i]);
	}
	
	sentence[44] = '\0';
	
	strtok(sentence, &delim);
	GPSStatus = strtok(NULL, &delim);
	
	if (*GPSStatus != 'V') {
		strncpy(latitude,strtok(NULL, &delim),10);
		strtok(NULL, &delim);
		strncpy(longitude,strtok(NULL, &delim),11);
	}
	printf("\n\nLatitude:%s\n", latitude);
	printf("Longitude:%s", longitude);
	char degA[3];
	degA[0] = latitude[0];
	degA[1] = latitude[1];
	degA[2] = '\0';

	double deg = atof(degA);
	printf("%d\n",(int)deg);
	char minA[8];
	minA[0] = latitude[2];
	minA[1] = latitude[3];
	minA[2] = latitude[4];
	minA[3] = latitude[5];
	minA[4] = latitude[6];
	minA[5] = latitude[7];
	minA[6] = latitude[8];
	minA[7] = '\0';
	printf("\nminA:%s\n",minA);
	double minutes = atof(minA);
	*lat = deg + minutes/60;
	printf("cphora2\n");
	char degO[4];
	degO[0] = longitude[0];
	degO[1] = longitude[1];
	degO[2] = longitude[2];
	degO[3] = '\0';
	printf("\ndegO:%s\n",degO);
	deg  = atof(degO);
	printf("cphora3\n");
	char minO[8];
	minO[0] = longitude[3];
	minO[1] = longitude[4];
	minO[2] = longitude[5]; 
	minO[3] = longitude[6];
	minO[4] = longitude[7];
	minO[5] = longitude[8];
	minO[6] = longitude[9];
	minO[7] = '\0';

	minutes = atof(minO);
	*lon = deg + minutes/60;
printf("cphora4\n");
	
}


unsigned char USART_ReceiveGPS( void )
{
	/* Wait for data to be received */
	while ( !(UCSR1A & (1<<RXC1)) )
	;
	/* Get and return received data from buffer */
	return UDR1;
}

