/*
 * GPS_parser.h
 *
 * Parser for the GPS data.
 *
 * Created: 2014-05-15 09:33:23
 *  Author: anjo0409
 */ 

#include <avr/io.h>
#include <string.h>
#include <stdio.h>

/* Global containers for text representation of latitude and longitude. */
extern char latitude[10];
extern char longitude[11];

/* Function declarations. */
void initGPSParser(unsigned int ubrr);
void parseGPS();
unsigned char USART_ReceiveGPS(void);