/*
 * GPSparser.h
 *
 * Created: 2014-05-13 14:34:26
 *  Author: masc0058
 */ 


#ifndef GPSPARSER_H_
#define GPSPARSER_H_

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>

/* Global variables for containment of the vehicle coordinates in double decimal degree form. */
extern double lat;
extern double lon;

/* Global variables for containment of the vehicle coordinates in text form. */
extern char latitude[10];
extern char longitude[11];

/* Function declarations. */
void setupGpsParser(unsigned int baud);
void parseGPS();
unsigned char USART_ReceiveGPS( void );

#endif /* GPSPARSER_H_ */