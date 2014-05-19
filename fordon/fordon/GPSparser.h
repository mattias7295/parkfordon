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


void setupGpsParser(unsigned int baud);
void parseGPS(double *lat, double *lon);
unsigned char USART_ReceiveGPS( void );

#endif /* GPSPARSER_H_ */