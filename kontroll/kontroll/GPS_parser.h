/*
 * GPS_parser.h
 *
 * Created: 2014-05-15 09:33:23
 *  Author: anjo0409
 */ 

#include <avr/io.h>
#include <string.h>
#include <stdio.h>

extern char latitude[10];
extern char longitude[11];

void initGPSParser(unsigned int ubrr);
void parseGPS();
unsigned char USART_ReceiveGPS(void);