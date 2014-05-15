/*
 * GPS_parser.h
 *
 * Created: 2014-05-15 09:33:23
 *  Author: anjo0409
 */ 

#include <avr/io.h>
#include <string.h>

extern char latitude[9];
extern char longitude[10];

void initGPSParser(unsigned int ubrr);
void parseGPS();
unsigned char USART_ReceiveGPS(void);