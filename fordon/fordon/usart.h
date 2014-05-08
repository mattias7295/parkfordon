/*
 * usart.h
 *
 * Created: 2014-05-08 20:03:15
 *  Author: Mattias
 */ 

#include <avr/io.h>

#ifndef USART_H_
#define USART_H_

#define MYUBRR 12

void USART_Init( unsigned int baud );
void USART_Transmit( unsigned char data );
unsigned char USART_Receive();


#endif /* USART_H_ */