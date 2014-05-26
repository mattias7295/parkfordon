/*
 * usart.h
 *
 * Created: 2014-05-09 09:23:45
 *  Author: masc0058
 */ 


#ifndef USART_H_
#define USART_H_

#include <avr/io.h>
#include <avr/interrupt.h>

#define MYUBRR 12

void USART_Init( unsigned int baud );
void USART_Transmit( uint8_t data );
unsigned char USART_Receive( void );



#endif /* USART_H_ */