/*
 * fordon.c
 *
 * Created: 2014-05-09 09:20:07
 *  Author: masc0058
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "usart.h"
#include "adc.h"
#include "pwm.h"
#include "spi.h"
#include "twi.h"
#include "GPSparser.h"

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
typedef int bool;
#define true 1
#define false 0

bool directionRight;
bool directionLeft;
bool doNotChangeDirection = false;

void parseBluetooth(unsigned char command);

uint8_t prevSpeedR = 255;
uint8_t prevSpeedL = 255;

ISR(TIMER1_OVF_vect)
{
	if(directionRight)
	{
		if (OCR0B == 255)
		{
		if (OCR0A > prevSpeedR)
		{
			OCR0A--;
		}
		else if (OCR0A < prevSpeedR)
		{
			OCR0A++;
		}
		initEngineRightForward();
		}		
		else
		{
			OCR0B++;
		}

	}
	else
	{
		if (OCR0A == 255)
		{
		if (OCR0B > prevSpeedR)
		{
			OCR0B--;
		}
		else if (OCR0B < prevSpeedR)
		{
			OCR0B++;
		}
		initEngineRightBackward();
		}
		else
		{
			OCR0A++;
		}		
	}
	
	if(directionLeft)
	{
		if (OCR2B == 255)
		{
		if (OCR2A > prevSpeedL)
		{
			OCR2A--;
		}
		else if (OCR2A < prevSpeedL)
		{
			OCR2A++;
		}
		initEngineLeftForward();
		}
		else
		{
			OCR2B++;
		}		
	}
	else
	{
		if (OCR2A == 255)
		{
		if (OCR2B > prevSpeedL)
		{
			OCR2B--;
		}
		else if (OCR2B < prevSpeedL)
		{
			OCR2B++;
		}
		initEngineLeftBackward();
		}
		else
		{
			OCR2A++;
		}		
	}
}

int main(void)
{
	OCR1A = 1600;
	TIMSK1 = (1<<TOIE1);
	TCNT1 = 0;
	TCCR1B = (1<<CS10); // 64 prescale
	sei();
	DDRB |= (1<<PB0);
	PORTB |= (1<<PB0);
	USART_Init(51);
	init_pwm();
	//setupGpsParser(51);
	//adc_init();
	PORTC |= (1<<PC0)|(1<<PC1);
    while(1)
    {
		//parseGPS();
		//_delay_ms(1000);
		parseBluetooth(USART_Receive());
		//compas_update();
		//_delay_ms(100);
        
    }
}


void parseBluetooth(unsigned char command) {

	uint8_t speed1 = 0;
	uint8_t speed2 = 0;
	speed1 = (command >> 4) & 0x7;
		if (speed1 == 0 || speed1 == 1)	{
			speed1 = 0xff;
			doNotChangeDirection = true;
		}
		else {
			doNotChangeDirection = false;
			speed1 = 255-(speed1 * 36);
		}
		
		speed2 = command & 0x7;
		if (speed2 == 0 || speed2 == 1)	{
			speed2 = 0xff;
			doNotChangeDirection = true;
		}
		else {
			doNotChangeDirection = false;
			speed2 = 255-(speed2 * 36);
		}
		
	if(CHECK_BIT(command,7)){
		//initEngineRightForward((unsigned char)speed1);
		if (!doNotChangeDirection)
		{
			directionRight = true;
		}
		prevSpeedR = speed1;
	}else {	
		//initEngineRightBackward((unsigned char)speed1);
		if (!doNotChangeDirection)
		{
			directionRight = false;
		}
		prevSpeedR = speed1;
	}
	
	
	if (CHECK_BIT(command, 3)) {
		//initEngineLeftForward((unsigned char)speed2);
		if (!doNotChangeDirection)
		{
			directionLeft = true;
		}
		prevSpeedL = speed2;
	}else {

		//initEngineLeftBackward((unsigned char)speed2);
		if (!doNotChangeDirection)
		{
			directionLeft = false;
		}
		prevSpeedL = speed2;
	}
}