/*
 * fordon.c
 *
 * Created: 2014-05-09 09:20:07
 *  Author: masc0058
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include "usart.h"
#include "adc.h"
#include "pwm.h"
#include "spi.h"
#include "twi.h"
#include "GPSparser.h"
//#include "autodrive.h"

#define FORWARDADC	1
#define BACKWARDADC 0
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
#define R 6371
#define TO_RAD (3.1415926536 / 180)

typedef int bool;
#define true 1
#define false 0

bool forwardRight;
bool forwardLeft;
bool doNotChangeDirection = false;

double latitudeVehile;
double longitudeVehicle;

void parseBluetooth(unsigned char command);
static void put_char(uint8_t c, FILE* stream);
void spin(double latPerson, double lonPerson);
static FILE mystdout = FDEV_SETUP_STREAM(put_char, NULL, _FDEV_SETUP_WRITE);

extern uint8_t prevSpeedR;
extern uint8_t prevSpeedL;

ISR(TIMER1_OVF_vect)
{
	if(forwardRight)
	{
		if (OCR0B == 255)
		{
			if (OCR0A == 255)
			{
				OCR0A = 130;
			}
			if (OCR0A > prevSpeedR)
			{
				OCR0A--;
			}
			else if (OCR0A < prevSpeedR)
			{
				OCR0A++;
			}
			else if (OCR0A == prevSpeedR && prevSpeedR == 130)
			{
				OCR0A = 255;
			}
			initEngineRightForward();
		}		
		else
		{
			if (OCR0B == 130)
			{
				OCR0B = 255;
			}
			else
			{
				OCR0B++;	
			}
		}

	}
	else
	{
		if (OCR0A == 255)
		{
			if (OCR0B == 255)
			{
				OCR0B = 130;
			}
			if (OCR0B > prevSpeedR)
			{
				OCR0B--;
			}
			else if (OCR0B < prevSpeedR)
			{
				OCR0B++;
			}
			else if (OCR0B == prevSpeedR && prevSpeedR == 130)
			{
				OCR0B = 255;
			}
			initEngineRightBackward();
		}
		else
		{
			if (OCR0A == 130)
			{
				OCR0A = 255;
			}
			else
			{
				OCR0A++;	
			}
		}		
	}
	
	if(forwardLeft)
	{
		if (OCR2B == 255)
		{
			if (OCR2A == 255)
			{
				OCR2A = 130;
			}
			if (OCR2A > prevSpeedL)
			{
				OCR2A--;
			}
			else if (OCR2A < prevSpeedL)
			{
				OCR2A++;
			}
			else if (OCR2A == prevSpeedL && prevSpeedL == 130)
			{
				OCR2A = 255;
			}
			initEngineLeftForward();
		}
		else
		{
			if (OCR2B == 130)
			{
				OCR2B = 255;
			}
			else
			{
				OCR2B++;	
			}
		}		
	}
	else
	{
		if (OCR2A == 255)
		{
			if (OCR2B == 255)
			{
				OCR2B = 130;
			}
			if (OCR2B > prevSpeedL)
			{
				OCR2B--;
			}
			else if (OCR2B < prevSpeedL)
			{
				OCR2B++;
			}
			else if (OCR2B == prevSpeedL && prevSpeedL == 130)
			{
				OCR2B = 255;
			}
			initEngineLeftBackward();
		}
		else
		{
			if (OCR2A == 130)
			{
				OCR2A = 255;
			}
			else
			{
				OCR2A++;
			}
		}		
	}
}

int main(void)
{
	stdout = &mystdout;
	// Setup 16-bit timer for acceleration
	OCR1A = 1600;
	TIMSK1 = (1<<TOIE1);
	TCNT1 = 0;
	TCCR1B = (1<<CS10); // no prescale
	sei();
	
	
	DDRB |= (1<<PB0); // EN enable till H-bryggorna
	PORTB |= (1<<PB0); 

	USART_Init(51);
	init_pwm();
	setupGpsParser(51);
	adc_init();
	PORTC |= (1<<PC0)|(1<<PC1); // Pull-ups till twi
	TWBR = 8; // twi clock frequency

	while (!(PINB & _BV(PB1)))
	{
		
	}
	int x = 0;
	/*while(x<5)
	{
		if(!(PINB & _BV(PB2)))
		{
			x++;
		}
		else
		{
			x = 0;
		}
		_delay_ms(4000);
	}*/
	_delay_ms(8000);
    while(1)
    {
		USART_Transmit(0xff);
		parseBluetooth(USART_Receive());
		//printf("Kompass:%d\n",compas_update());
		// Wait for fix
		//calcHeading();

		_delay_ms(8000);
    }

}

void calcHeading() {
	double latPerson;
	double lonPerson;
	char latitude[10];
	char longitude[11];
	USART_Transmit(0xFF);
	for(int i = 0;i<10;i++) {
		latitude[i] = USART_Receive();
		if(latitude[i] == 0x00) {
			return 1;
		}
	}
	USART_Transmit(0xFF);
	for(int i = 0;i<11;i++) {
		longitude[i] = USART_Receive();
		if(longitude[i] == 0x00) {
			return 1;
		}
	}
	
	char degA[3];
	strncpy(degA, latitude,2);
	degA[2] = '\0';

	char minA[8];
	strncpy(minA,latitude+2,7);
	minA[7] = '\0';


	
	double d,e;
	d = strtod(degA,NULL);
	e = strtod(minA,NULL);
	latPerson = (d + e/60);
	
	strncpy(degA,longitude+1,2);
	degA[2] = '\0';
	strncpy(minA, longitude+3,7);
	minA[7] = '\0';
	d = strtod(degA,NULL);
	e = strtod(minA,NULL);
	lonPerson = (d + e/60);
	printf("Lat: %lf \nLon: %lf \n", latPerson, lonPerson);
	/*parseGPS();
	latPerson = 63.820374;
	lonPerson = 20.308906;


	
	// Räkna ut vilken riktning fordonet ska vända sig åt av argumenten
	
	// Om vinkeln stämmer ungefär, kör framåt tills koordinaterna överenstämmer
	while(!checkDistance(latPerson,lonPerson)){
		spin(latPerson, lonPerson);
		forwardRight = true;
		forwardLeft = true;
		prevSpeedL = 100;
		prevSpeedR = 100;
		_delay_ms(500);
		
	}*/
	
}
void spin(double latPerson, double lonPerson) 
{
	int angle = atan2( lonPerson-lon, latPerson-lat);

	// Om cos av vinkeln mellan riktningarna är positiv: vrid åt höger, Annars vänster
	if(cos(angle)<0) {
		// Läs av riktning
		double dir = (double)compas_update();
		dir = dir/10;
		//Vänd åt vänster
		while(!(abs(dir-angle)<10 || 360 - abs(dir-angle)<10)) {
			printf("calcheading\n");
			dir = (double)compas_update();
			dir = dir/10;
			forwardRight = false;
			prevSpeedR = 60;
			forwardLeft = true;
			prevSpeedL = 60;
			//_delay_ms(500);
		}
		
	} else {
		double dir = (double)compas_update();
		dir = dir/10;
		//Vänd åt höger
		while(!(abs(dir-angle)<10 || 360 - abs(dir-angle)<10)) {
			dir = (double)compas_update();
			dir = dir/10;
			forwardRight = true;
			prevSpeedR = 60;
			forwardLeft = false;
			prevSpeedL = 60;
			
			//_delay_ms(500);
		}
		
	}
}
//Haversine formula
int checkDistance(double latPerson, double lonPerson) {

	parseGPS();
	
	double dx, dy, dz;
	latPerson -= lon;
	latPerson *= TO_RAD, lonPerson *= TO_RAD, lon *= TO_RAD;
	
	dz = sin(lonPerson) - sin(lon);
	dx = cos(latPerson) * cos(lonPerson) - cos(lon);
	dy = sin(latPerson) * cos(lonPerson);
	return (asin(sqrt(dx * dx + dy * dy + dz * dz) / 2) * 2 * R)<0.00002;
}

static void put_char(uint8_t c, FILE* stream)
{
	if (c == '\n') put_char('\r', stream);
	while(!(UCSR0A & (1 << UDRE0)));
	UDR0 = c;
}


void parseBluetooth(unsigned char command) {
	// Change to auto
	if(command==0){
		USART_Transmit(0xff);
		if(USART_Receive()==0xff){
			USART_Transmit(0xff);
			if(USART_Receive() == 0) {
				calcHeading();
			}
		}
		return;
	}
	
	
	uint8_t speed1 = 0;
	uint8_t speed2 = 0;
	speed1 = (command >> 4) & 0x7;
		if (speed1 == 0)	{
			speed1 = 130;
			doNotChangeDirection = true;
		}
		else {
			doNotChangeDirection = false;
			speed1 = 130-(speed1 * 18);
		}
		
		speed2 = command & 0x7;
		if (speed2 == 0)	{
			speed2 = 130;
			doNotChangeDirection = true;
		}
		else {
			doNotChangeDirection = false;
			speed2 = 130-(speed2 * 18);
		}
		
	if(CHECK_BIT(command,7)){
		if (!doNotChangeDirection)
		{
			forwardRight = true;
		}
		if(adc_read(FORWARDADC)>20)
		{
			prevSpeedR = 130;
		}
		else
		{
			prevSpeedR = speed1;
		}
	}else {	
		if (!doNotChangeDirection)
		{
			forwardRight = false;
		}
		if(adc_read(BACKWARDADC)>20)
		{
			prevSpeedR = 130;
		}
		else
		{
			prevSpeedR = speed1;
		}
	}
	
	
	if (CHECK_BIT(command, 3)) {
		if (!doNotChangeDirection)
		{
			forwardLeft = true;
		}
		if(adc_read(FORWARDADC)>20)
		{
			prevSpeedL = 130;
		}
		else
		{
			prevSpeedL = speed2;
		}
		
	}else {

		if (!doNotChangeDirection)
		{
			forwardLeft = false;
		}
		if(adc_read(BACKWARDADC)>20)
		{
			prevSpeedL = 130;
		}
		else
		{
			prevSpeedL = speed2;
		}
	}
}
