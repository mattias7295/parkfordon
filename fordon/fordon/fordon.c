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
#define PI 3.1415926536
#define TO_RAD (PI / 180)
#define TO_DEG (180 / PI)

typedef int bool;
#define true 1
#define false 0

bool forwardRight;
bool forwardLeft;
bool doNotChangeDirection = false;


int parseBluetooth(unsigned char command);
static void put_char(uint8_t c, FILE* stream);
void spin(double latPerson, double lonPerson);
int calcHeading(unsigned char command);
static FILE mystdout = FDEV_SETUP_STREAM(put_char, NULL, _FDEV_SETUP_WRITE);
double absDouble(double number);

extern uint8_t prevSpeedR;
extern uint8_t prevSpeedL;
extern double lat;
extern double lon;
bool autoDrive;

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
	DDRD |= (1<<PD3);
	
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
	
	_delay_ms(8000);
	USART_Transmit(3);
	
	if(USART_Receive()==255)
	{
		autoDrive = true;
	}
	else
	{
		autoDrive = false;
	}		
		
    while(1)
    {
		USART_Transmit(1);
		unsigned char command = USART_Receive();
		if(command == 1)
		{
			if(autoDrive==true)
			{
				autoDrive = false;
			}
			else
			{
				autoDrive = true;
			}
		}
		else
		{
			if(autoDrive==true)
			{
				int x = 0;
				while(x<5)
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
				}
				calcHeading(command);
			}
			else
			{
				parseBluetooth(command);
			}
		}
	
    }

}

int calcHeading(unsigned char command) {
	double latPerson;
	double lonPerson;
	char latitude[10];
	char longitude[11];
	latitude[0] = command;
	for(int i = 1;i<9;i++) {
		latitude[i] = USART_Receive();
	}
	USART_Transmit(2);
	for(int i = 0;i<10;i++) {
		longitude[i] = USART_Receive();
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
	//printf("Lat: %lf \nLon: %lf \n", latPerson, lonPerson);
	
	parseGPS();

	
	//printf("LatV: %lf \nLonV: %lf \n", latitudeVehile, longitudeVehicle);
	
	// Räkna ut vilken riktning fordonet ska vända sig åt av argumenten
	
	// Om vinkeln stämmer ungefär, kör framåt tills koordinaterna överenstämmer
	while(!checkDistance(latPerson,lonPerson)){
		spin(latPerson, lonPerson);
		forwardRight = true;
		forwardLeft = true;
		prevSpeedL = 100;
		prevSpeedR = 100;
		USART_Transmit(4);
		for(int i = 0;i<9;i++) {
			latitude[i] = USART_Receive();
			if(latitude[i] == 1)
			{
				if(autoDrive)
				{
					autoDrive = false;
				}
				else
				{
					autoDrive = true;
				}
				return 1;
			}
		}
		USART_Transmit(4);
		for(int i = 0;i<10;i++) {
			longitude[i] = USART_Receive();
			if(longitude[i] == 1)
			{
				if(autoDrive)
				{
					autoDrive = false;
				}
				else
				{
					autoDrive = true;
				}
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
		_delay_ms(500);
		
	}
	
	return 0;
	
}

void spin(double latPerson, double lonPerson) {
	
	/* Calculate the displacement of the car compared to the position 
	 * of the person in radians. */
	double displacement = atan2(latPerson-lat, lonPerson-lon) * TO_DEG;
	
	/* Convert the radian angle value in the unit circle to a degree value 
	 * in the unit circle in [0, 360). */
	if (displacement >= 0) {
		displacement *= TO_DEG;
	} else {
		displacement = (2*PI + displacement) * TO_DEG; 
	}
	
	printf("Angle: %lf\n", displacement);
	
	/* The angle of the nose of the vehicle compared to north, that is,
	 * 0 means north, 90 is east, 180 is south and 270 west. */
	double currentAngle = ((double) compas_update())/10;
	
	/* Get new wanted angle, that is, the angle of the direction of the person. */
	double wantedAngle = currentAngle -  displacement;
	
	/* Convert the wanted angle to be in the interval [0, 360). */
	if (wantedAngle < 0) {
		wantedAngle += 360;
	}
	
	printf("wantedAngle: %lf\n", wantedAngle);
	
	if (displacement <= 180) {
		
		/* Turn left until current angle and wanted angle match
		 * with a 5 degree accuracy. */
		while (absDouble(currentAngle - wantedAngle) >= 5) {
			currentAngle = ((double) compas_update())/10;
			printf("currentAngle: %lf\n", currentAngle);
			
			forwardRight = true;
			prevSpeedR = 60;
			
			forwardLeft = false;
			prevSpeedL = 60;
		}
		
	} else {
		
		/* Turn right until current angle and wanted angle match
		 * with a 5 degree accuracy. */
		while (abs((int)(currentAngle - wantedAngle)) >= 5) {
			currentAngle = ((double) compas_update())/10;
			printf("currentAngle: %lf\n", currentAngle);
			
			forwardRight = false;
			prevSpeedR = 60;
			
			forwardLeft = true;
			prevSpeedL = 60;
		}
		
	}
}

//Haversine formula
int checkDistance(double latPerson, double lonPerson) {

	parseGPS();
	
	/*double dx, dy, dz;
	lonPerson -= lon;
	lonPerson *= TO_RAD, latPerson *= TO_RAD, lat *= TO_RAD;
	
	dz = sin(latPerson) - sin(lat);
	dx = cos(lonPerson) * cos(latPerson) - cos(lat);
	dy = sin(lonPerson) * cos(latPerson);
	return (asin(sqrt(dx * dx + dy * dy + dz * dz) / 2) * 2 * R)<0.00002;*/
	
	/* Calculate the distance in lat and long axes. */
	double dx = absDouble(lon - lonPerson);
	double dy = absDouble(lat - latPerson);
	
	/* Return whether or not the vehicle is within ~2 meters of the person. */
	return sqrt(dx*dx + dy*dy) < 0.00004;
}

double absDouble(double number) {
	
	if (number < 0) {
		return number*(-1);
	} else {
		return number;
	}
	
}

static void put_char(uint8_t c, FILE* stream)
{
	if (c == '\n') put_char('\r', stream);
	while(!(UCSR1A & (1 << UDRE1)));
	UDR1 = c;
}


int parseBluetooth(unsigned char command) {	
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
	return 0;
}
