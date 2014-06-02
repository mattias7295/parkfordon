/*
 * fordon.c
 *
 * Created: 2014-05-09 09:20:07
 *  Author: masc0058
 */ 

#include "fordon.h"

bool forwardRight;
bool forwardLeft;
bool doNotChangeDirection = false;

uint8_t prevSpeedR;
uint8_t prevSpeedL;
double lat;
double lon;
bool autoDrive;
bool interruptCurrentLoop = false;

/*
* Function: ISR timer interrupt routin
* Input: Timer interrupt vector
* Output: -
* Description: accelerate to the value we store in prevSpeedR/prevSpeedL
*/
ISR(TIMER1_OVF_vect)
{
	/* Should we go forward on the engines on the right side */
	if(forwardRight)
	{
		/* Stop going backward before we can go forward */
		if (OCR0B == 255)
		{
			/* Skip to ~50% duty cycle because 0%-49% duty cycle
			 is not enough*/
			if (OCR0A == 255)
			{
				OCR0A = 130;
			}
			if (OCR0A > prevSpeedR)
			{
				// decrease speed
				OCR0A--;
			}
			else if (OCR0A < prevSpeedR)
			{
				// increase speed
				OCR0A++;
			}
			else if (OCR0A == prevSpeedR && prevSpeedR == 130)
			{
				// Set back to 255 when we stop.
				OCR0A = 255;
			}
			initEngineRightForward();
		}		
		else
		{
			// decrease backward speed before we can go forward.
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


int main(void) {
	// Init every component
	init();
	
	// Wait for GPS fix 
	int x = 0;
	while(x < 5) {
	
		if(!(PINB & _BV(PB2))) {
			x++;
		} else {
			x = 0;
		}
		
		_delay_ms(4000);
	}
	
	int steerData = 255;
	int turnOff = 0;
	
	int trashData;
	
	while(1) {
		turnOff = USART_Receive();
		if (turnOff == 254) {
			// Turn of vehicle and wait for data from controller.
			USART_Transmit(5);
			trashData = USART_Receive();
		}
		
		USART_Transmit(5);
		
		steerData = USART_Receive();
		if (steerData == 0) {
			autoDrive = false;
		} else if (steerData == 1) {
			autoDrive = true;
		}
		
		if(autoDrive == true) {
			
			/* Turn off timer. */
			TCCR1B = 0;
			
			/* Drive in auto mode. */
			calcHeading();
			
		} else {
			
			/* Turn on timer. */
			TCCR1B |= (1<<CS10);
			
			/* Drive in man mode. */
			parseBluetooth();
		}
		
		USART_Transmit(5);
		
    }

}
/*
* Function: init
* Input: - 
* Output: -
* Description: Initializes timers, usart, pwm, gpsparser,
* setting global interrupt flag.
*/
void init() {
	
	/* Initialize timer. */
	timer_init();
	
	/* Initialize USART. */
	USART_Init(51);
	
	/* Set global interrupt flag. */
	sei();
	
	/* EN enable for the H-bridges. */
	DDRB |= (1<<PB0);
	PORTB |= (1<<PB0);
	
	/* Initialize PWM. */
	init_pwm();
	
	/* Initialize GPS parser. */
	setupGpsParser(51);
	
	/* Initialize ADC. */
	adc_init();
	
	/* Pull-ups till twi. */
	PORTC |= (1<<PC0)|(1<<PC1); 
	/ twi clock frequency
	TWBR = 8; 
}

/*
* Function: timer_init
* Input: - 
* Output: - 
* Description: initiate a 16 bit timer for acceleration
*/
void timer_init() {
	
	// Setup 16-bit timer for acceleration
	OCR1A = 1600;
	TIMSK1 = (1<<TOIE1);
	TCNT1 = 0;
	TCCR1B = (1<<CS10);
	DDRD |= (1<<PD3);
}

/*
* Function: calcHeading
* Input: -
* Output: - 
* Description: Calculates the heading, turning the vehicle to the right heading,
* drive forward until the vehicle is 4 meters from the controller.
*/
int calcHeading() {

	double latPerson;
	double lonPerson;
	
	char latitude[10];
	char longitude[11];
	/* Get the controllers coordinates */
	USART_Transmit(5);
	
	for(int i = 0; i < 9; i++) {
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
	/* Make the string to doubles */
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

	/* Parse the vehicle gps data */
	parseGPS();
	
	/* Calculate the heading and turn the vehicle */
	spin(latPerson, lonPerson);

	double dist = 0, tempDist = 0;
	
	
	dist = checkDistance(latPerson,lonPerson);
	tempDist = dist;
	
	/* If the vehicle is within a two metre radius from the person
	 *  or the vehicle is getting further away from the person, stop. */
	if (dist <= 0.00004 || tempDist > dist) {
		
		/* Stop. */
		OCR0A = 255;
		OCR0B = 255;
		OCR2A = 255;
		OCR2B = 255;
		
	} else {
		
		/* Drive forward. */
		TCCR0A = (1<<COM0A0)|(1<<COM0A1)|(1<<WGM00);
		OCR0A = 100; // 70 standard
		TCCR2A = (1<<COM2A0)|(1<<COM2A1)|(1<<WGM20);
		OCR2A = 100;
	}

	return 0;
	
}
 
 
/*
* Function: spin
* Input: latPerson: double - latitude of the controller
*		 lonPerson: double - longitude of the controller.
* Output: -
* Description: Calculate the heading and turn the vehicle.
*/
void spin(double latPerson, double lonPerson) {
	
	/* Calculate the displacement of the car compared to the position 
	 * of the person in radians. */
	double displacement = atan2(latPerson - lat, lonPerson - lon);
	
	/* Convert the radian angle value in the unit circle to a degree value 
	 * in the unit circle in [0, 360). */
	if (displacement >= 0) {
		displacement *= TO_DEG;
	} else {
		displacement = (2*PI + displacement) * TO_DEG; 
	}
	
	/* The angle of the nose of the vehicle compared to north, that is,
	 * 0 means north, 90 is east, 180 is south and 270 west. */
	double currentAngle = 0;
	
	for (int i = 0; i < 10; i++) {
		currentAngle += ((double) compass_update())/10;
	}
	
	currentAngle = currentAngle/10;
	
	/* Get new wanted angle, that is, the angle of the direction of the person. */
	double wantedAngle = 90 -  displacement;
	
	/* Convert the wanted angle to be in the interval [0, 360). */
	if (wantedAngle < 0) {
		wantedAngle += 360;
	}
	
	/* Calculate the difference between the angles (counterwise, neg. value indicates that 
	 * the current angle is on the low side of zero and that the wanted angle is on the high side). */
	double diff = currentAngle - wantedAngle;
	
	/* Calculate the absolute difference between the angles. */
	double absDiff = absDouble(currentAngle - wantedAngle);
	
	/* If the angle is not within a 20 degree range from the wanted
	 * angle, turn some. */
	while (absDiff > 10 && absDiff < 350) {

		
		OCR0A = 255;
		OCR0B = 255;
		OCR2A = 255;
		OCR2B = 255;
		
		/* Wait for the vehicle to stop properly in order to get better compass data. */
		for (int i = 0; i < 10; i++) {
			_delay_ms(100);
		}
		
		currentAngle = 0;
		
		for (int i = 0; i < 10; i++) {
			currentAngle += ((double) compass_update())/10;
		}
		
		currentAngle = currentAngle/10;
		
		/* If the wanted angle is closest to the current angle 
		 * if you turn counterwise, turn left. */ 
		if ((diff > 0 && diff <= 180) || (diff < 0 && diff <= -180)) {
			
			/* Turn left. */
			TCCR0A = (1<<COM0A0)|(1<<COM0A1)|(1<<WGM00);
			OCR0A = 255;
			TCCR2A = (1<<COM2A0)|(1<<COM2A1)|(1<<WGM20);
			OCR2A = 70;
		
		} else {
		
			/* Turn right. */
			TCCR0A = (1<<COM0A0)|(1<<COM0A1)|(1<<WGM00);
			OCR0A = 70;
			TCCR2A = (1<<COM2A0)|(1<<COM2A1)|(1<<WGM20);
			OCR2A = 255;
		}
		
		/* Turn a few degrees. */
		for (int i = 0; i < 20; i++) {
			_delay_ms(100);
		}

		absDiff = absDouble(currentAngle - wantedAngle);
			
	}
}

/*
* Function: checkDistance
* Input: latPersonIn: double - the persons(controllers)latitude.
*		 lonPersonIn: double - the persons(controllers)longitude.
* Output: double - return the distance between the vehicle and the controller.
* Description:
*/
double checkDistance(double latPersonIn, double lonPersonIn) {

	parseGPS();
	
	double latPerson =  latPersonIn;
	double lonPerson = lonPersonIn;
	
	/* latPersonIn and lonPersonIn are in degrees we 
	need to convert them to radians */
	double dx, dy, dz;
	lonPerson -= lon;
	lonPerson *= TO_RAD, latPerson *= TO_RAD, lat *= TO_RAD;
	
	dz = sin(latPerson) - sin(lat);
	dx = cos(lonPerson) * cos(latPerson) - cos(lat);
	dy = sin(lonPerson) * cos(latPerson);
	
	/* Return the distance between the vehicle and the person. */
	return asin(sqrt(dx * dx + dy * dy + dz * dz) / 2) * 2 * R;
}

/*
* Function: absDouble
* Input: number: double - the floating number we want to get the absolute value
* Output: number: double the absolute value of number.
* Description: returns the absolute value of a double.
*/
double absDouble(double number) {
	
	if (number < 0) {
		return number*(-1);
	} else {
		return number;
	}
	
}

/*
* Function: parseBluetooth
* Input: -
* Output: -
* Description: This method is used with the manual steering. The vehicle 
* receive a coded byte of data from the controller and turn on the engine
* according to the data. 
*/
int parseBluetooth() {	
	uint8_t speed1 = 0;
	uint8_t speed2 = 0;
	
	unsigned char command;
	
	USART_Transmit(5);
	command = USART_Receive();
	
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
