/*
 * fordon.c
 *
 * Created: 2014-05-09 09:20:07
 *  Author: masc0058
 */ 

#include "fordon.h"

/* Global variables for the previous speeds of the left and right engines. */
uint8_t prevSpeedR;
uint8_t prevSpeedL;

/* Global variables for the soft steering. */
bool forwardRight;
bool forwardLeft;
bool doNotChangeDirection = false;

/* Global variables for the double decimal degree coordinates of the vehicle. */
double lat;
double lon;

/*
* Function: ISR timer interrupt routine.
* Input:	Timer interrupt vector
* Output:	-
* Description: Accelerate to the value we store in prevSpeedR/prevSpeedL.
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
	
	/* Boolean value for the steer mode of the vehicle. 
	 * If true, then we are in auto mode. */
	bool autoDrive;
	
	/* Initialize every component. */
	init();
	
	/* Wait for GPS fix. */
	int x = 0;
	
	while(x < 5) {
	
		if(!(PINB & _BV(PB2))) {
			x++;
		} else {
			x = 0;
		}
		
		_delay_ms(4000);
	}
	
	/* Variables for steer and power data. */
	int steerData = 255;
	int turnOff = 0;
	
	/* Temporary trash data. */
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
			automaticSteering();
			
		} else {
			
			/* Turn on timer for soft driving. */
			TCCR1B |= (1<<CS10);
			
			/* Drive in man mode. */
			manualSteering();
		}
		
		USART_Transmit(5);
		
    }

}

/*
* Function: init
* Input:	- 
* Output:	-
* Description:	Initializes timers, USART, PWM and GPS parser 
*				and sets the global interrupt flag.
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
	
	/* Pull-ups to TWI. */
	PORTC |= (1<<PC0)|(1<<PC1); 
	
	/* TWI clock frequency. */
	TWBR = 8; 
}

/*
* Function: timer_init
* Input:	- 
* Output:	- 
* Description: Initialize a 16 bit timer for acceleration.
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
* Function: automaticSteering
* Input:	-
* Output:	- 
* Description:	Performs one step of the automatic driving through calculating 
*				the current and wanted heading angles and turns the vehicle to 
*				the correct heading and driving forward, unless the vehicle is 
*				within a 2 meter radius from the control pad or is about to 
*				collide, in those cases it stops instead of driving forward.
*/
int automaticSteering() {

	/* Decimal degree values of the latitude and longitude of the person (control pad). */
	double latPerson;
	double lonPerson;
	
	/* String storage for the coordinates. */
	char latitude[10];
	char longitude[11];
	
	/* Parse the GPS values of the vehicle. */
	parseGPS();
	
	USART_Transmit(5);
	
	/* Get the latitude of the control pad. */
	for (int i = 0; i < 9; i++) {
		latitude[i] = USART_Receive();
	}
	
	USART_Transmit(2);
	
	/* Get the longitude of the control pad. */
	for (int i = 0; i < 10; i++) {
		longitude[i] = USART_Receive();
	}
	
	/* Prepare for conversion. */
	char degA[3];
	strncpy(degA, latitude, 2);
	degA[2] = '\0';

	char minA[8];
	strncpy(minA, latitude+2, 7);
	minA[7] = '\0';
	
	/* Convert the text value to double decimal degree value. */
	double d,e;
	d = strtod(degA,NULL);
	e = strtod(minA,NULL);
	latPerson = (d + e/60);
	
	/* Prepare for conversion. */
	strncpy(degA, longitude+1 ,2);
	degA[2] = '\0';

	strncpy(minA, longitude+3, 7);
	minA[7] = '\0';

	/* Convert the text value to double decimal degree value. */
	d = strtod(degA,NULL);
	e = strtod(minA,NULL);
	lonPerson = (d + e/60);

	double dist = 0, tempDist = 0;
	
	/* Save previous distance and get the next distance. */
	tempDist = dist;
	dist = getDistance(latPerson, lonPerson);
	
	/* If the vehicle is within a two meter radius from the person
	 * or the vehicle is getting further away from the person, stop. */
	if (dist <= 0.002 || tempDist > dist) {
		
		/* Stop. */
		OCR0A = 255;
		OCR0B = 255;
		OCR2A = 255;
		OCR2B = 255;
		
	} else {
		
		/* Calculate the heading and turn the vehicle */
		spin(latPerson, lonPerson);
		
		/* If the vehicle is about to collide, stop. Else, continue driving ahead. */
		if (adc_read(FORWARDADC)>20) {
			
			/* Stop. */
			TCCR0A = (1<<COM0A0)|(1<<COM0A1)|(1<<WGM00);
			OCR0A = 255;
			TCCR2A = (1<<COM2A0)|(1<<COM2A1)|(1<<WGM20);
			OCR2A = 255;
			
		} else {
		
			/* Drive forward. */
			TCCR0A = (1<<COM0A0)|(1<<COM0A1)|(1<<WGM00);
			OCR0A = 100;
			TCCR2A = (1<<COM2A0)|(1<<COM2A1)|(1<<WGM20);
			OCR2A = 100;
		}
		
	}

	return 0;
	
}
 
 
/*
* Function: spin
* Input:	latPerson: double - The latitude of the control pad.
*			lonPerson: double - The longitude of the control pad.
* Output:	-
* Description:	Calculates the current and wanted angles and turns 
*				the vehicle to the correct heading.
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
		
		/* Update angle difference variables. */
		diff = currentAngle - wantedAngle;
		absDiff = absDouble(currentAngle - wantedAngle);
		
		/* If the wanted angle is closest to the current angle 
		 * if you turn counterwise, turn left. */ 
		if ((diff > 0 && diff <= 180) || (diff < 0 && diff <= -180)) {
			
			/* Turn left. */
			TCCR0A = (1<<COM0A0)|(1<<COM0A1)|(1<<WGM00);
			OCR0A = 255;
			TCCR2A = (1<<COM2A0)|(1<<COM2A1)|(1<<WGM20);
			OCR2A = 60;
		
		} else {
		
			/* Turn right. */
			TCCR0A = (1<<COM0A0)|(1<<COM0A1)|(1<<WGM00);
			OCR0A = 60;
			TCCR2A = (1<<COM2A0)|(1<<COM2A1)|(1<<WGM20);
			OCR2A = 255;
		}
		
		/* Turn a few degrees. */
		for (int i = 0; i < 20; i++) {
			_delay_ms(100);
		}
			
	}
}

/*
* Function: getDistance
* Input:	latPerson: double - The person's (control pad's) latitude.
*			lonPerson: double - The person's (control pad's) longitude.
* Output:	double - The distance between the vehicle and the control pad.
* Description:	Uses the haversine formula for the distance between two points
*				on the shell of a sphere to get the distance between the vehicle
*				and the person with the control pad.
*/
double getDistance(double latPerson, double lonPerson) {

	/* Difference in latitude and longitude measured in radians. */
	double deltaLatRad = (latPerson - lat)*TO_RAD;
	double deltaLonRad = (lonPerson - lon)*TO_RAD;

	/* Temporary variable for the expression under the square root in the
	 * get-distance-form of the haversine formula. */
	double temp = sin(deltaLatRad/2) * sin(deltaLatRad/2) +
	cos(lat*TO_RAD) * cos(latPerson*TO_RAD) * sin(deltaLonRad/2) * sin(deltaLonRad/2);

	/* Return the distance in kilometers. */
	return 2 * R * asin(sqrt(temp));
}

/*
* Function: absDouble
* Input:	number: double - The floating point number we want to get the absolute value of.
* Output:	double -  The absolute value of the input number.
* Description: Returns the absolute value of a double.
*/
double absDouble(double number) {
	
	if (number < 0) {
		return number*(-1);
	} else {
		return number;
	}
	
}

/*
* Function: manualSteering
* Input:	-
* Output:	-
* Description:	This method is used for the manual steering. The vehicle 
*				receives a coded byte of data from the control pad and 
*				turns on the engines according to the data. The seventh bit
*				is the direction of the left engines (1 forward, 0 backwards),
*				bits 6-4 is the strength of the left engines and the low
*				nipple is interpreted in the same way but for the right engines.
*/
int manualSteering() {	
	uint8_t speed1 = 0;
	uint8_t speed2 = 0;
	
	unsigned char command;
	
	USART_Transmit(5);
	command = USART_Receive();
	
	speed1 = (command >> 4) & 0x7;
	
	if (speed1 == 0) {
		speed1 = 130;
		doNotChangeDirection = true;
	} else {
		doNotChangeDirection = false;
		speed1 = 130-(speed1 * 18);
	}
	
	speed2 = command & 0x7;
	
	if (speed2 == 0) {
		speed2 = 130;
		doNotChangeDirection = true;
	} else {
		doNotChangeDirection = false;
		speed2 = 130-(speed2 * 18);
	}
	
	if (CHECK_BIT(command,7)) {
		
		if (!doNotChangeDirection) {
			forwardRight = true;
		}
		
		if (adc_read(FORWARDADC)>20) {
			prevSpeedR = 130;
		} else {
			prevSpeedR = speed1;
		}
		
	} else {	
		
		if (!doNotChangeDirection) {
			forwardRight = false;
		}
		
		if (adc_read(BACKWARDADC)>20) {
			prevSpeedR = 130;
		} else {
			prevSpeedR = speed1;
		}
	}
	
	
	if (CHECK_BIT(command, 3)) {
	
		if (!doNotChangeDirection) {
			forwardLeft = true;
		}
		
		if (adc_read(FORWARDADC)>20) {
			prevSpeedL = 130;
		} else {
			prevSpeedL = speed2;
		}
		
	} else {

		if (!doNotChangeDirection) {
			forwardLeft = false;
		}
		
		if (adc_read(BACKWARDADC)>20) {
			prevSpeedL = 130;
		} else {
			prevSpeedL = speed2;
		}
		
	}
	
	return 0;
}
