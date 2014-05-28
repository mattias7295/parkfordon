/*
 * fordon.c
 *
 * Created: 2014-05-09 09:20:07
 *  Author: masc0058
 */ 

#include "fordon.h"

void init();
void timer_init();
int parseBluetooth();
static void put_char(uint8_t c, FILE* stream);
void spin(double latPerson, double lonPerson);
int calcHeading();
double absDouble(double number);
double getDistance(double latPerson, double lonPerson);
ISR(TIMER1_OVF_vect);
//ISR(USART_RXC_vect);

static FILE mystdout = FDEV_SETUP_STREAM(put_char, NULL, _FDEV_SETUP_WRITE);

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
ISR(USART0_RX_vect)
{
	char receivedByte = UDR0;

	if (receivedByte == 5)
	{
		// Jump back to main by setting a boolean
		interruptCurrentLoop = true;
	}
	
}
*/

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

/*
Interrupt hela tiden så att vi inte hinner skriva ut?! Testa lägga en delay i controlpadkoden.

One other thing to keep in mind is that there are actually two different interrupt vectors associated with the USART transmitter:
1) A "UART Data Register Empty" (UDRE) interrupt will fire continually as long as there's space in the double-buffered UDR for additional characters to be added to the hardware queue.

2) A "UART Transmit Complete" (TXC) interrupt will fire once following the completion of the final queued character.
*/

int main(void) {
	
	// redirect stdout to put_char method
	stdout = &mystdout;
	
	init();
	
	//printf("Efter USART INIT första while\n");
	
	int x = 0;

	while(x < 5) {
	
		if(!(PINB & _BV(PB2))) {
			x++;
		} else {
			x = 0;
		}
		
		_delay_ms(4000);
	}
	
	//_delay_ms(8000);
	
	int steerData = 255;
	int turnOff = 0;
	
	int trashData;
	
	while(1) {
		
		//_delay_ms(1000);
		
		turnOff = USART_Receive();
		
		if (turnOff == 254) {
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
		
		//USART_Transmit(5); I funktionerna ist.
		
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

void init() {
	
	/* Initialize timer. */
	timer_init();
	
	/* Initialize USART. */
	USART_Init(51);
	
	/* Set global interrupt flag. */
	sei();
	
	/* ONE enable for the H-bridges. */
	DDRB |= (1<<PB0);
	PORTB |= (1<<PB0);
	
	/* Initialize PWM. */
	init_pwm();
	
	/* Initialize GPS parser. */
	setupGpsParser(51);
	
	/* Initialize ADC. */
	adc_init();
	
	PORTC |= (1<<PC0)|(1<<PC1); // Pull-ups till twi
	TWBR = 8; // twi clock frequency

	/* Wait for connection. */
//	while (!(PINB & _BV(PB1))){}
	
}

void timer_init() {
	
	// Setup 16-bit timer for acceleration
	OCR1A = 1600;
	TIMSK1 = (1<<TOIE1);
	TCNT1 = 0;
	TCCR1B = (1<<CS10);
	DDRD |= (1<<PD3);
}

int calcHeading() {

	double latPerson;
	double lonPerson;
	
	char latitude[10];
	char longitude[11];
	
	/* Parse the GPS values of the vehicle. */
	parseGPS();
	
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

	/* Constant test values. */
	
/*	lat = 63.820401;
	lon = 20.310892;
*/	
/*	latPerson = 63.820344;
	lonPerson = 20.311167;
*/
	printf("\n");

	printf("LatPerson: %lf \nLonPerson: %lf \n", latPerson, lonPerson);
	
	printf("LatVehicle: %lf \nLonVehicle: %lf \n", lat, lon);

	// Om vinkeln stämmer ungefär, kör framåt tills koordinaterna överenstämmer
	double dist = 0, tempDist = 0;
	
	/* Drive forward until the vehicle is within a two metre radius from the person,
	 * or until the distance between the two increases. */
	//do {
		//
		///* Save earlier distance in order to know whether or not we are getting closer to the person. */
		//tempDist = dist;
		//dist = checkDistance(latPerson,lonPerson);
		//
		///* Drive forward. */
		//TCCR0A = (1<<COM0A0)|(1<<COM0A1)|(1<<WGM00);
		//OCR0A = 70;
		//TCCR2A = (1<<COM2A0)|(1<<COM2A1)|(1<<WGM20);
		//OCR2A = 70;
		//
		//printf("Vinkel: %lf\n", ((double)compas_update())/10);
		//
		///* If we are not getting closer to the person, stop the engines and go back to main loop. */
		//if (tempDist > dist) {
			//OCR0A = 255;
			//OCR0B = 255;
			//OCR2A = 255;
			//OCR2B = 255;
			//dist = 0;
		//}
		//
	//} while(dist > 0.00004);
	
	printf("Vinkel: %lf\n", ((double)compas_update())/10);
	
	printf("\n");
	
	tempDist = dist;
	dist = getDistance(latPerson, lonPerson);
	
	/* If the vehicle is within a two metre radius from the person
	 * or the vehicle is getting further away from the person, stop. */
	if (dist <= 0.002 || tempDist > dist) {
		
		/* Stop. */
		OCR0A = 255;
		OCR0B = 255;
		OCR2A = 255;
		OCR2B = 255;
		
	} else {
		
		/* Räkna ut vilken riktning fordonet ska vända sig åt av argumenten */
		spin(latPerson, lonPerson);
		
		/* Drive forward. */
		TCCR0A = (1<<COM0A0)|(1<<COM0A1)|(1<<WGM00);
		OCR0A = 100; // 70 standard
		TCCR2A = (1<<COM2A0)|(1<<COM2A1)|(1<<WGM20);
		OCR2A = 100;
	}

	return 0;
	
}

void spin(double latPerson, double lonPerson) {
	
	/* Get the differences in X and Y position between the vehicle and person. */
//	double deltaX = getDistance(lat, lonPerson);
//	double deltaY = getDistance(latPerson, lon);
	
	/* Set the correct sign. */
/*	if (lonPerson < lon) {
		deltaX *= -1;
	}
	
	if (latPerson < lat) {
		deltaY *= -1;
	}
*/	
	/* Calculate the displacement of the car compared to the position 
	 * of the person in radians. Since the distance does not matter
	 * when calculating the angle, we use the degree values to measure
	 * change in x and y axes where longitude is x and latitude is y. */
	double displacement = atan2(latPerson - lat, lonPerson - lon);
//	double displacement = atan2(deltaY, deltaX);
	
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
		currentAngle += ((double) compas_update())/10;
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
			currentAngle += ((double) compas_update())/10;
		}
		
		currentAngle = currentAngle/10;
		
		printf("\n");
		
		printf("currentAngle: %lf\n", currentAngle);
		printf("wantedAngle: %lf\n", wantedAngle);
		printf("Displacement: %lf\n", displacement);
		
		/* Update angle difference variables. */
		diff = currentAngle - wantedAngle;
		absDiff = absDouble(currentAngle - wantedAngle);
		
		printf("diff: %lf\n", diff);
		printf("absDiff: %lf\n", absDiff);
		
		printf("\n");
			
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
	
	///* If the wanted angle is closest to the current angle if you turn counterwise, 
	 //* turn left until angles match. Else, turn right until they match. */
	//if ((diff > 0 && diff <= 180) || (diff < 0 && diff <= -180)) {
		//
		///* Turn left until current angle and wanted angle match
		 //* with a 10 degree accuracy. */
		//while (absDiff > 10 && absDiff < 350) {
			//
			//printf("Absdiff %lf \n" , absDiff);
			//
			//currentAngle = 0;
			//
			//for (int i = 0; i < 10; i++) {
				//currentAngle += ((double) compas_update())/10;
			//}
			//
			//currentAngle = currentAngle/10;
			//
			//printf("currentAngle: %lf\n", currentAngle);
			//printf("wantedAngle: %lf\n", wantedAngle);
			//printf("Displacement: %lf\n", displacement);
			//
			///*forwardRight = true;
			//prevSpeedR = 120;
			//
			//forwardLeft = false;
			//prevSpeedL = 120;*/
			//
			//TCCR0A = (1<<COM0A0)|(1<<COM0A1)|(1<<WGM00);
			//OCR0A = 255;
			//TCCR2A = (1<<COM2A0)|(1<<COM2A1)|(1<<WGM20);
			//OCR2A = 70;
			//
			////_delay_ms(1);
			//
			///* Turn a few degrees. */
			//for (int i = 0; i < 10; i++) {
				//_delay_ms(100);
			//}
			//
			//OCR0A = 255;
			//OCR0B = 255;
			//OCR2A = 255;
			//OCR2B = 255;
			//
			//
			///* Wait for the vehicle to stop properly in order to get better compass data. */
			//for (int i = 0; i < 30; i++) {
				//_delay_ms(100);
			//}
			//
			//absDiff = absDouble(currentAngle - wantedAngle);
			//
		//}
				//
	//} else {
		//
		///* Calculate the absolute difference between the angles. */
		//double absDiff = absDouble(currentAngle - wantedAngle);
		//
		///* Turn left until current angle and wanted angle match
		 //* with a 10 degree accuracy. */
		//while (absDiff > 10 && absDiff < 350) {
			//printf("Absdiff %lf \n" , absDiff);
			//if (interruptCurrentLoop)
			//{
				//printf("Interrupt\n");
				//break;
			//}
			//currentAngle = 0;
			//
			//for (int i = 0; i < 10; i++) {
				//currentAngle += ((double) compas_update())/10;
			//}
			//
			//currentAngle = currentAngle/10;
			//
			//printf("currentAngle: %lf\n", currentAngle);
			//printf("wantedAngle: %lf\n", wantedAngle);
			//printf("Displacement: %lf\n", displacement);
			//
			///*forwardRight = false;
			//prevSpeedR = 120;
			//
			//forwardLeft = true;
			//prevSpeedL = 120;*/
			////_delay_ms(1);
			//
			//TCCR0A = (1<<COM0A0)|(1<<COM0A1)|(1<<WGM00);
			//OCR0A = 70;
			//TCCR2A = (1<<COM2A0)|(1<<COM2A1)|(1<<WGM20);
			//OCR2A = 255;
			//
			///* Turn a few degrees. */
			//for (int i = 0; i < 10; i++) {
				//_delay_ms(100);
			//}
			//
			//OCR0A = 255;
			//OCR0B = 255;
			//OCR2A = 255;
			//OCR2B = 255;
			//
			///* Wait for the vehicle to stop properly in order to get better compass data. */
			//for (int i = 0; i < 30; i++) {
				//_delay_ms(100);
			//}
			//absDiff = absDouble(currentAngle - wantedAngle);	
		//}		
	//}
}

//Haversine formula
double getDistance(double latPerson, double lonPerson) {

//	parseGPS();
/*	lat = 63.820401;
	lon = 20.310892;
*/	

/*	double latPerson =  latPersonIn;
	double lonPerson = lonPersonIn;
	
	double dx, dy, dz;
	lonPerson -= lon;
	lonPerson *= TO_RAD, latPerson *= TO_RAD, lat *= TO_RAD;
	
	dz = sin(latPerson) - sin(lat);
	dx = cos(lonPerson) * cos(latPerson) - cos(lat);
	dy = sin(lonPerson) * cos(latPerson);
*/
	/* Return the distance between the vehicle and the person. */
//	return asin(sqrt(dx * dx + dy * dy + dz * dz) / 2) * 2 * R;

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


int parseBluetooth() {	
	uint8_t speed1 = 0;
	uint8_t speed2 = 0;
	
	unsigned char command;
	
	USART_Transmit(5);
	command = USART_Receive();
	
	printf("%d\n",command);
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
