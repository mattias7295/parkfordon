/*
 * fordon.c
 *
 * Created: 2014-05-09 09:20:07
 *  Author: masc0058
 */ 

#include "fordon.h"

int parseBluetooth(unsigned char command);
static void put_char(uint8_t c, FILE* stream);
void spin(double latPerson, double lonPerson);
int calcHeading(unsigned char command);
double absDouble(double number);
double checkDistance(double latPersonIn, double lonPersonIn);
ISR(TIMER1_OVF_vect);
ISR(USART_RXC_vect);

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
int globalint = 0;

ISR(USART0_RX_vect)
{
	char receivedByte = UDR0;
	globalint++;
	
	if (receivedByte == 5)
	{
		// Jump back to main by setting a boolean
		interruptCurrentLoop = true;
	}
	
}

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

int main(void) 
{
	// redirect stdout to put_char method
	stdout = &mystdout;
	// Setup 16-bit timer for acceleration
	OCR1A = 1600;
	TIMSK1 = (1<<TOIE1);
	TCNT1 = 0;
	TCCR1B = (1<<CS10); // no prescale TESTA TA BORT TIMERN OCH TESTA MER, KAN STACKA 2 
	DDRD |= (1<<PD3);
	
	USART_Init(51);
	/* Set global interrupt flag. */
	sei();
	
	DDRB |= (1<<PB0); // EN enable till H-bryggorna
	PORTB |= (1<<PB0); 
	
	init_pwm();
	setupGpsParser(51);
	
	adc_init();
	PORTC |= (1<<PC0)|(1<<PC1); // Pull-ups till twi
	TWBR = 8; // twi clock frequency

	
	
	//while (!(PINB & _BV(PB1))){}
	
	printf("Efter USART INIT första while\n");
	
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
	
//	_delay_ms(8000);
//	USART_Transmit(3);
	
	/*if(USART_Receive()==255)
	{
		autoDrive = true;
	}
	else
	{
		autoDrive = false;
	}	*/
		autoDrive = false;
	
	while(1) {
		
		printf("GLOBALINT: %d\n", globalint);
		_delay_ms(1000);
		
		//interruptCurrentLoop = false;
		//// RX interrupt disable
		////UCSR0B &= ~(1<<RXCIE0);
		////sei();
		//
		//USART_Transmit(1);
		//unsigned char command = USART_Receive();
		//if(command == 1)
		//{
			//if(autoDrive==true)
			//{
				//autoDrive = false;
			//}
			//else
			//{
				//autoDrive = true;
			//}
		//}
		//else
		//{
			//if(autoDrive==true)
			//{
				//TCCR1B = 0;
				//calcHeading(command);
			//}
			//else
			//{
				//TCCR1B = (1<<CS10);
				//parseBluetooth(command);
			//}
		//}
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

	/*Set RX interrupt enable*/
	UCSR0B |= (1<<RXCIE0);
	sei();
	
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
	
	parseGPS();
	lat = 63.820401;
	lon = 20.310892;
	
	latPerson = 63.820344;
	lonPerson = 20.311167;

	printf("LatV: %lf \nLonV: %lf \n", lat, lon);
	
	// Räkna ut vilken riktning fordonet ska vända sig åt av argumenten
	spin(latPerson, lonPerson);

	OCR0A = 255;
	OCR0B = 255;
	OCR2A = 255;
	OCR2B = 255;

	// Om vinkeln stämmer ungefär, kör framåt tills koordinaterna överenstämmer
	double dist = 0, tempDist = 0;
	
	/* Drive forward until the vehicle is within a two metre radius from the person,
	 * or until the distance between the two increases. */
	do {
		if (interruptCurrentLoop)
		{
			// Jump out
			printf("Interrupt\n");
			break;
		}
		/* Save earlier distance in order to know whether or not we are getting closer to the person. */
		tempDist = dist;
		dist = checkDistance(latPerson,lonPerson);
		
		/* Drive forward. */
		TCCR0A = (1<<COM0A0)|(1<<COM0A1)|(1<<WGM00);
		OCR0A = 70;
		TCCR2A = (1<<COM2A0)|(1<<COM2A1)|(1<<WGM20);
		OCR2A = 70;
		
		printf("Vinkel: %lf\n", ((double)compas_update())/10);
		
		/* If we are not getting closer to the person, stop the engines and go back to main loop. */
		if (tempDist > dist) {
			OCR0A = 255;
			OCR0B = 255;
			OCR2A = 255;
			OCR2B = 255;
			dist = 0;
		}
		
	} while(dist > 0.00004);

	return 0;
	
}

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
	
	/* If the wanted angle is closest to the current angle if you turn counterwise, 
	 * turn left until angles match. Else, turn right until they match. */
	if ((diff > 0 && diff <= 180) || (diff < 0 && diff <= -180)) {
		
		/* Calculate the absolute difference between the angles. */
		double absDiff = absDouble(currentAngle - wantedAngle);
		
		/* Turn left until current angle and wanted angle match
		 * with a 10 degree accuracy. */
		while (absDiff > 10 && absDiff < 350) {
			printf("Absdiff %lf \n" , absDiff);
			if (interruptCurrentLoop)
			{
				printf("Interrupt\n");
				break;
			}
			currentAngle = 0;
			
			for (int i = 0; i < 10; i++) {
				currentAngle += ((double) compas_update())/10;
			}
			
			currentAngle = currentAngle/10;
			
			printf("currentAngle: %lf\n", currentAngle);
			printf("wantedAngle: %lf\n", wantedAngle);
			printf("Displacement: %lf\n", displacement);
			
			/*forwardRight = true;
			prevSpeedR = 120;
			
			forwardLeft = false;
			prevSpeedL = 120;*/
			
			TCCR0A = (1<<COM0A0)|(1<<COM0A1)|(1<<WGM00);
			OCR0A = 70;
			TCCR2A = (1<<COM2A0)|(1<<COM2A1)|(1<<WGM20);
			OCR2A = 255;
			//_delay_ms(1);
			
			
			/* Turn a few degrees. */
			for (int i = 0; i < 10; i++) {
				_delay_ms(100);
			}
			
			OCR0A = 255;
			OCR0B = 255;
			OCR2A = 255;
			OCR2B = 255;
			
			
			/* Wait for the vehicle to stop properly in order to get better compass data. */
			for (int i = 0; i < 30; i++) {
				_delay_ms(100);
			}
			absDiff = absDouble(currentAngle - wantedAngle);
			
		}
				
	} else {
		
		/* Calculate the absolute difference between the angles. */
		double absDiff = absDouble(currentAngle - wantedAngle);
		
		/* Turn left until current angle and wanted angle match
		 * with a 10 degree accuracy. */
		while (absDiff > 10 && absDiff < 350) {
			printf("Absdiff %lf \n" , absDiff);
			if (interruptCurrentLoop)
			{
				printf("Interrupt\n");
				break;
			}
			currentAngle = 0;
			
			for (int i = 0; i < 10; i++) {
				currentAngle += ((double) compas_update())/10;
			}
			
			currentAngle = currentAngle/10;
			
			printf("currentAngle: %lf\n", currentAngle);
			printf("wantedAngle: %lf\n", wantedAngle);
			printf("Displacement: %lf\n", displacement);
			
			/*forwardRight = false;
			prevSpeedR = 120;
			
			forwardLeft = true;
			prevSpeedL = 120;*/
			//_delay_ms(1);
			
			TCCR0A = (1<<COM0A0)|(1<<COM0A1)|(1<<WGM00);
			OCR0A = 255;
			TCCR2A = (1<<COM2A0)|(1<<COM2A1)|(1<<WGM20);
			OCR2A = 70;
			
			/* Turn a few degrees. */
			for (int i = 0; i < 10; i++) {
				_delay_ms(100);
			}
			
			OCR0A = 255;
			OCR0B = 255;
			OCR2A = 255;
			OCR2B = 255;
			
			/* Wait for the vehicle to stop properly in order to get better compass data. */
			for (int i = 0; i < 30; i++) {
				_delay_ms(100);
			}
			absDiff = absDouble(currentAngle - wantedAngle);	
		}		
	}
}

//Haversine formula
double checkDistance(double latPersonIn, double lonPersonIn) {

	parseGPS();
	//lat = 63.820401;
	//lon = 20.310892;
	
	double latPerson =  latPersonIn;
	double lonPerson = lonPersonIn;
	
	double dx, dy, dz;
	lonPerson -= lon;
	lonPerson *= TO_RAD, latPerson *= TO_RAD, lat *= TO_RAD;
	
	dz = sin(latPerson) - sin(lat);
	dx = cos(lonPerson) * cos(latPerson) - cos(lat);
	dy = sin(lonPerson) * cos(latPerson);
	
	/* Return the distance between the vehicle and the person. */
	return (asin(sqrt(dx * dx + dy * dy + dz * dz) / 2) * 2 * R)<0.00002;
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
