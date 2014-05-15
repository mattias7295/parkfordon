/*
 * kontroll.c
 *
 * Created: 2014-05-09 09:20:57
 *  Author: masc0058
 */ 

#include "control_pad.h"

#include <util/delay.h>

/* Function declarations. */
void init();
void USART_Init(unsigned int ubrr);
void USART_Transmit(unsigned char data);
unsigned char USART_Receive();
uint16_t adc_read(uint8_t adcx);
int getXValue();
int getYValue();
void setDirections(engine_data *edata, const double angle);
void setThrottles(engine_data *edata, const double angle, const int x_value, const int y_value);
unsigned char compactData(engine_data *edata);
void sleepMode();

/* Global joystick coordinate variables. */
int x_value;
int y_value;

int main(void) {
	
	/* Data to be sent via bluetooth. */
	unsigned char send_data;
	
    /* Engine data container. */
	engine_data *edata, ed;
	edata = &ed;
	
	/* Initialize setup. */
	init();

	/* Main loop. */
	while (1) {
		
		if (steer == MAN) {
			
			/* Get coordinates. */
			x_value = getXValue();
			y_value = getYValue();
		
			/* Calculate angle of the position in a coordinate system. */
			double angle = atan2((double)y_value, (double)x_value);
		
			/* Set all info in edata. */
			setDirections(edata, angle);
			setThrottles(edata, angle, x_value, y_value);
		
			/* Compact all engine data into one 8-bit char. */
			send_data = compactData(edata);
		
			/* Start data transfer signal. */
			//USART_Transmit(255);
			
			/* Send data via bluetooth. */
//			USART_Transmit(send_data);
		
		} else {
			// Get steering info from GPS unit
		}
		
		parseGPS();
		
		for (int i = 0; i < 9; i++) {
		//	USART_Transmit(latitude[i]);
		}
				
		_delay_ms(1000);
		
		for (int i = 0; i < 10; i++) {
		//	USART_Transmit(longitude[i]);
		}
		
		_delay_ms(1000);
		
		/* Check if sleep mode is to be activated. */
		if (power == OFF) {
			sleepMode();
		}
				
	}
}

/*
* Function:	init
* Input:	- 
* Output:	- 
* Description:	Initializes ports, joystick, ADC, USART (bluetooth)
*				and timer.
*/
void init() {
	
	/* Set power ports to output and high. */
	DDRA |= _BV(POWER_PORT_3V);
	DDRA |= _BV(POWER_PORT_5V);
	PORTA |= _BV(POWER_PORT_3V);
	PORTA |= _BV(POWER_PORT_5V);
	
	/* Set power control to output and constantly high since we start in on mode. */
	DDRB |= _BV(POWER_CONTROL);
	PORTB |= _BV(POWER_CONTROL);
	
	/* Set steer control to output. */
	DDRB |= _BV(STEER_CONTROL);
	
	/* Set switches as inputs with pull up resistance. */
	DDRD &= ~_BV(ON_OFF_SWITCH);
	DDRD &= ~_BV(STEER_SWITCH);
	PORTD |= _BV(ON_OFF_SWITCH);
	PORTD |= _BV(STEER_SWITCH);

	/* Enable the ADC. */
	ADCSRA |= _BV(ADEN);
	
	/* Initialize the USART. */
	USART_Init(MYUBRR);
	
	/* Initialize the GPS parser. */
	initGPSParser(MYUBRR);
	
	/* Initialize global flags and indicate steering mode with LED. */	
	power = ON;
	
	if (PINB & (1<<STEER_SWITCH_IN)) {
		steer = MAN;
		PORTB &= ~_BV(STEER_CONTROL);
	} else {
		steer = AUTO;
		PORTB |= _BV(STEER_CONTROL);
	}
	
	/* Initialize the interrupts for turning off the control pad and
	 * changing the steering mode. */
	initOffInterrupt();	
	initSteerInterrupt();
	
	/* Set global interrupt flag. */
	sei();
}

/*
* Function: USART_Init
* Input:	ubrr: unsigned int - The wanted baud rate.
* Output:	-
* Description: Initializes the USART.
*/
void USART_Init(unsigned int ubrr) {
	
	/*Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	
	/*Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

/*
* Function: USART_Transmit
* Input:	data: unsigned char - The data to be sent.
* Output:	-
* Description: Transmits data via bluetooth.
*/
void USART_Transmit(unsigned char data) {
	
	/* Wait for empty transmit buffer */
	while (!(UCSR0A & (1<<UDRE0)));
	
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

/*
* Function:	USART_Receive
* Input:	-
* Output:	unsigned char - The received data.
* Description: Receives data via bluetooth.
*/
unsigned char USART_Receive() {
	
	/* Wait for data to be received */
	while (!(UCSR0A & (1<<RXC0)));
	
	/* Get and return received data from buffer */
	return UDR0;
}

/*
* Function:	adc_read
* Input:	adcx: uint8_t - The analog pin we want to use.
* Output:	uint16_t - The analog reading on input pin.
* Description:  Reads the ADC from input pin and returns the analog reading. 
*				Borrowed from the ATMega328P datasheet and modified.
*/
uint16_t adc_read(uint8_t adcx) {
	
	/* Takes voltage levels from Aref and sets which analog pin we want
	 * to read from. */
	ADMUX &= (1<<REFS0)|(1<<ADLAR);
	ADMUX |= adcx;

	/* This starts the conversion. */
	ADCSRA |= _BV(ADSC);

	/* This is an idle loop that just wait around until the conversion
	* is finished. It constantly checks ADCSRA's ADSC bit, which we just
	* set above, to see if it is still set. This bit is automatically
	* reset (zeroed) when the conversion is ready so if we do this in
	* a loop the loop will just go until the conversion is ready. */
	while ((ADCSRA & _BV(ADSC)));

	/* Finally, we return the converted value to the calling function. */
	return ADC;
}

/*
* Function: getXValue
* Input:	-
* Output:	int - The x coordinate of the joystick.
* Description:	Gets the X value of the position of the joystick in
*				a Cartesian coordinate system depending on the return
*				value of the adc_read function.
*/
int getXValue() {
	
	int x_value;
	uint16_t read = adc_read(X_AXIS);
	
	if (read >= ADC_THRESHOLDH7) {
		x_value = 7;
	} else if (read > ADC_THRESHOLDH6) {
		x_value = 6;
	} else if (read > ADC_THRESHOLDH5) {
		x_value = 5;
	} else if (read > ADC_THRESHOLDH4) {
		x_value = 4;
	} else if (read > ADC_THRESHOLDH3) {
		x_value = 3;
	} else if (read > ADC_THRESHOLDH2) {
		x_value = 2;
	} else if (read > ADC_THRESHOLDH1) {
		x_value = 1;
	} else if (read <= ADC_THRESHOLDL7) {
		x_value = -7;
	} else if (read < ADC_THRESHOLDL6) {
		x_value = -6;
	} else if (read < ADC_THRESHOLDL5) {
		x_value = -5;
	} else if (read < ADC_THRESHOLDL4) {
		x_value = -4;
	} else if (read < ADC_THRESHOLDL3) {
		x_value = -3;
	} else if (read < ADC_THRESHOLDL2) {
		x_value = -2;
	} else if (read < ADC_THRESHOLDL1){
		x_value = -1;
	} else {
		x_value = 0;
	}
					
	return x_value;
}

/*
* Function: getYValue
* Input:	-
* Output:	int - The y coordinate of the joystick.
* Description:	Gets the Y value of the position of the joystick in
*				a Cartesian coordinate system depending on the return
*				value of the adc_read function.
*/
int getYValue() {
	
	int y_value;
	uint16_t read = adc_read(Y_AXIS);
	
	if (read >= ADC_THRESHOLDH7) {
		y_value = -7;
	} else if (read > ADC_THRESHOLDH6) {
		y_value = -6;
	} else if (read > ADC_THRESHOLDH5) {
		y_value = -5;
	} else if (read > ADC_THRESHOLDH4) {
		y_value = -4;
	} else if (read > ADC_THRESHOLDH3) {
		y_value = -3;
	} else if (read > ADC_THRESHOLDH2) {
		y_value = -2;
	} else if (read > ADC_THRESHOLDH1) {
		y_value = -1;
	} else if (read <= ADC_THRESHOLDL7) {
		y_value = 7;
	} else if (read < ADC_THRESHOLDL6) {
		y_value = 6;
	} else if (read < ADC_THRESHOLDL5) {
		y_value = 5;
	} else if (read < ADC_THRESHOLDL4) {
		y_value = 4;
	} else if (read < ADC_THRESHOLDL3) {
		y_value = 3;
	} else if (read < ADC_THRESHOLDL2) {
		y_value = 2;
	} else if (read < ADC_THRESHOLDL1) {
		y_value = 1;
	} else {
		y_value = 0;
	}
	
	return y_value;
}

/*
* Function: setDirections
* Input:	edata: engine_data - The engine data to be altered.
*			angle: const double - The angle of the joystick position in
*									the interval [-pi, pi]. 
* Output:	-
* Description: Sets the engine directions given the angle of the joystick.
*/
void setDirections(engine_data *edata, const double angle) {
	
	if (angle <= 3*PI/4 && angle >= -PI/4) {
		edata->left_engine_dir = FORWARD;
	} else {
		edata->left_engine_dir = BACKWARD;
	}
	
	if (angle <= PI/4 && angle >= -3*PI/4) {
		edata->right_engine_dir = BACKWARD;
	} else {
		edata->right_engine_dir = FORWARD;
	}
}

/*
* Function: setThrottles
* Input:	edata: engine_data - The engine data to be altered.
*			angle: const double - The angle of the joystick position in
*									the interval [-pi, pi].
*			x_value: const int - The current x position of the joystick.
*			y_value: const int - The current y position of the joystick.
* Output:	-
* Description: Sets the engine throttles. 
*/
void setThrottles(engine_data *edata, const double angle, const int x_value, const int y_value) {
	
	/* Variable for storing the percentage of the speed of one engine
	 * to be set to the other. */
	double factor; 
	
	/* If the angle is in [B, B + PI/4) we set the throttle of 
	* the current head engine to the largest of the x and y values
	* and the angle decides how great a percentage of the x value
	* we will set the other engine throttle to through the formulae:
	*
	* 1 - (A - B)/I	for [0, PI/4), [PI/2, 3PI/4), [-PI/4, 0) and [-3PI/4, -PI/2).  
	* (A - B)/I		for [PI/4, PI/2), [3PI/4, PI), [-PI/2, -PI/4) and [-PI, -3PI/4). 
	*
	* where A is the angle, B is the low angle of the interval and 
	* I is the interval length (constantly PI/4).
	* 
	* The head engines are:
	* Left engine		for [0, PI/2) and [-PI, -PI/2)  
	* Right engine		for [PI/2, PI) and [-PI/2, 0). 
	*/
	if (angle >= 0 && angle < PI/4) {
		factor = 1 - angle/(PI/4);
		edata->left_engine_throttle = (double)(abs(x_value));
		edata->right_engine_throttle = edata->left_engine_throttle * factor;
		
	} else if (angle >= PI/4 && angle < PI/2) {
		factor = (angle - PI/4)/(PI/4);
		edata->left_engine_throttle = (double)abs(y_value);
		edata->right_engine_throttle = edata->left_engine_throttle * factor;
		
	} else if (angle >= PI/2 && angle < 3*PI/4) {
		factor = 1 - (angle - PI/2)/(PI/4);
		edata->right_engine_throttle = (double)abs(y_value);
		edata->left_engine_throttle = edata->right_engine_throttle * factor;
		
	} else if (angle >= 3*PI/4 && angle <= PI) {
		factor = (angle - 3*PI/4)/(PI/4);
		edata->right_engine_throttle = (double)abs(x_value);
		edata->left_engine_throttle = edata->right_engine_throttle * factor;
		
	} else if (angle < 0 && angle > -PI/4) {
		factor = 1 - angle*(-1)/(PI/4);
		edata->right_engine_throttle = (double)abs(x_value);
		edata->left_engine_throttle = edata->right_engine_throttle * factor;
		
	} else if (angle <= -PI/4 && angle > -PI/2) {
		factor = (angle*(-1) - PI/4)/(PI/4);
		edata->right_engine_throttle = (double)abs(y_value);
		edata->left_engine_throttle = edata->right_engine_throttle * factor;
		
	} else if (angle <= -PI/2 && angle > -3*PI/4) {
		factor = 1 - (angle*(-1) - PI/2)/(PI/4);
		edata->left_engine_throttle = (double)abs(y_value);
		edata->right_engine_throttle = edata->left_engine_throttle * factor;
		
	} else {
		factor = (angle*(-1) - 3*PI/4)/(PI/4);
		edata->left_engine_throttle = (double)abs(x_value);
		edata->right_engine_throttle = edata->left_engine_throttle * factor;
	}
}

/*
* Function: compactData
* Input:	edata: engine_data - The engine data.
* Output:	unsigned char - The compressed data to be sent.
* Description:	Adds all engine info into one byte in the form
* [dir. left (1 bit)][throttle left (3 bit)][dir. right (1 bit)][throttle right (3 bit)].
*/
unsigned char compactData(engine_data *edata) {
	
	unsigned char send_data;
	
	/* Place the left engine throttle data in send_data. */ 
	if (edata->left_engine_throttle >= 6.5) {
		send_data = (unsigned char) 7;
	} else if (edata->left_engine_throttle >= 5.5) {
		send_data = (unsigned char) 6;
	} else if (edata->left_engine_throttle >= 4.5) {
		send_data = (unsigned char) 5;
	} else if (edata->left_engine_throttle >= 3.5) {
		send_data = (unsigned char) 4;
	} else if (edata->left_engine_throttle >= 2.5) {
		send_data = (unsigned char) 3;
	} else if (edata->left_engine_throttle >= 1.5) {
		send_data = (unsigned char) 2;
	} else if (edata->left_engine_throttle >= 0.5) {
		send_data = (unsigned char) 1;
	} else {
		send_data = (unsigned char) 0;
	}
	
	/* Shift data four steps, which puts the left engine throttle 
	 * information in bit 6, 5 and 4. */
	send_data = send_data << 4;

	/* Put the right engine throttle information in the three lowest bits. */
	if (edata->right_engine_throttle >= 6.5) {
		send_data = send_data | (unsigned char) 7;
	} else if (edata->right_engine_throttle >= 5.5) {
		send_data = send_data | (unsigned char) 6;
	} else if (edata->right_engine_throttle >= 4.5) {
		send_data = send_data | (unsigned char) 5;
	} else if (edata->right_engine_throttle >= 3.5) {
		send_data = send_data | (unsigned char) 4;
	} else if (edata->right_engine_throttle >= 2.5) {
		send_data = send_data | (unsigned char) 3;
	} else if (edata->right_engine_throttle >= 1.5) {
		send_data = send_data | (unsigned char) 2;
	} else if (edata->right_engine_throttle >= 0.5) {
		send_data = send_data | (unsigned char) 1;
	} else {
		send_data = send_data | (unsigned char) 0;
	}
	
	/* Set the direction information in bit 7 for the left engine
	 * and bit 3 for the right engine. */
	if (edata->left_engine_dir == FORWARD) {
		send_data = send_data | (1<<7);
	} else {
		send_data = send_data | (0<<7);
	}
	
	if (edata->right_engine_dir == FORWARD) {
		send_data = send_data | (1<<3);
	} else {
		send_data = send_data | (0<<3);
	}
	
	return send_data;
}

/* 
* Function: sleepMode
* Input:	-
* Output:	-
* Description: Enters sleep mode.
*/
void sleepMode() {
	
	/* Clear global interrupt flag. */
	cli();
	
	/* Initialize low interrupt on INT0 in order for the MCU to be awoken. */
	initOnInterrupt();
	
	/* Turn off power to voltage regulator that powers bluetooth and GPS units. */
	PORTA &= ~_BV(POWER_PORT_3V);
	PORTA &= ~_BV(POWER_PORT_5V);
	
	/* Turn off leds that indicate power and steering. */
	PORTB &= ~_BV(POWER_CONTROL);
	PORTB &= ~_BV(STEER_CONTROL);
	
	/* Set output pins to input in order to save more power. */
	DDRA &= ~_BV(POWER_PORT_3V);
	DDRA &= ~_BV(POWER_PORT_5V);
	DDRB &= ~_BV(POWER_CONTROL);
	DDRB &= ~_BV(STEER_CONTROL);
	
/*	DDRD &= ~_BV(LED_PIN8);
	DDRC &= ~_BV(LED_PIN7);
	DDRC &= ~_BV(LED_PIN6);
	DDRC &= ~_BV(LED_PIN5);
	DDRC &= ~_BV(LED_PIN4);
	DDRD &= ~_BV(LED_PIN3);
	DDRD &= ~_BV(LED_PIN2);
	DDRD &= ~_BV(LED_PIN1);
*/

	/* Set sleep mode and enable sleep setup. */
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	
	/* Disable ADC, watchdog and BOD. */
	ADCSRA = 0;
	wdt_disable();
	sleep_bod_disable();
	
	/* Set global interrupt flag to allow waking signals from
	 * the external interrupt and power down MCU. */
	sei();
	sleep_cpu();
	
	/* Wake up here and disable sleep setup mode. */
	sleep_disable();
	
	/* Re-initialize. */
	init();
}