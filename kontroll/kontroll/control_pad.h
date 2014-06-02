/*
 * control_pad.h
 *
 * Routines for a control pad used for driving a vehicle either 
 * manually or automatically, depending on the position of a 
 * switch on the control pad.
 *
 * Created: 2014-05-09 11:56:39
 *  Author: anjo0409
 */ 

#include <avr/io.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "switch_interrupt.h"
#include "GPS_parser.h"

/* USART definitions. */
#define FOSC 8000000 // Clock Speed
#define BAUD 9600
#define MYUBRR 51

/* Which analog pin we want to read from. The pins are labeled "ADC0"
* "ADC1" etc on the pinout in the data sheet. In this case X_AXIS
* being 0 means we want to use ADC0. */
#define X_AXIS PA0
#define Y_AXIS PA1

/* Power to bluetooth port. */
#define POWER_PORT_3V PA3
#define POWER_PORT_5V PA2

/* Bluetooth connection port. */
#define BT_CONNECTION_PORT PD6

/* GPS fix port. */
#define GPS_fix_port PB3

/* The ADC value we will consider the cutoff point for turning the LED
* on or off. The ADC we are using is 10-bits so can be a value from
* 0 to 1023. The value 0 means that there is no voltage on the ADC pin
* and the value 1023 means the voltage has reached the voltage on the
* AREF pin. */
#define ADC_THRESHOLDH7 960
#define ADC_THRESHOLDH6 896
#define ADC_THRESHOLDH5 832
#define ADC_THRESHOLDH4 768
#define ADC_THRESHOLDH3 704
#define ADC_THRESHOLDH2 640
#define ADC_THRESHOLDH1 576
#define ADC_THRESHOLDM 512
#define ADC_THRESHOLDL1 448
#define ADC_THRESHOLDL2 384
#define ADC_THRESHOLDL3 320
#define ADC_THRESHOLDL4 256
#define ADC_THRESHOLDL5 192
#define ADC_THRESHOLDL6 128
#define ADC_THRESHOLDL7 64

/* Direction enum. */
typedef enum {FORWARD, BACKWARD} direction;

/* Engine direction and throttle variables. */
typedef struct  {
	direction left_engine_dir;
	direction right_engine_dir;
	double left_engine_throttle;
	double right_engine_throttle;
} engine_data;

/* PI. */
#define PI 3.1415926535897932385

/* Global joystick position variables. */
extern int x_value;
extern int y_value;

/* Function declarations. See control-pad.c for further information. */
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