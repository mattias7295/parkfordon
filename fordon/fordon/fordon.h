/*
 * fordon.h
 *
 * Created: 2014-05-26 15:56:01
 *  Author: masc0058
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include "usart.h"
#include "adc.h"
#include "pwm.h"
#include "twi.h"
#include "GPSparser.h"

#define FORWARDADC	1
#define BACKWARDADC 0
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

#define R  6362.697 // Earth radius in Sweden.
#define PI 3.1415926535897932385
#define TO_RAD (PI / 180)
#define TO_DEG (180 / PI)

/* Boolean type. */
typedef int bool;
#define true 1
#define false 0

/* Global variables for the previous speeds of the left and right engines. */
extern uint8_t prevSpeedR;
extern uint8_t prevSpeedL;

/* Global variables for the soft steering. */
extern bool forwardRight;
extern bool forwardLeft;
extern bool doNotChangeDirection;

/* Global variables for the double decimal degree coordinates of the vehicle. */
extern double lat;
extern double lon;

/* Function declarations. */
void init();
void timer_init();
int manualSteering();
void spin(double latPerson, double lonPerson);
int automaticSteering();
double absDouble(double number);
double getDistance(double latPerson, double lonPerson);
ISR(TIMER1_OVF_vect);
