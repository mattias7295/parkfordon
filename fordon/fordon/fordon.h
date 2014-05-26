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

extern uint8_t prevSpeedR;
extern uint8_t prevSpeedL;
extern double lat;
extern double lon;

int parseBluetooth(unsigned char command);
static void put_char(uint8_t c, FILE* stream);
void spin(double latPerson, double lonPerson);
int calcHeading(unsigned char command);
double absDouble(double number);
double checkDistance(double latPersonIn, double lonPersonIn);
ISR(TIMER1_OVF_vect);
ISR(USART_RXC_vect);