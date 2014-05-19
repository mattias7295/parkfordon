/*
 * autodrive.h
 *
 * Created: 2014-05-13 11:28:02
 *  Author: johe0179
 */ 


#ifndef AUTODRIVE_H_
#define AUTODRIVE_H_
#include <avr/io.h>
#include <stdlib.h>
#include <math.h>
#include <util/delay.h>
#include <stdio.h>
#include "twi.h"
#include "pwm.h"


void calcHeading();
int checkDistance(double latPerson, double lonPerson);
static void put_char(uint8_t c, FILE* stream);
static FILE mystdout = FDEV_SETUP_STREAM(put_char, NULL, _FDEV_SETUP_WRITE);


#endif /* AUTODRIVE_H_ */