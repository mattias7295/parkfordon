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
#include "twi.h"
#include "pwm.h"


void calcHeading(double latPerson, double lonPerson);
int checkDistance(double latPerson, double lonPerson);


#endif /* AUTODRIVE_H_ */