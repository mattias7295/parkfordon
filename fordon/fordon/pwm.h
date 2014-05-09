/*
 * pwm.h
 *
 * Created: 2014-05-09 10:35:53
 *  Author: masc0058
 */ 


#ifndef PWM_H_
#define PWM_H_

#include <avr/io.h>

void init_pwm();
void initEngineRightForward(unsigned char speed);
void initEngineRightBackward(unsigned char speed);
void initEngineLeftForward(unsigned char speed);
void initEngineLeftBackward(unsigned char speed);


#endif /* PWM_H_ */