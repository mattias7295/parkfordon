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
void initEngineRightForward();
void initEngineRightBackward();
void initEngineLeftForward();
void initEngineLeftBackward();



#endif /* PWM_H_ */