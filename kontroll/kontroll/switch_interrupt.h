/*
 * button_interrupt.h
 *
 * Created: 2014-05-12 13:05:47
 *  Author: anjo0409
 */ 

#include <avr/interrupt.h>

/* Define switch ports. */
#define ON_OFF_SWITCH INT0
#define STEER_SWITCH INT1

/* Power mode enum. */
typedef enum {ON, OFF} power_mode;

/* Steer mode enum. */
typedef enum {MAN, AUTO} steer_mode;

/* Global steer and power on/off flags. */
extern power_mode power;
extern steer_mode steer;

/* Function declarations. */
void initOnInterrupt();
void initOffInterrupt();
void initSteerInterrupt();