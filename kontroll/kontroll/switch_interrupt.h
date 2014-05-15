/*
 * button_interrupt.h
 *
 * Created: 2014-05-12 13:05:47
 *  Author: anjo0409
 */ 

#include <avr/interrupt.h>

/* Define switch interrupt ports. */
#define ON_OFF_SWITCH INT1
#define STEER_SWITCH INT2

/* Define steer switch input port for checking. */
#define STEER_SWITCH_IN PB2

/* Power and steer control lights. */
#define POWER_CONTROL PB0
#define STEER_CONTROL PB1

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
ISR(INT1_vect);
ISR(INT2_vect);