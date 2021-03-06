/*
 * twi.c
 *
 * Created: 2014-05-09 12:43:18
 *  Author: johe0179
 */ 
#include <avr/io.h> 
#include "twi.h"
#include <util/delay.h>
#include "usart.h"

/*
* Function: compass_update
* Input: -
* Output: total: uint16_t - the heading
* Description: Communicate with the compass with twi 
* and return the heading.
*/
uint16_t compass_update() {
	
	/* Start. */
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
	
	/* Send address. */
	TWDR = 0x42;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
		
		
	/* Send data. */
	TWDR = 0x41;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
		
	/* Stop. */
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
	_delay_ms(560);
		
	/* Start. */
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
		
		
	/* Send address. */
	TWDR = 0x43;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
		
	/* Read data. */
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR & (1<<TWINT)));
	uint16_t high = TWDR;
	
	
	/* Read data. */
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	uint8_t low = TWDR;
	uint16_t total = (high<<8) | low;

	/* Stop. */
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
	return total;
}