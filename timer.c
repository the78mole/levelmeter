/*
 * timer.c
 *
 *  Created on: 23.12.2014
 *      Author: daniel
 */

#include <avr/io.h>

void timer_init() {
	TCCR1A = 0x00;
	// Set prescaler to 1
	TCCR1B |= (1 << CS10);
	// Enable the timer capture interrupt
	TCCR1C = 0x00;
}
