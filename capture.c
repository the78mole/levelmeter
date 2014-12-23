/*
 * capture.c
 *
 *  Created on: 23.12.2014
 *      Author: daniel
 */

#include <avr/io.h>

#include "capture.h"

uint16_t caprising = 0;
uint16_t capfalling = 0;
uint16_t capdiff = 0;

uint16_t icp_start, icp_stop, icp_period;

char capslope = 0;

uint16_t capture_pulsewidth(void) {

	uint16_t pstart, pend, ptot;
	uint32_t putmp;

	PORTD &= ~(1 << PD5);

	// Set rising edge for input capture and 1/8 prescale
	TCCR1B = (1 << ICES3) | (2 << CS10);
	// Clear all timer flags
	TIFR1 = (1 << ICF1) | (1 << OCF1A);

	// Trigger measurement
	PORTC &= ~(1 << PC6); // Trigger

	// Wait for rising edge
	while(1) {
		if (TIFR1 & (1 << ICF1)) {
			pstart = ICR1;
			break;
		} else if (PIND & (1 << PD4)) {
			pstart = TCNT1;
			break;
		}
	}

	// Revert the capture edge to falling
	TCCR1B &= ~(1 << ICES3);
	// Clear all timer flags
	TIFR1 = (1 << ICF1);

	// Wait for falling edge
	while(1) {
		if (TIFR1 & (1 << ICF1)) {
			pend = ICR1;
			break;
		} else if ((PIND & (1 << PD4)) == 0) {
			pend = TCNT1;
			break;
		}
	}

	// Save the captured value
	ptot = pend - pstart;

	PORTC |= 1 << PC6; // Trigger off

	// ptot = ptot * (SOS / 2) * (1 / F_CPU) * CLK_DIV;
	putmp = ptot;
	putmp *= 340;	// speed of sound
	putmp /= 2; 	// wave traveling forth and back
	// putmp *= 1000; 	// m -> mm
	// putmp *= 8;		// prescaler of timer
	putmp /= (F_CPU / 8) / 1000;

	if(putmp > (uint32_t) UINT16_MAX) {
		ptot = UINT16_MAX;
	} else {
		ptot = (uint16_t) putmp;
	}

	PORTD |= 1 << PD5;

	return ptot;
}
