/*
 * levelmeter.c
 *
 *  Created on: 23.12.2014
 *      Author: daniel
 */

#include <util/delay.h>
#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>

#include "usart.h"
#include "timer.h"

// Addr 0 reacts on any command from serial bus with a measurement
#define CMD_ADDR 'a'

static FILE mystdout = FDEV_SETUP_STREAM(usart_putchar_printf, NULL, _FDEV_SETUP_WRITE);


int main(void) {

	char ctmp = 0;
	int i;
	uint32_t ptime;

	stdout = &mystdout;

	usart_init();

	timer_init();

	DDRB |= 1 << PB0; // TX LED -> Ready indicator
	DDRD |= 1 << PD5; // RX LED -> Measurement indicator
	DDRC |= 1 << PC6; // US-Trigger

	// Initialize trigger as high
	PORTC |= 1 << PC6;

	//printf("\n\n\n\nInit...\n\n");
	while (1) {

		PORTB &= ~(1 << PB0);

		if (CMD_ADDR == 0)
			while(!(ctmp = usart_getchar()));
		else
			while((ctmp = usart_getchar()) != CMD_ADDR);

		PORTB |= 1 << PB0;

		//printf("Got command: %c\n", ctmp);

		ptime = capture_pulsewidth();

		if(CMD_ADDR != 0)
			printf("%c: ", CMD_ADDR);

		printf("%lu mm\n", ptime);

		PORTD |= 1 << PD4;
	}
}
