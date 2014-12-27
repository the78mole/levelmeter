/*
 * levelmeter.c
 *
 *  Created on: 23.12.2014
 *      Author: daniel
 */

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>

#include "usart.h"
#include "timer.h"

#define CMD_ADDR 'a'

static FILE mystdout = FDEV_SETUP_STREAM(usart_putchar_printf, NULL, _FDEV_SETUP_WRITE);

volatile uint8_t measurement_state = 0;
const uint8_t MEAS_SLEEP = 0;
const uint8_t MEAS_REQUESTED = 1;
const uint8_t MEAS_WAIT_START = 2;
const uint8_t MEAS_WAIT_END = 3;
const uint8_t MEAS_DONE = 4;

volatile uint16_t ts_start;
volatile uint16_t ts_end;
volatile uint16_t tof_count = 0;
volatile uint32_t total_time = 0;

#define LEDS_OFF ( (1 << PB2) | (1 << PB3) | (1 << PB6) )

#define RXLEDON PORTB &= ~(1 << PB6)
#define RXLEDOFF PORTB |= (1 << PB6)

#define CAPLEDON PORTB &= ~(1 << PB2)
#define CAPLEDOFF PORTB |= (1 << PB2)

#define MEASLEDON PORTB &= ~(1 << PB0)
#define MEASLEDOFF PORTB |= (1 << PB0)

static inline void gen_meas_trigger(void)
{
	PORTC |= 1 << PC6;
	_delay_us(100);
	PORTC &= ~(1 << PC6);
}

void ports_init(void) {
	DDRB |= 1 << PB0; // TX LED -> Measurement indicator
	DDRD |= 1 << PD5; // RX LED -> Sleep inidcator
	DDRC |= 1 << PC6; // US-Trigger
	DDRB |= 1 << PB6; // LED1 -> USART RX
	DDRB |= 1 << PB2; // LED2 -> Capture
	DDRB |= 1 << PB3; // LED3 -> Overflow

	// Initialize trigger as high
	//PORTC |= 1 << PC6;

	PORTB |= (1 << PB0) | LEDS_OFF;
	PORTD |= (1 << PD5);

}

const
int main(void) {

	stdout = &mystdout;

	usart_init();

	timer_init();

	ports_init();

	UDIEN = 0; // Disable the USB interrupts
	USBCON = (1 << FRZCLK); // Disable USB core

	sei(); // Enable global interrupt

	printf("# Init...\n");

	while (1) {

		if (measurement_state == MEAS_REQUESTED)
		{
				MEASLEDON;
				printf("# Measurement requested.\n");
				// Clear all timer flags
				TIFR1 = (1 << ICF1) | (1 << OCF1A);

				measurement_state = MEAS_WAIT_START;
				// Enable the timer interrupts
				TIMSK1 |= (1 << ICIE1);
				// Set rising edge for input capture and 1/1 prescale
				TCCR1B |= (1 << ICES3);
				// Generate the trigger signal
				tof_count = 0;
				gen_meas_trigger();
		}
		else if (measurement_state == MEAS_DONE)
		{
			printf("# Measurement done.\n");
			// Do the calculations
			total_time = ts_end - ts_start;
/*			if (tof_count > 0)
			{
				if (ts_start < ts_end)
					tof_count--;

				total_time += ((uint32_t) tof_count - 1) * ((uint32_t) UINT16_MAX);
			} */
			total_time += tof_count * UINT16_MAX;

			// Send result over serial port
			printf("#  Timer start : %u\n", ts_start);
			printf("#  Timer end   : %u\n", ts_end);
			printf("#  Timer oflows: %u\n", tof_count);
			printf("#  Timer diff  : %u\n", ts_end - ts_start);
			printf("#  Timer total : %lu\n", total_time);

			// Convert to tenth of milli-meters
			total_time *= 34; 				// speed of sound in meters per tenth sec
			total_time /= 2;				// Forth and back travel of wave
			total_time *= 10;				// tenth to seconds
			total_time /= (F_CPU / 10000); 	// Count frequency in kHz

			printf("RESULT [%c] : %lu.%d mm\n", CMD_ADDR, total_time / 10, total_time % 10);
			measurement_state = MEAS_SLEEP;
			MEASLEDOFF; // Deactivate Measurement LED
		}
		else if (measurement_state == MEAS_SLEEP)
		{
			PORTB |= LEDS_OFF;
			// Set uC to sleep with active RX IRQ
		}

	}
}

ISR(USART1_RX_vect)
{
	char ctmp;

	//PORTD &= ~(1 << PD5);
	RXLEDON;

	ctmp = UDR1;
	if (ctmp == CMD_ADDR) {
		measurement_state = MEAS_REQUESTED;
	}

	RXLEDOFF;
}

ISR(TIMER1_CAPT_vect)
{
	cli();

	// Clear input capture flag
	TIFR1 = (1 << ICF1) | (1 << OCF1A);

	if (measurement_state == MEAS_WAIT_START)
	{
		CAPLEDON;
		TCCR1B &= ~(1 << ICES3);
		TIMSK1 |= (1 << OCIE1A);
		measurement_state = MEAS_WAIT_END;
		ts_start = ICR1;
		OCR1A = ts_start;
	}
	else if (measurement_state == MEAS_WAIT_END)
	{
		measurement_state = MEAS_DONE;
		TIMSK1 &= ~((1 << OCIE1A) | (1 << ICIE1));
		TIFR1 = (1 << ICF1) | (1 << OCF1A);
		ts_end = ICR1;
		CAPLEDOFF;
	}

	sei();
}

ISR(TIMER1_COMPA_vect) {
	cli();
	TIFR1 = (1 << OCF1A);
	// Clear overflow flag
	tof_count++;
	// Toggle OV LED
	sei();
}
