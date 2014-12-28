/*
 * levelmeter.c
 *
 *  Created on: 23.12.2014
 *      Author: daniel
 */

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <stdlib.h>
#include <stdio.h>

#include "usart.h"
#include "timer.h"

//#define CMD_ADDR 'a'

uint8_t CMD_ADDR_EE EEMEM = 'a';

volatile uint8_t cmd_addr;

static FILE mystdout = FDEV_SETUP_STREAM(usart_putchar_printf, NULL, _FDEV_SETUP_WRITE);

volatile uint8_t measurement_state = 0;
const uint8_t MEAS_SLEEP = 0;
const uint8_t MEAS_REQUESTED = 1;
const uint8_t MEAS_WAIT_START = 2;
const uint8_t MEAS_WAIT_END = 3;
const uint8_t MEAS_DONE = 4;
const uint8_t MEAS_RESET = 0xFE;

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

const double ticks_to_mm = (2.0 * F_CPU) / (340.0 * 1000.0);

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

	cmd_addr = eeprom_read_byte(&CMD_ADDR_EE);

	usart_init();

	timer_init();

	ports_init();

	UDIEN = 0; // Disable the USB interrupts
	USBCON = (1 << FRZCLK); // Disable USB core

	sei(); // Enable global interrupt

	printf("# [%02x] Init...\n", cmd_addr);
	printf("# [%02x] Tick to mm div: ~%d\n", cmd_addr, (int) ticks_to_mm);

	while (1) {

		if (measurement_state == MEAS_REQUESTED)
		{
				MEASLEDON;
				printf("# [%02x] Measurement requested.\n", cmd_addr);
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
			printf("# [%02x] Measurement done.\n", cmd_addr);
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
			printf("# [%02x]  Timer start : %u\n", cmd_addr, ts_start);
			printf("# [%02x]  Timer end   : %u\n", cmd_addr, ts_end);
			printf("# [%02x]  Timer oflows: %u\n", cmd_addr, tof_count);
			printf("# [%02x]  Timer diff  : %u\n", cmd_addr, ts_end - ts_start);
			printf("# [%02x]  Timer total : %lu\n", cmd_addr, total_time);

			// Convert to tenth of milli-meters
			total_time = total_time / ticks_to_mm;

			printf("RESULT [%02x] : %lu mm\n", cmd_addr, total_time);
			measurement_state = MEAS_SLEEP;
			MEASLEDOFF; // Deactivate Measurement LED
		}
		else if (measurement_state == MEAS_SLEEP)
		{
			PORTB |= LEDS_OFF;
			// Set uC to sleep with active RX IRQ
		}
		else if (measurement_state == MEAS_RESET) {
			printf("# [%02x] Resetting in 250 ms\n", cmd_addr);
			wdt_enable(WDTO_250MS);
			cli();
			while(1);
		}

	}
}

char last_rx[3] = {0,0,0};

ISR(USART1_RX_vect)
{
	char ctmp;

	//PORTD &= ~(1 << PD5);
	RXLEDON;

	ctmp = UDR1;
	if (last_rx[0] == 'r' && last_rx[1] == 's' && last_rx[2] == 't' && ctmp == cmd_addr)
	{
			measurement_state = MEAS_RESET;
	}
	else if (ctmp == cmd_addr)
	{
		measurement_state = MEAS_REQUESTED;
	}

	last_rx[0] = last_rx[1];
	last_rx[1] = last_rx[2];
	last_rx[2] = ctmp;

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
