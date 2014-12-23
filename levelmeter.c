/*
 * levelmeter.c
 *
 *  Created on: 23.12.2014
 *      Author: daniel
 */

#include <util/delay.h>
#include <avr/io.h>
#include <stdlib.h>

#define BAUD 9600
#include <util/setbaud.h>

void uart_init(void) {
	UBRR1 = 207; // UBRR_VALUE;
	UCSR1A |= (1 << U2X1);
	UCSR1B = (1 << RXEN1) | (1 << TXEN1);
	UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);

	// hier weitere Initialisierungen (TX und/oder RX aktivieren, Modus setzen
}

int main(void) {

	char ctmp = 0;

	uart_init();

	DDRB |= 1 << PB0;
	DDRD |= 1 << PD5;

	while (1) {
	}
}
