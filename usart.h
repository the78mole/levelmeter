/*
 * usart.h
 *
 *  Created on: 23.12.2014
 *      Author: daniel
 */

#ifndef USART_H_
#define USART_H_

#include <avr/io.h>
#include <stdio.h>
#include <stdbool.h>

#define BAUD 9600
#include <util/setbaud.h>

/********************************************************************************
Function Prototypes
********************************************************************************/

extern void usart_init( void );
extern char usart_getchar( void );
extern void usart_putchar( char data );
extern void usart_pstr (char *s);
extern unsigned char usart_kbhit(void);

extern int usart_putchar_printf(char var, FILE *stream);

#endif /* USART_H_ */
