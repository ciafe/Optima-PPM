/*
* Copyright (c) 2012 Scott Driessens
* Licensed under the MIT License
*/

#include <avr/io.h>

#include "optima.h"

pin_map outputs[OPTIMA_NUM_CHANNELS] = {
	{(1<<PC1), &PINC, &PORTC, &DDRC}, // Channel 1  (Arduino ProMini A1/D15)
	{(1<<PC0), &PINC, &PORTC, &DDRC}, // Channel 2  (Arduino ProMini A0/D14)
	{(1<<PD4), &PIND, &PORTD, &DDRD}, // Channel 3  (Arduino ProMini D4)
	{(1<<PD3), &PIND, &PORTD, &DDRD}, // Channel 4  (Arduino ProMini D3)
	{(1<<PD2), &PIND, &PORTD, &DDRD}, // Channel 5  (Arduino ProMini D2)
	{(1<<PC2), &PINC, &PORTC, &DDRC}, // Channel 6  (Arduino ProMini A2/D16)
	{(1<<PC3), &PINC, &PORTC, &DDRC}, // Channel 7 - Optima 7/9 only  (Arduino ProMini A3/D17)
	{(1<<PC4), &PINC, &PORTC, &DDRC}, // Channel 8 - Optima 9 only
	{(1<<PC5), &PINC, &PORTC, &DDRC}  // Channel 9 - Optima 9 only */
};

/* PORT MAP
PORTB
	PB0	-
	PB1	-
	PB2	-
	PB3	-
	PB4	-
	PB5	-
	PB6	- XTAL
	PB7	- XTAL
PORTC
	PC0	- Channel 2
	PC1	- Channel 1
	PC2	- Channel 6
	PC3	- Channel 7 - Optima 7/9 only
	PC4	- Channel 8 - Optima 9 only
	PC5	- Channel 9 - Optima 9 only
	PC6	- 
	PC7	-
PORTD
	PD0	- TXD
	PD1	- RXD
	PD2	- Channel 5
	PD3	- Channel 4
	PD4	- Channel 3
	PD5	- 
	PD6	- 
	PD7	-	
*/
