#ifndef BUTTONS_LIB_H
#define BUTTONS_LIB_H

#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include "port_macros.h"

#define UP					PD2
#define DOWN				PD3
#define ENTER				PD4
#define DDRX_BUTTON			DDRD
#define PINX_BUTTON			PIND
#define MASK_BUTTONS 	    ((1<<DOWN)|(1<<UP)|(1<<ENTER))

#define NO_BUTTONS			MASK_BUTTONS

#define BUTTON_UP			(MASK_BUTTONS ^(1<<UP))
#define BUTTON_DOWN			(MASK_BUTTONS ^(1<<DOWN))
#define BUTTON_ENTER		(MASK_BUTTONS ^(1<<ENTER))

#define STATE_NO_BUTTONS    0
#define STATE_UP			1
#define STATE_UP_PRESSED	2
#define STATE_DOWN			3
#define STATE_DOWN_PRESSED	4
#define STATE_ENTER			5
#define STATE_ENTER_5SEC	6

void initButtons();
char getStateButton(uint8_t);

#endif