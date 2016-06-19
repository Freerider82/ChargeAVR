/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include "port_macros.h"
#include <avr/io.h>
#include <util/delay.h>
#define UP					PD0
#define DOWN				PD1	
#define ENTER				PD2
#define DDRX_BUTTON			DDRD
#define PINX_BUTTON			PIND
#define MASK_BUTTONS 	    ((1<<DOWN)|(1<<UP)|(1<<ENTER))

#define NO_BUTTONS			MASK_BUTTONS

#define BUTTON_UP			(MASK_BUTTONS ^(1<<UP))
#define BUTTON_DOWN			(MASK_BUTTONS ^(1<<DOWN))
#define BUTTON_ENTER		(MASK_BUTTONS ^(1<<ENTER))

void initPWM();
void initButtons();
void setPwm(unsigned int);
char checkButtons();

typedef struct {
	unsigned int currentPWM;
	
} PWM;

int main (void)
{
	unsigned int currentPWM=0;
	
	
	initButtons();
	initPWM();
	DDRA=0xFF;
	DDRB=0xFF;
	char stateButtons=NO_BUTTONS;
	while(1){
		
		stateButtons=checkButtons();		
		
		if(stateButtons!=NO_BUTTONS){
			switch(stateButtons){
				case BUTTON_UP: 
				if(currentPWM>=1023){
					currentPWM=0;	
				}else{
					currentPWM++;
				}
				break;
				case BUTTON_DOWN:
				if(currentPWM==0){
					currentPWM=1023;
					}else{
					currentPWM--;
				}
				break;
				case BUTTON_ENTER:
				
				break;					
			}			
			
			setPwm(currentPWM);
		}		
		
	}
}

/********************************************************************/

void initPWM(){
	/* Настройка ШИМ 
	* Используем вывод OCR1A PD5
	* Быстрая ШИМ прдел 0x3FF или 1023
	*/
	DDRD|=(1<<PD5);
	TCCR1A|=(1<<COM1A1)|(1<<WGM11)|(1<<WGM10);//0x83;
	TCCR1B|=(1<<WGM12)|(1<<CS10);//0x09;
	SET_16BIT_RG(OCR1A,0);
}
/********************************************************************/

void initButtons(){
	DDRX_BUTTON &= ~MASK_BUTTONS;
}
/********************************************************************/

void setPwm(unsigned int x ){
	SET_16BIT_RG(OCR1A,x);
}
/********************************************************************/

char checkButtons(){
	static char oldStateButton;
	char stateButton=PINX_BUTTON & MASK_BUTTONS;	
	//char stateButton=3 & MASK_BUTTONS;
	if(stateButton!=oldStateButton){
		oldStateButton=stateButton;
		switch(stateButton){
			case BUTTON_UP:			return BUTTON_UP;
			case BUTTON_DOWN:		return BUTTON_DOWN;
			case BUTTON_ENTER:		return BUTTON_ENTER;
		}
	}
	
	
	return NO_BUTTONS;
}
