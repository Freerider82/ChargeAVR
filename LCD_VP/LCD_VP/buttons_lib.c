#include "buttons_lib.h"

/********************************************************************/

void initButtons(){
	DDRX_BUTTON &= ~MASK_BUTTONS;
}

/********************************************************************/

char getStateButton(uint8_t codeButton){	
		
		volatile uint8_t temp=0;
		switch(codeButton){
			case BUTTON_UP:					
				return STATE_UP;
			case BUTTON_DOWN:				
				return STATE_DOWN;
			case BUTTON_ENTER:				
			while(codeButton==BUTTON_ENTER){
				_delay_ms(20);
				temp++;
				if(temp==200){
					return STATE_ENTER_5SEC;
				}
				codeButton=PINX_BUTTON & MASK_BUTTONS;				
			}
			return STATE_ENTER;
		}
	
		return STATE_NO_BUTTONS;	
	
}