#include "pwm_lib.h"
#include "port_macros.h"
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
void setPwm(unsigned int x ){
	SET_16BIT_RG(OCR1A,x);
}
/********************************************************************/
char getPercentPwm(){
	volatile unsigned long tempPwm=0;
	GET_16BIT_RG(tempPwm,OCR1A);
	
	return (char)((tempPwm*100)/1024);
}