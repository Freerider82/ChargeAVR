#ifndef PWM_LIB_H
#define PWM_LIB_H

#include <avr/io.h>
#include <util/delay.h>


void initPWM();
void setPwm(unsigned int);
char getPercentPwm();
#endif