#ifndef USART_H
#define USART_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

void USART_Init(void); 
void USART_SendChar(unsigned char sym);
unsigned char USART_GetChar(void);
void sendDigitalUsart(unsigned int);

#endif //USART_H