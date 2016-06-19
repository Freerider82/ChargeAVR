#include "usart.h"

//����������� �����
volatile unsigned char usartRxBuf = 0;

//������������� usart`a
void USART_Init(void)
{
  UBRRH=0;
  UBRRL=25; //�������� ������ 2400 ���
  UCSRB=(0<<RXCIE)|(0<<RXEN)|(1<<TXEN); //����. ������ ��� ������, ���� ������, ���� ��������.
  UCSRC=(1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);  //������ ����� 8 ��������
}

//�������� ������� �� usart`�
void USART_SendChar(unsigned char sym)
{
	//UDRE ���� ����������� �������� ������ UDR
  while(!(UCSRA & (1<<UDRE)));
  UDR = sym;  
}

//����� ������� �� usart`� � �����
ISR(USART_RX_vect)
{
   usartRxBuf = UDR;  
} 

//������ ������
unsigned char USART_GetChar(void)
{
  unsigned char tmp;
  
  ATOMIC_BLOCK(ATOMIC_FORCEON)
  {
        tmp = usartRxBuf;
        usartRxBuf = 0;
  }

  return tmp;  
}

void sendDigitalUsart(unsigned int sym){
	if(sym==0){
		USART_SendChar('0');
		return;
	}
	unsigned char i=0;
	unsigned char arrayDigital[5];
	while(sym!=0){
		arrayDigital[i++]=sym%10;
		sym/=10;
	}
	while(i!=0)	{
		USART_SendChar(arrayDigital[--i]+'0');		
	}
	
}

