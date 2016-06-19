//***************************************************************************
//  File........: lcd_lib.h
//
//  Author(s)...: Pashgan    chipenable.ru
//
//  Target(s)...: ATMega...
//
//  Compiler....: IAR, CodeVision, GNU GCC
//
//  Description.: ������� ������������������� �� �������
//
//  Data........: 20.07.13  
//
//***************************************************************************
#ifndef LCD_LIB_H
#define LCD_LIB_H

#include "compilers_4.h"

//#define VIRT_PORT
#ifdef VIRT_PORT

//����� � ��������� ����������� ����

//���� ������ LCD
#define LCD_PORT  LCD_DATA, F, _VIRT

#define LCD_DATA_0  C, 0, _NONE
#define LCD_DATA_1  C, 1, _NONE
#define LCD_DATA_2  C, 2, _NONE
#define LCD_DATA_3  C, 3, _NONE
#define LCD_DATA_4  C, 4, _HI
#define LCD_DATA_5  C, 5, _HI
#define LCD_DATA_6  C, 6, _HI
#define LCD_DATA_7  C, 7, _HI

//����������� ������ LCD
#define LCD_RS  C, 0, _HI
#define LCD_RW  C, 1, _HI
#define LCD_EN  C, 2, _HI 

#else

//� ����� � ��������� �������� ����

//���� ������ LCD
#define LCD_PORT  LCD_DATA, C, _REAL

#define LCD_DATA_0  C, 0, _NONE
#define LCD_DATA_1  C, 1, _NONE
#define LCD_DATA_2  C, 2, _NONE
#define LCD_DATA_3  C, 3, _NONE
#define LCD_DATA_4  C, 4, _HI
#define LCD_DATA_5  C, 5, _HI
#define LCD_DATA_6  C, 6, _HI
#define LCD_DATA_7  C, 7, _HI

//����������� ������ LCD
#define LCD_RS  C, 0, _HI
#define LCD_RW  C, 1, _HI
#define LCD_EN  C, 2, _HI

#endif

/*____________________________________________________________________*/

//���������� ��������� ��������
#define LCD_CHECK_FL_BF             1
#define LCD_BUS_4_8_BIT             0

//��������� ������������� �������
#define LCD_ONE_TWO_LINE            1
#define LCD_FONT58_FONT511          0
#define LCD_DEC_INC_DDRAM           1
#define LCD_SHIFT_RIGHT_LEFT        0
#define LCD_DISPLAY_OFF_ON          1
#define LCD_CURSOR_OFF_ON           0
#define LCD_CURSOR_BLINK_OFF_ON     0 //��������� ������ ���� �������
#define LCD_CURSOR_DISPLAY_SHIFT    0

/*_____________________����� �������________________________________*/

//�������
#define LCD_CLEAR_DISPLAY  0x01
#define LCD_RETURN_HOME    0x02

//������� �������
#define LCD_Clear() do{LCD_WriteCom(LCD_CLEAR_DISPLAY); _delay_ms(2);}while(0)   

//������� ������� � ��������� ���������
#define LCD_ReturnHome()  do{LCD_WriteCom(LCD_RETURN_HOME);}while(0)

//���������������� �������
#define LCD_Goto(x, y)    LCD_WriteCom(((((y)& 1)*0x40)+((x)& 15))|128)  

/*___________________���������������� �������_______________________*/

void LCD_Init(void);                                  //������������� ������ � ��� �� ���������
void LCD_WriteCom(uint8_t data);                      //�������� ������� ���
void LCD_WriteData(char data);                        //������� ������ �� ���
void LCD_SendStr(char *str);                          //������� ������ �� ���
void LCD_SendNumber(uint16_t ,unsigned char);
void LCD_ClearField(uint8_t,uint8_t,uint8_t);

#ifdef __GNUC__
   void LCD_SendStrFl(char *str);                           //������� ������ �� ���� ������     
   void LCD_SetUserChar(uint8_t const *sym, uint8_t adr);   //��������� ���������������� ������
#else
   void LCD_SendStrFl(char __flash *str);                   //������� ������ �� ���� ������
   void LCD_SetUserChar(uint8_t __flash *sym, uint8_t adr); //��������� ���������������� ������
#endif

#endif