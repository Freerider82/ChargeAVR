//***************************************************************************
//
//  Author(s)...: Pashgan    http://ChipEnable.Ru
//
//  Target(s)...: ATMega8535
//
//  Compiler....: AtmelStudio 6
//
//  Description.: Библиотека для LCD в связке с макросами виртуальных портов
//
//
//  Data........: 20.07.13
//
//***************************************************************************
#define F_CPU 1000000UL

#include "compilers_4.h"
#include "lcd_lib_2.h"
#include "port_macros.h"
#include <util/delay.h>
#include "buttons_lib.h"
#include "pwm_lib.h"
#include "usart.h"



#define StartConvAdc() ADCSRA |= (1<<ADSC)
#define SWITCH_ADC_CHANNEL(reg,val)		do{(reg) ^= (val);}while(0)
#define CHANNEL_CURRENT			11 //(1<<MUX3)|(1<<MUX1)|(1<<MUX0)
#define NUMBER_OF_MEASURING		100
#define MEASURING_CURRENT		1
#define MEASURING_VOLTAGE		0
#define TIMER0_INTERRUPT		244

#define TIME_1_SEC				4

#define DISPLAY_START				0
#define DISPLAY_FAST_CHANGE			1
#define DISPLAY_PROGR				2
#define DISPLAY_P0					3
#define DISPLAY_P1					4
#define DISPLAY_P2					5
#define DISPLAY_P3					6
#define DISPLAY_P4					7
#define DISPLAY_P5					8
#define LAST_DISPLAY				DISPLAY_P5-DISPLAY_PROGR

//Именнованные индексы для arrVal
#define normalAmperage					0
#define normalVoltage					1
#define dischargeVoltage				2
#define dischargeAmperage				3
#define currentAmperage					4
#define currentVoltage					5
#define currentPWM						6
#define windowsSettings					7
#define oldNormalAmperage				8	
#define oldNormalVoltage				9
#define oldCurrentAmperage				10		
#define oldCurrentVoltage				11
#define	oldCurrentPWM					12
#define	oldChargeTime					13
#define	oldDischargeTime				14
#define	oldDischargeVoltage				15
#define oldDischargeAmperage			16
#define oldWindowsSettings				17

#define charge							0
#define automatic						1
#define numDischargeCharge				2
#define oldNumDischargeCharge			3
#define oldCharge						4
#define oldAutomatic					5

#define indChargeTime					0
#define indDischargeTime				1

#define SET_CHARGE			do{(PORTD) &= ~(1<<(PD6));}while(0)			
#define SET_DISCHARGE		do{(PORTD) |= (1<<(PD6));}while(0)
	
//Объявления для отладки
#define _DEBUG_BUTTONS


//Уровни напряжения/10 мВ  помех при измерении
uint16_t voltNoise[]    = { 100, //от 0 до 250мВ
	120,	//от 250 до 500 мВ
	150,	//от 250 до 500 мВ
	180,	//от 500 до 750 мВ
	200,	//от 750 до 1000 мВ
	250,	//от 1000 до 1250 мВ
	350,	//от 1250 до 1500 мВ
	380,	//от 1500 до 1750 мВ
	450,	//от 1750 до 2000 мВ
	600,	//от 2000 до 2250 мВ
650};	//от 2250 до 2500 мВ


volatile uint16_t adcResult=0;
volatile uint8_t  numberMeasure =0;
volatile uint16_t currentTime=0;
volatile uint16_t chargeTime=0;
volatile uint16_t dischargeTime=0;
volatile uint8_t time1000ms=TIME_1_SEC;
volatile uint16_t safeTimeCharge[10][2];
volatile uint8_t indDisCharge=0;


uint8_t *pCharge;


void ADC_Init();
void TIMER0_Init();
uint16_t getNormalADC(uint16_t);
void showADC(uint8_t,uint8_t,uint16_t);
void showPWM(uint8_t,uint8_t,uint16_t);
void showTime(uint8_t,uint8_t,uint16_t);
void showWinSettings(uint8_t ,uint8_t ,uint16_t );
void changePwmCharge(uint8_t,uint16_t *);
uint16_t changePWM(uint16_t,uint16_t);
uint16_t changeVauleFromButtons(uint16_t,uint16_t,uint8_t);
void showCurrentDisplay(uint8_t);
void showAndSetValue(uint8_t,volatile uint16_t *,volatile uint8_t *);
void setZeroOldValue(volatile uint16_t *,volatile uint8_t *);
void changeChargeMode(volatile uint8_t *);
void initCharge(volatile uint8_t *);

int main( void )
{		
	volatile uint8_t stateButtons=STATE_NO_BUTTONS;
	volatile uint8_t  switchMeasuring = MEASURING_VOLTAGE;
	
	
	volatile uint8_t arrVal8[oldAutomatic+1];
	pCharge=& arrVal8[charge];
	arrVal8[charge]=0;	
	arrVal8[automatic]=1;	
	arrVal8[numDischargeCharge]=2;
	
	
	volatile uint16_t arrVal[oldWindowsSettings+1];
	arrVal[normalAmperage]=120;	
	arrVal[normalVoltage]=15;
	arrVal[dischargeVoltage]=10;	
	arrVal[dischargeAmperage]=100;	
	arrVal[currentPWM]=250;
	arrVal[currentVoltage]=0;
	arrVal[currentAmperage]=0;
	arrVal[windowsSettings]=1;
	
	
	volatile uint8_t currentDisplay=DISPLAY_START;
	volatile uint8_t oldCurrentDisplay=1;
	volatile uint16_t timeResetDisplay=100;
	
	
	USART_Init();	
	
	
	//инициализируем дисплей
	DDRD|=(1<<PD6);
	initCharge(arrVal8);
	
	LCD_Init();	
	ADC_Init();
	TIMER0_Init();
	initButtons();
	initPWM();
	setPwm(arrVal[currentPWM]);		
	
	setZeroOldValue(arrVal,arrVal8);
	
	
	

	
	StartConvAdc();
	sei();
	
	
	
	
	
	while(1){	
		
		
			volatile uint8_t codeButton=PINX_BUTTON & MASK_BUTTONS;
			
			if(stateButtons!=STATE_ENTER_5SEC && codeButton!=NO_BUTTONS  ){
				stateButtons=getStateButton(codeButton);
				timeResetDisplay=currentTime+5;
				_delay_ms(250);	
				
				switch(stateButtons){
				case STATE_ENTER:
					switch(currentDisplay){
						case DISPLAY_START:currentDisplay=DISPLAY_FAST_CHANGE;break;
						case DISPLAY_FAST_CHANGE:currentDisplay=DISPLAY_START;break;
						case DISPLAY_PROGR:currentDisplay=DISPLAY_PROGR+arrVal[windowsSettings];break;
						default:currentDisplay=DISPLAY_PROGR;
					}	
					break;				
				case STATE_ENTER_5SEC:
					if(currentDisplay>DISPLAY_P0){
						currentDisplay=DISPLAY_START;						
						}else{
						currentDisplay=DISPLAY_PROGR;
					}
					break;
				default:					
					switch(currentDisplay){
					case DISPLAY_FAST_CHANGE:
						arrVal8[charge]^=1;	
						setPwm(512);//Установка в 50 % при смене режима 		
						break;
					case DISPLAY_PROGR:						
						arrVal[windowsSettings]=changeVauleFromButtons(arrVal[windowsSettings],LAST_DISPLAY,stateButtons);
						if(arrVal[windowsSettings]==(LAST_DISPLAY+1)){
							arrVal[windowsSettings]=1;
						}else if(arrVal[windowsSettings]==0){
							arrVal[windowsSettings]=1;
						}
						break;
					case DISPLAY_P0:
						arrVal[normalVoltage]=changeVauleFromButtons(arrVal[normalVoltage],240,stateButtons);
						break;
					case DISPLAY_P1:
						arrVal[dischargeVoltage]=changeVauleFromButtons(arrVal[dischargeVoltage],240,stateButtons);
						break;
					case DISPLAY_P2:
						arrVal[dischargeAmperage]=changeVauleFromButtons(arrVal[dischargeAmperage],1000,stateButtons);
						break;
					case DISPLAY_P3:
						arrVal[normalAmperage]=changeVauleFromButtons(arrVal[normalAmperage],1022,stateButtons);
						break;
					case DISPLAY_P4:
						arrVal8[automatic]^=1;
						break;
					case DISPLAY_P5:
						arrVal8[numDischargeCharge]=changeVauleFromButtons(arrVal8[numDischargeCharge],10,stateButtons);
						if(arrVal8[numDischargeCharge]==11){
							arrVal8[numDischargeCharge]=1;
						}
						break;
					}
					
				}				
				
				} else if(codeButton==NO_BUTTONS) {
					if(currentDisplay!=DISPLAY_START){
						if(currentTime>timeResetDisplay){
							currentDisplay=DISPLAY_START;
						}						
					}
					
					if(stateButtons==STATE_ENTER_5SEC){
						stateButtons=STATE_NO_BUTTONS;
					}				
				}
		
		
		if(numberMeasure>=NUMBER_OF_MEASURING){
			
			adcResult=getNormalADC(adcResult);
			
			switch (switchMeasuring)
			{				
				case MEASURING_VOLTAGE:					
					switchMeasuring=MEASURING_CURRENT;
					arrVal[currentVoltage]=adcResult;
					//Отключаем если автомат режим 
					if(arrVal8[automatic] &&						
						(arrVal[currentVoltage]<=arrVal[dischargeVoltage] || arrVal[currentVoltage]>=arrVal[normalVoltage])){
							changeChargeMode(arrVal8);																		
					}			
				break;
				
				case MEASURING_CURRENT:				
					arrVal[currentAmperage]=adcResult;				
					volatile uint8_t normal;
					if(arrVal8[automatic]){
						normal=(arrVal8[charge])?normalAmperage:dischargeAmperage;
						//Изменяем наше текущее значение ШИМ если оно не в норме
						if(arrVal[currentAmperage]!=arrVal[normal]){
							arrVal[currentPWM]=changePWM(arrVal[currentAmperage],arrVal[normal]);
						}
					}
					switchMeasuring=MEASURING_VOLTAGE;								
				break;
			}
			//Смена первым идет измерение напряжение ADC0 - напряжения
			//Потом меняем  на измерение   ADC2 -ток
			SWITCH_ADC_CHANNEL(ADMUX,CHANNEL_CURRENT);
			
			
			_delay_us(250);
			adcResult=0;
			numberMeasure=0;
			StartConvAdc();
		}
		
		
		if(oldCurrentDisplay!=currentDisplay){
			oldCurrentDisplay=currentDisplay;
			setZeroOldValue(arrVal,arrVal8);			
			showCurrentDisplay(currentDisplay);			
		}	
		
		showAndSetValue(currentDisplay,arrVal,arrVal8);
	}
	
	return 0;
}
/****************************************************************************/

void ADC_Init(){
	/*Настройка АЦП
	*Частота 62500кГц 1 такт -16мкс Результат получаем за 16*13=208мкс
	*при кол-ве измерение NUMBER_OF_MEASURING 100 получаем вывод на экран
	*каждые 0,02 сек
	*Можно и помедленей но пока удобно высчитывать помеху
	*С учетом помехи удалось измерять только до 2485 мВ =(25500-650)/100
	*Сначала меряем напряжение ADC0 pos ADC1 neg
	*/
	
	ADMUX|=(1<<ADLAR)|(1<<REFS1)|(1<<REFS0)|(1<<MUX4);
	
	ADCSRA|=(1<<ADEN)|(1<<ADIE)|(0<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	
}
/****************************************************************************/

void TIMER0_Init(){
	//Коэф деления 1024 это примерно 1 мс при 1 МГц
	TCCR0|=(1<<CS02)|(1<<CS00);
	TCNT0=0;
	TIMSK|=(1<<OCIE0);
	OCR0=TIMER0_INTERRUPT;
}
/****************************************************************************/
uint16_t getNormalADC(uint16_t adc){
	//Извлекаем из массива напряжений помех значение
	volatile uint16_t noise=voltNoise[adc/2500];
	//Проверяем если adcResult=0 и меньше уровня помехи то пропускаем
	//
	if((adcResult!=0)&&(adc>noise)){
		adc-=noise;
	}
	//Преобразуем в нормальные показания сумма/кол-во измерений
	adc/=NUMBER_OF_MEASURING;
	return adc;
	
}
/***********************Показ результата АЦП на экран*****************************************************/

void showADC(uint8_t x,uint8_t y,uint16_t value){
	
		LCD_ClearField(x,y,4);
		LCD_Goto(x,y);	
		
		if(y==0){
			LCD_SendNumber(value,'.');
			}else{
			LCD_SendNumber(value,' ');
		}		
}
/****************************************************************************/
void showPWM(uint8_t x,uint8_t y,uint16_t value){	
		
		LCD_ClearField(x,y,4);
		LCD_Goto(x,y);		
		LCD_SendNumber(getPercentPwm(),' ');
		LCD_WriteData('%');
		
}
/****************************************************************************/
void showTime(uint8_t x,uint8_t y ,uint16_t value){
		
		uint8_t sec=value%60;
		uint8_t minute=(value%3600)/60;
		uint8_t hour=value/3600;
		if(sec==0){
			LCD_ClearField(x,y,8);
		}
		LCD_Goto(x,y);
		LCD_SendNumber(hour,' ');
		LCD_WriteData(':');
		LCD_SendNumber(minute,' ');
		LCD_WriteData(':');
		LCD_SendNumber(sec,' ');
			
}
/****************************************************************************/
void showWinSettings(uint8_t x,uint8_t y ,uint16_t value){
	LCD_ClearField(x,y,16);
	LCD_Goto(x,y);
	switch(value){
		case DISPLAY_P0:
			LCD_SendStr(" U off");
			break;
		case DISPLAY_P1:
			LCD_SendStr(" U discharge");
			break;	
		case DISPLAY_P2:
			LCD_SendStr(" I discharge");
			break;
		case DISPLAY_P3:
			LCD_SendStr(" I charge");
			break;
		case DISPLAY_P4:
			LCD_SendStr(" Mode A or M");
			break;
		case DISPLAY_P5:
			LCD_SendStr(" N Disch_Charge");
			break;
	}
}
/****************************************************************************/
void changePwmCharge(uint8_t mode,uint16_t *array){
	//0 -режим разрядки 1 - зарядки	 
	volatile uint8_t normal=(mode)?normalAmperage:dischargeAmperage; 	
	
	if(array[currentAmperage]!=array[normal]){
		array[currentPWM]=changePWM(array[currentAmperage],array[normal]);
	}	
}
/****************************************************************************/
uint16_t changePWM(uint16_t adc,uint16_t normalValue){
	uint16_t pwm=0;
	GET_16BIT_RG(pwm,OCR1A);
	if(adc>normalValue){
		if(adc>0){
			pwm--;
		}
		} else if(adc<normalValue && adc>10){
		if(pwm<1023){
			pwm++;
		}
	}
	setPwm(pwm);
	return pwm;
}
/****************************************************************************/
uint16_t changeVauleFromButtons(uint16_t value,uint16_t max,uint8_t stateButton){
	switch(stateButton){
		case STATE_UP:
		if(value<=max){
			value++;
		}
		break;
		case STATE_DOWN:
		if(value>=1){
			value--;
		}
		break;	
	}
	return value;
}
/****************************************************************************/
void showCurrentDisplay(uint8_t display){
	LCD_Clear();
	
	switch(display){
		case DISPLAY_START:
			LCD_Goto(0,0);
			LCD_SendStr("V=");
			LCD_Goto(0,1);
			LCD_SendStr("I=");
			LCD_Goto(8,0);
			LCD_SendStr("M=");			
		break;
		case DISPLAY_FAST_CHANGE:
			LCD_Goto(0,0);
			LCD_SendStr("CHARGE DISCHARGE");			
		break;
		case DISPLAY_PROGR:
			LCD_Goto(0,0);
			LCD_SendStr("WINDOWS SETTING");
				
			break;
		case DISPLAY_P0:
			LCD_Goto(0,1);
			LCD_SendStr("U Normal");
		break;
		case DISPLAY_P1:
			LCD_Goto(0,1);
			LCD_SendStr("U Discharge");
		break;
		case DISPLAY_P2:
			LCD_Goto(0,0);
			LCD_SendStr("I Discharge");
		break;
		case DISPLAY_P3:
			LCD_Goto(0,0);
			LCD_SendStr("I Сharge");
			LCD_Goto(0,1);
			LCD_SendStr("Norm.I=");
		break;
		case DISPLAY_P4:
			LCD_Goto(0,0);
			LCD_SendStr("Automat or Manual");
		break;
		case DISPLAY_P5:
			LCD_Goto(0,0);
			LCD_SendStr("N DischargeCharg");
		break;
	}
}
/****************************************************************************/
void showAndSetValue(uint8_t display ,volatile uint16_t *array,volatile uint8_t *array8){	
	
	switch(display){
		
		case DISPLAY_START:
			{
				volatile uint16_t time=(array8[charge])? chargeTime:dischargeTime;
				volatile uint8_t index=(array8[charge])? oldChargeTime:oldDischargeTime;
				
				if(array[index]!=time){
					array[index]=time;
					showTime(8,1,time);
				}
			}			
			if(array[oldCurrentVoltage]!=array[currentVoltage]){
				array[oldCurrentVoltage]=array[currentVoltage];
				showADC(2,0,array[currentVoltage]);				
			}			
			if(array[oldCurrentAmperage]!=array[currentAmperage]){
				array[oldCurrentAmperage]=array[currentAmperage];
				LCD_Goto(2,1);
				volatile uint8_t ch=(array8[charge])? '+':'-';								
				LCD_WriteData(ch);				
				showADC(3,1,array[currentAmperage]);
			}
			if(array[oldCurrentPWM]!=array[currentPWM]){
				array[oldCurrentPWM]=array[currentPWM];
				showPWM(10,0,array[currentPWM]);
				setPwm(array[currentPWM]);
			}	
			if(array8[oldNumDischargeCharge]!=array8[numDischargeCharge]){
				array8[oldNumDischargeCharge]=array8[numDischargeCharge];
				LCD_Goto(14,0);
				LCD_SendNumber(array8[numDischargeCharge],' ');
			}		
		break;
		case DISPLAY_FAST_CHANGE:
			if(array8[oldCharge]!=array8[charge]){
				array8[oldCharge]=array8[charge];	
				LCD_ClearField(0,1,9);			
				LCD_Goto(0,1);
				if(array8[charge]){					
					LCD_SendStr("Charge");
					BM_ClearBit(TCCR1A,COM1A0);
					SET_CHARGE;					
				}else{
					LCD_SendStr("Discharge");
					BM_SetBit(TCCR1A,COM1A0);
					SET_DISCHARGE;
				}				
			}			
		break;
		case DISPLAY_PROGR:
			if(array[oldWindowsSettings]!=array[windowsSettings]){
				array[oldWindowsSettings]=array[windowsSettings];
				showWinSettings(0,1,DISPLAY_PROGR+array[windowsSettings]);
			}
		break;
		case DISPLAY_P0:
			if(array[oldNormalVoltage]!=array[normalVoltage]){
				array[oldNormalVoltage]=array[normalVoltage];
				showADC(0,0,array[normalVoltage]);
			}
		break;
		case DISPLAY_P1:
			if(array[oldDischargeVoltage]!=array[dischargeVoltage]){
				array[oldDischargeVoltage]=array[dischargeVoltage];
				showADC(0,0,array[dischargeVoltage]);
			}
		break;
		case DISPLAY_P2:
			if(array[oldDischargeAmperage]!=array[dischargeAmperage]){
				array[oldDischargeAmperage]=array[dischargeAmperage];
				showADC(0,1,array[dischargeAmperage]);
			}
		break;
		case DISPLAY_P3:
			if(array[oldNormalAmperage]!=array[normalAmperage]){
				array[oldNormalAmperage]=array[normalAmperage];
				showADC(7,1,array[normalAmperage]);
			}
		break;
		case DISPLAY_P4:
			if(array8[oldAutomatic]!=array8[automatic]){
				array8[oldAutomatic]=array8[automatic];
				LCD_ClearField(0,1,9);
				LCD_Goto(0,1);
				if(array8[automatic]){
					LCD_SendStr("Automatic");				
					} else {
					LCD_SendStr("Manual");				
				}
			}
		break;
		case DISPLAY_P5:
			if(array8[oldNumDischargeCharge]!=array8[numDischargeCharge]){
				array8[oldNumDischargeCharge]=array8[numDischargeCharge];
				LCD_ClearField(0,1,3);
				LCD_Goto(0,1);
				LCD_SendNumber(array8[numDischargeCharge],' ');
			}
		break;
	}
}
/****************************************************************************/
void setZeroOldValue(volatile uint16_t *array,volatile uint8_t *array8){
	for (int i=oldNormalAmperage;i<=oldWindowsSettings;i++)	{
		array[i]=0;
	}
	array8[oldCharge]=(array8[charge])? 0:1;
	array8[oldAutomatic]=(array8[automatic])? 0:1;
	array8[oldNumDischargeCharge]=0;
	
}
/****************************************************************************/
void changeChargeMode(volatile uint8_t *array8){
	
	if(array8[numDischargeCharge]!=0){
		//Если кол-во итераций разрядки - зарядки не достигло 0
		if(array8[charge]) {
			//Если режим зарядки
			safeTimeCharge[indDisCharge][indChargeTime]=chargeTime;
			chargeTime=0;
			array8[numDischargeCharge]--;			
			
			USART_SendChar('N');
			sendDigitalUsart(indDisCharge+1);
			USART_SendChar('C');
			sendDigitalUsart(safeTimeCharge[indDisCharge][indChargeTime]);
			USART_SendChar('D');
			sendDigitalUsart(safeTimeCharge[indDisCharge][indDischargeTime]);
			indDisCharge++;
			
			if(array8[numDischargeCharge]==0){
				setPwm(0);
			}
			
		} else {
			safeTimeCharge[indDisCharge][indDischargeTime]=dischargeTime;
			dischargeTime=0;
		}
		
		array8[charge]^=1;
		initCharge(array8);
	}
}
/****************************************************************************/
void initCharge(volatile uint8_t *array8){
	if(array8[charge]){
		BM_ClearBit(TCCR1A,COM1A0);
		SET_CHARGE;
		}else{
		BM_SetBit(TCCR1A,COM1A0);
		SET_DISCHARGE;
	}
}
/****************************************************************************/

ISR(ADC_vect)
{
	volatile uint16_t temp=0;
	/*Расчет АЦП 8 -бит 255
	* Так как ADLAR=1  то результат сдвигается вправо
	* Измеряем дифф.входами ADC0=pos ADC1-neg
	*/
	temp=ADCL>>7;
	temp|=ADCH<<1;
	
	if(BM_BitIsSet(ADCH,7)){
		//Заходим когда измеряем отрицательное напряжение на Сопр шунта 
		//Например U=100 мВ на шунте при дифф каналах и Uop=2560 мВ 1дел=5 мВ при 9бит , 8бит 10мВ
		//Получаем 100/5=20 в представлении 10-битном 11 1110 1100 
		//У нас левое выравнивание ADCL=0000 0000 temp=ADCL>>7 temp=0
		//ADCH = 1111 1011 старший бит ответ за знак 1 знчит минус
		//temp|=ADCH<<1 = 1 1111 0110  это -10 9 битном представлении		
				
		temp^=0x1FF;
		temp++;							
	} 
	
	adcResult+=temp;
	
	numberMeasure++;
	
	if(numberMeasure<NUMBER_OF_MEASURING){
		StartConvAdc();
	}
	
}
ISR(TIMER0_COMP_vect)
{
	time1000ms--;
	if(time1000ms==0){
		time1000ms=TIME_1_SEC;
		currentTime++;
		if(*pCharge){
			chargeTime++;
		} else{
			dischargeTime++;
		}
	}
	
	
}