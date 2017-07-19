//https://www.appelsiini.net/2011/simple-usart-with-avr-libc

#define F_CPU 16000000UL
#define BAUD 9600
#include "stdio.h"

#include <util/setbaud.h>
char uart_getchar(void);
void uart_putchar(char c);
void uart_init(void);
void send_string(char s[]);
int Command(char s[]);
int uartRxTest(void);
int Input[]={0,0,0,0,0,0,0,0};
int InputVal=0;

void SerialPrint(char s[]);

void PobDuinoInit(void);
int DigitalRead(int Channel);					//DIGITALREAD,Channel (4-8)=>0/1
int MovRead(void);								//MOVREAD =>0/1
int AnalogRead(int Channel);					//ANALOGREAD,Channel(0-3)=>0-100
int DistRead(int Channel);						//DISTREAD,Channel(0-3)=>D(cm)
void UsRead(int Channel);						//USREAD,Channel(4-7)=>D(cm)
void Servo(int N, int Angle, int Speed); 		//SERVO,Channel(0-15),Degres(0-180),Speed(0-31/32)
void Motor(char Side, char Sens, int Speed);	//MOTOR,Side(L/R),Sens(F/R),Speed(0-100)
void Move(char Sens, int Dist, int Speed);		//MOVE,Sens(F/R),Dist(mm 0-30000),Speed(0-100)
void Stop(void);								//STOP
void Turn(char Sens, int Angle, int Speed);		//TURN,Sens(L/R),Angle(1-360),Speed(0-100)
void Rotation(char Sens, int Angle);			//ROTATE,Sens(L/R),Angle(1-360),Speed(0-100)