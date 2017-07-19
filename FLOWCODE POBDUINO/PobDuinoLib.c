//http://www.avrfreaks.net/forum/sending-strings-over-uart
//https://www.appelsiini.net/2011/simple-usart-with-avr-libc

void PobDuinoInit(void)
{
	uart_init();
	FCI_DELAYBYTE_S(1);
}

void uart_init(void) {
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

#if USE_2X
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= ~(_BV(U2X0));
#endif

    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */
}

void uart_putchar(char c) {
    loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
    UDR0 = c;
}

//You can receive data from UART by reading a byte from USART Data Register UDR0. USART Receive Complete RXC0 flag is set if to unread data exists in data register.

char uart_getchar(void) 
{
    loop_until_bit_is_set(UCSR0A, RXC0); /* Wait until data exists. */
	return UDR0;
	//if(bit_is_set(UCSR0A, RXC0)) return (int)UDR0;
	//else return -1;
}

int uartRxTest(void)
{
	if(bit_is_set(UCSR0A, RXC0)) return 1;
	else return 0;
}

void SerialPrint(char s[])
{
	int i =0;
	while (s[i] != '\0')
	{
		uart_putchar(s[i]);
		i++;
	}
	uart_putchar('\r');
}

int Command(char s[])
{
	int i =0;
	char AnalogRx[4];
	char Trash;
	int Analog=0;
	
	// Sortie
        // Sortie: true -> B5
        FCP_SET(B, B, 0x20, 5, (FCV_TRUE));
	
	//while (uartRxTest()) Trash=uart_getchar();
	
	while (s[i] != '\0')
	{
		uart_putchar(s[i]);
		i++;
	}
	//uart_putchar('\n');
	uart_putchar('\r');
	
	 // Sortie
     // Sortie: false -> B5
     FCP_SET(B, B, 0x20, 5, (FCV_FALSE));
	
	while(uart_getchar()!='>');
	
	AnalogRx[0]=uart_getchar();
	AnalogRx[1]=uart_getchar();
	AnalogRx[2]=uart_getchar();
	AnalogRx[3]='\0';
	
	Analog = atoi(AnalogRx);
	
	
	 
	return(Analog);
	
}
//*************************************
void DigitalWrite(int N, int Value) //DIGITALWRITE,Channel(8-15),Value(0-100)
{
	char CommandLine[]="DIGITALWRITE,NN,100";
	sprintf(CommandLine, "DIGITALWRITE,%d,%d",N,Value);
	Command(CommandLine);
	FCI_DELAYBYTE_MS(1);
}

//*************************************
void Servo(int N, int Angle, int Speed) //SERVO,Channel(0-15),Degres(0-180),Speed(0-31/32)
{
	char CommandLine[]="SERVO,NN,180,255\0";
	sprintf(CommandLine, "SERVO,%d,%d,%d",N,Angle,Speed);
	Command(CommandLine);
	FCI_DELAYBYTE_MS(1);
}
//*************************************
int AnalogRead(int Voie) //ANALOGREAD,Channel(0-3)=>0-100
{
	FCI_DELAYBYTE_MS(1);
	char CommandLine[]="ANALOGREAD,NN\0";
	sprintf(CommandLine,"ANALOGREAD,%d",Voie );
	if(Voie<=3) Input[Voie]=Command(CommandLine);
	return(Input[Voie]);
	
}
//*************************************
int DistRead(int Voie)	//DISTREAD,Channel(0-3)=>D(cm)
{
	FCI_DELAYBYTE_MS(1);
	char CommandLine[]="DISTREAD,NN\0";
	sprintf(CommandLine,"DISTREAD,%d",Voie );
	if(Voie<=3) Input[Voie]=Command(CommandLine);
	return(Input[Voie]);
	
}
void UsRead(int Voie) //USREAD,Channel(4-7)=>D(cm)
{
	FCI_DELAYBYTE_MS(1);
	char CommandLine[]="USREAD,NN\0";
	sprintf(CommandLine,"USREAD,%d",Voie );
	if((Voie<=7)&&(Voie>=4)) Input[Voie]=Command(CommandLine);
	
}
//*************************************
int DigitalRead(int Voie)	//DIGITALREAD,Channel (4-8)=>0/1
{
	FCI_DELAYBYTE_MS(1);
	char CommandLine[]="DIGITALREAD,NN\0";
	sprintf(CommandLine,"DIGITALREAD,%d",Voie );
	if((Voie<=8)&&(Voie>=4)) Input[Voie]=Command(CommandLine);
	return(Input[Voie]);

}
//*************************************

int MovRead(void)	//MOVREAD =>0/1
{
	FCI_DELAYINT_MS(1000);
	//FCI_DELAYBYTE_MS(100);
	char CommandLine[]="MOVREAD\0";
	sprintf(CommandLine,"MOVREAD");
	Input[0]=Command(CommandLine);
	return(Input[0]);
	
}
//*************************************
void Motor(char Side, char Sens, int Speed)	//MOTOR,Side(L/R),Sens(F/R),Speed(0-100)
{
	char CommandLine[]="MOTOR,G,F,1000\0";
	sprintf(CommandLine,"MOTOR,%c,%c,%d",Side,Sens,Speed);
	Command(CommandLine);
	FCI_DELAYBYTE_MS(1);
}
//*************************************
void Move(char Sens, int Dist, int Speed)	//MOVE,Sens(F/R),Dist(mm 0-30000),Speed(0-100)
{
	char CommandLine[]="MOVE,F,10000,100\0";
	sprintf(CommandLine,"MOVE,%c,%d,%d",Sens,Dist,Speed);
	Command(CommandLine);
	//FCI_DELAYBYTE_MS(200);
	//while(MovRead());
}
//*************************************
void Stop(void)	//STOP
{
	char CommandLine[]="STOP\0";
	sprintf(CommandLine,"STOP");
	Command(CommandLine);
	FCI_DELAYBYTE_MS(1);
}
//*************************************
void Halt(void)	
{
	char CommandLine[]="STOP\0";
	sprintf(CommandLine,"STOP");
	Command(CommandLine);
	FCI_DELAYBYTE_MS(1);
	while(1);
}

void Rotation(char Sens, int Angle)	//ROTATE,Sens(L/R),Angle(1-360)
{
	float Temp;
	Temp=(float)Angle*0.98;
	char CommandLine[]="ROTATE,R,360\0";
	sprintf(CommandLine,"ROTATE,%c,%d",Sens,(int)Temp);
	Command(CommandLine);
	//FCI_DELAYBYTE_MS(50);
	//while(MovRead());
}