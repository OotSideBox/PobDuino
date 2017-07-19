#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SparkFunMiniMoto.h>
//attention! nécessite une capa découplage sur alim (essais OK avec 1000uF)
//http://wiki.seeed.cc/Grove-Ultrasonic_Ranger/
#include <Ultrasonic.h>
#include "Metro.h"     //Include Metro library

#define MONITOR

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)
#define SERVO_INIT 90 //position angulaire initiale des servos en degrés 
#define CR 0x0D
#define LF 0x0A
#define SPLITTERS 10
#define SPLIT ','

#define WHEELPULSE_L 2
#define WHEELPULSE_R 3
#define ADC_REF 5
#define LED_BOOT 13
#define CMD_MOVE 1
#define CMD_ROTATE 2

char RecievedChar;
String ReceivedString = "";
String Commande[SPLITTERS];
int SplitIndex[SPLITTERS];
bool CommandeFlag = false;
volatile int MotPulseLeft, MotPulseRight;
float CompMotR=1;
float CompMotL=1;
int Distance;
float Voltage;
int ServoNum;
int ServoVal;
int ServoDegres;
struct sServo{int CurrentPos; int Consigne;} Servo[16];
int ServoSpeed=33;
volatile bool MovingL=false;
volatile bool MovingR=false;
volatile bool CmdMoveL=false;
volatile bool CmdMoveR=false;
volatile int EndMove=0;
volatile int CmdMoveRotate=0;


Metro ServoTempo = Metro(1);
//Ultrasonic ultrasonic(7);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0X40);
// Create two MiniMoto instances, with different address settings.
MiniMoto motor0(0xC4); // A1 = 1, A0 = clear
MiniMoto motor1(0xC0); // A1 = 1, A0 = 1 (default)

//************************************************
//roue codeuse = 24 dents
void FourcheOptLeft()
{
 noInterrupts();
  if(MotPulseLeft>0) MotPulseLeft--;
  if(MotPulseLeft==0) 
  {
    motor0.brake();
    MotPulseLeft=-1;
    EndMove--;
    switch(CmdMoveRotate)
    {
      case CMD_MOVE:
            motor1.brake();
            MotPulseRight=-1;
            EndMove=0;
            ResultSend(0);
      break;
      case CMD_ROTATE:
            if(EndMove==0) ResultSend(0);
      break;
    }
  }
 
interrupts();  
  
}
//*************************************************************
void FourcheOptRight()
{
  noInterrupts();
  if(MotPulseRight>0) MotPulseRight--;
  if(MotPulseRight==0) 
  {
    motor1.brake();
    MotPulseRight=-1;
    EndMove--;
    switch(CmdMoveRotate)
    {
      case CMD_MOVE:
            motor0.brake();
            MotPulseLeft=-1;
            EndMove=0;
            ResultSend(0);
      break;
      case CMD_ROTATE:
            if(EndMove==0) ResultSend(0);
      break;
    }
  }
 interrupts();
 
}
//********************************************************************
void setup()
{
  pinMode(WHEELPULSE_L, INPUT_PULLUP);
  pinMode(WHEELPULSE_R, INPUT_PULLUP);

  Serial.begin(9600);
  Serial.println("Hello, World!");
  pwm.begin();
  pwm.setPWMFreq(60);
  Serial.println("I2C Servo/PWM started");
  motor0.stop();
  motor1.stop();
  Serial.println("I2C Motor Driver Started");
  #ifdef MONITOR
  Serial.println("PobDuino Interpreter - By JNL - OOTSIDEBOX 05/26/2017");
  Serial.println();
  Serial.println("DIGITALREAD,Channel (4-8)=>0/1");
  Serial.println("DIGITALWRITE,Channel(8-15),Value(0-100)");
  Serial.println("ANALOGREAD,Channel(0-3)=>0-100");
  Serial.println("DISTREAD,Channel(0-3)=>D(cm)");
  Serial.println("USREAD,Channel(4-7)=>D(cm)");
  Serial.println("MOVREAD,=>0/1");
  Serial.println("SERVO,Channel(0-7),Degres(0-180),Speed(0-31/32)");
  Serial.println("MOTOR,Side(L/R),Sens(F/R),Speed(0-100)");
  Serial.println("MOVE,Sens(F/R),Dist(mm 0-30000),Speed(0-100)");
  //Serial.println("TURN,Sens(L/R),Angle(1-360),Speed(0-100)");
  Serial.println("ROTATE,Sens(L/R),Angle(1-360)");
  Serial.println("STOP");
  Serial.println();
  #endif
  attachInterrupt(0, FourcheOptLeft, RISING);
  attachInterrupt(1, FourcheOptRight, RISING);
  for (int ServoNum=0; ServoNum<8; ServoNum++)
  {
    ServoVal = map(SERVO_INIT, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(ServoNum, 0, ServoVal);
    Servo[ServoNum].CurrentPos=SERVO_INIT;
    Servo[ServoNum].Consigne=SERVO_INIT;
  }
  
}
//********************************************************************
void ServoDrive()
{
  int S;
  for (S=0; S<8; S++)
  {
    if(Servo[S].CurrentPos < Servo[S].Consigne) 
    {
      Servo[S].CurrentPos++;
       ServoVal = map(Servo[S].CurrentPos, 0, 180, SERVOMIN, SERVOMAX);
       pwm.setPWM(S, 0, ServoVal);
    }
    if(Servo[S].CurrentPos > Servo[S].Consigne) 
    {
      Servo[S].CurrentPos--;
      ServoVal = map(Servo[S].CurrentPos, 0, 180, SERVOMIN, SERVOMAX);
      pwm.setPWM(S, 0, ServoVal);
    }
   
  }
}

//************************************************
void CommandExec()
{

  int SensorValue;
  int SensorNum;
  String MotorSide;
  int MotorSpeed;
  String MotorSens;
  float ConsigneMM;
  int ConsignePulse;


  // USREAD,Channel **************************************
  if (Commande[0] == "USREAD")
  {
    noInterrupts();
    SensorNum = Commande[1].toInt();
    Ultrasonic ultrasonic(SensorNum);
    Distance=ultrasonic.MeasureInCentimeters();
    ResultSend(Distance);
  }

  // DIGITALREAD,Channel **************************************
  if (Commande[0] == "DIGITALREAD")
  {
    noInterrupts();
    SensorNum = Commande[1].toInt();
    SensorValue = digitalRead(SensorNum);
    if(SensorValue) ResultSend(1);
    else ResultSend(0);
  }
  // MOVREAD **************************************
  if (Commande[0] == "MOVREAD")
  {
    int Temp=EndMove;
    ResultSend(Temp);
  }
  // ANALOGREAD,Channel **************************************
  if (Commande[0] == "ANALOGREAD")
  {
    noInterrupts();
    SensorNum = Commande[1].toInt();
    switch (SensorNum)
    {
      case 0: SensorValue  = analogRead(A0);
        break;
      case 1: SensorValue  = analogRead(A1);
        break;
      case 2: SensorValue  = analogRead(A2);
        break;
      case 3: SensorValue  = analogRead(A3);
        break;
    }
    SensorValue = map(SensorValue, 0, 1023, 0, 100);
    ResultSend(SensorValue);
  }

  // DISTREAD,Channel **************************************
  if (Commande[0] == "DISTREAD")
  {
    noInterrupts();
    SensorNum = Commande[1].toInt();
    Voltage = getVoltage(SensorNum); 
    Distance=LinearInterpol_GP2Y0A21(Voltage);
    ResultSend(Distance);
  }
  
  // DIGITALWRITE,Channel,Value(0-100) **************************************
  if (Commande[0] == "DIGITALWRITE")
  {
    ServoNum = Commande[1].toInt();
    Servo[ServoNum].Consigne= Commande[2].toInt();
    
    // Sets pin without having to deal with on/off tick placement and properly handles
    // a zero value as completely off.  Optional invert parameter supports inverting
    // the pulse for sinking to ground.  Val should be a value from 0 to 4095 inclusive.
    //void Adafruit_PWMServoDriver::setPin(uint8_t num, uint16_t val, bool invert)

    ServoVal = map(Servo[ServoNum].Consigne, 0, 100, 0, 4095);
    pwm.setPin(ServoNum, ServoVal, 0);
    ResultSend(Servo[ServoNum].Consigne);
  }
 
  
  // SERVO,Channel,Degres **************************************
  if (Commande[0] == "SERVO")
    //https://learn.adafruit.com/16-channel-pwm-servo-driver/using-the-adafruit-library
  {
    ServoNum = Commande[1].toInt();
    ServoSpeed=Commande[3].toInt();
    if(ServoSpeed<32) ServoTempo.interval(32-ServoSpeed);
    Servo[ServoNum].Consigne= Commande[2].toInt();
    if(ServoSpeed >31)
    {
      Servo[ServoNum].CurrentPos=Servo[ServoNum].Consigne ;
      ServoVal = map(Servo[ServoNum].CurrentPos, 0, 180, SERVOMIN, SERVOMAX);
      pwm.setPWM(ServoNum, 0, ServoVal);
    }
    ResultSend(Servo[ServoNum].Consigne);
  }
 
  // MOTOR,Side,Sens,Speed **************************************
  if (Commande[0] == "MOTOR")
  {
    noInterrupts();
    MotorSide = Commande[1];
    MotorSens = Commande[2];
    MotorSpeed = Commande[3].toInt();

    MotPulseRight=-1;
    MotPulseLeft=-1;
    
    if (MotorSide == "L")
    {
      if (MotorSpeed == 0) motor0.brake();
      if (MotorSens == "F") motor0.drive(-MotorSpeed);
      if (MotorSens == "R") motor0.drive(MotorSpeed);
    }
    if (MotorSide == "R")
    {
      if (MotorSpeed == 0) motor1.brake();
      if (MotorSens == "F") motor1.drive(MotorSpeed);
      if (MotorSens == "R") motor1.drive(-MotorSpeed);
    }
    ResultSend(MotorSpeed);
  }

 // MOVE,Sens,mm,Speed **************************************
 // diametre roues = 65mm => circonférence = 204.2mm
 // roue codeuse = 24 dents
 // 1 pulse = 204.2/24 = 8.5mm
 // consigne en pulses = consigne mm / 8.5
 
 if (Commande[0] == "MOVE")
  {
    noInterrupts();
    CmdMoveL=true;
    CmdMoveR=true;
    
    MotorSens = Commande[1];
    ConsigneMM = (float)Commande[2].toInt();
    ConsignePulse= (int)(ConsigneMM/8.1);
    
    //noInterrupts();
    CmdMoveRotate=CMD_MOVE;
    MotPulseRight= ConsignePulse;
    MotPulseLeft= ConsignePulse;
    EndMove=2;
    //interrupts();
    MotorSpeed = Commande[3].toInt();
    if(Commande[2].toInt()>0)
    {
      if (MotorSpeed == 0) 
      {
        motor0.brake();
        motor1.brake();
      }
      
      int tempR=(int)((float)MotorSpeed * CompMotR);
      int tempL=(int)((float)MotorSpeed * CompMotL);
      if (MotorSens == "F") 
      {
        motor1.drive(tempR);
        motor0.drive(-tempL);
      }
      if (MotorSens == "R") 
      {
        motor1.drive(-tempR);
        motor0.drive(tempL);
      }
    }
    //ResultSend(MotorSpeed);
  
  }
  //********************************************************
  if (Commande[0] == "STOP")
  {
        noInterrupts();
        motor0.brake();
        motor1.brake();
        MotPulseRight= 0;
        MotPulseLeft= 0;
        ResultSend(0);
  }


  //ROTATE,Sens(L/R),Angle **************************************
  if (Commande[0] == "ROTATE") //rotation avec les 2 moteurs en sens inverse
  {
    noInterrupts();
    CmdMoveL=true;
    CmdMoveR=true;
    MotorSens = Commande[1];
    ConsigneMM =(float)map(Commande[2].toInt(), 0, 360, 0, 637);
    ConsignePulse= (int)(ConsigneMM/8.1);
    if(ConsignePulse<1) ConsignePulse=1;
    //noInterrupts();
    CmdMoveRotate=CMD_ROTATE;
    MotPulseRight= ConsignePulse;
    MotPulseLeft= ConsignePulse;
    EndMove=2;
    //interrupts();
    
    MotorSpeed=100;
    
    if(MotorSens=="L")
    {
      motor1.drive(MotorSpeed);
      motor0.drive(MotorSpeed);
    }
    if(MotorSens=="R")
    {
      motor1.drive(-MotorSpeed);
      motor0.drive(-MotorSpeed);
    }
    //ResultSend(MotorSpeed);
  }
interrupts();
}
//********************************************************************
void ResultSend(unsigned int Result)
{
  
  Serial.print(">");
  if(Result>999) Result=0;
  
    if (Result < 100) Serial.print("0");
    if (Result < 10)Serial.print("0");
    Serial.println(Result);
  
}

//********************************************************************
void serialEvent()
{
  if (Serial.available())
  {
    RecievedChar = Serial.read();
    if (RecievedChar == CR)
    {
      //Serial.print("ReceivedString: ");
      //Serial.println(ReceivedString);
      String_Split(SPLIT);
      CommandeFlag = true;
    }
    else ReceivedString += RecievedChar;
  }
}

//********************************************************************
void String_Split(char Splitter)
{
  int i;
  int Index = 0;
  int Nsubs;
  for (i = 0; i < SPLITTERS; i++)
  {
    SplitIndex[i] = ReceivedString.indexOf(Splitter, Index);
    Index = SplitIndex[i] + 1;
    if (!Index) break;
  }
  Nsubs = i + 1;
  Index = -1;
  for (i = 0; i < Nsubs; i++)
  {
    Commande[i] =  ReceivedString.substring(Index + 1, SplitIndex[i]);
    //Serial.print("Sub ");
    //Serial.print(i);
    //Serial.print(": ");
    //Serial.println(Commande[i]);
    Index = SplitIndex[i];
  }
  for (i = 0; i < Nsubs; i++)
  {
    //Serial.print("Int ");
    //Serial.print(i);
    //Serial.print("= ");
    //Serial.println(Commande[i].toInt());
  }
  ReceivedString = "";
  for (i = 0; i < SPLITTERS; i++)
  {
    SplitIndex[i] = -1;
  }
}
//**********************************************************************
 /****************************************************************************/
    /*Function: Get voltage from the sensor pin that is connected with analog pin*/
    /*Parameter:-void                                                       */
    /*Return:   -float,the voltage of the analog pin                        */
    float getVoltage(int Channel)
    {
        int sensor_value;
        int sum=0; 
        int Port;
        int Iterations=3;
       
        // read the analog in value:
        for (int i = 0;i < Iterations;i ++)
        {
          switch(Channel)
          {
            case 0: sensor_value = analogRead(A0);
            break; 
            case 1: sensor_value = analogRead(A1);
            break; 
            case 2: sensor_value = analogRead(A2);
            break; 
            case 3: sensor_value = analogRead(A3);
            break; 
        }
        sum += sensor_value;      
        }
        sensor_value = sum / Iterations;
        float voltage;
        voltage = (float)sensor_value*ADC_REF/1024;
        return voltage;
    }
//***************************************************************************
int LinearInterpol_GP2Y0A21(float Voltage)
{
  int Dist=80;
  int V=(int)(Voltage*100);
  
  if (V>=40)  Dist=map(V,40,45,80,70);
  if (V>=45)  Dist=map(V,45,51,70,60);
  if (V>=51)  Dist=map(V,50,61,60,50);
  if (V>=61)  Dist=map(V,61,74,50,40);
  if (V>=74)  Dist=map(V,74,92,40,30);
  if (V>=92)  Dist=map(V,92,108,30,25);
  if (V>=108) Dist=map(V,108,130,25,20);
  if (V>=130) Dist=map(V,130,165,20,15);
  if (V>=165) Dist=map(V,165,230,15,10);
  if (V>=230) Dist=map(V,230,275,10,8);
  if (V>=275) Dist=map(V,275,298,8,7);
  if (V>=298) Dist=map(V,298,310,7,6);
  if (V>=310) Dist=6;
  
  return(Dist);
}
//********************************************************************
void loop()
{

  if (CommandeFlag)
  {
    CommandExec();
    CommandeFlag = false;
  }
  if (ServoTempo.check() == 1) 
  {
    ServoDrive();
  }
  /*
  noInterrupts();
  
  if((!MovingL || !MovingR)&& CmdMove) 
  {
    CmdMove=false;
    MotPulseRight= 0;
    MotPulseLeft= 0;
    motor0.brake();
    motor1.brake();
  }
  interrupts(); */
}

