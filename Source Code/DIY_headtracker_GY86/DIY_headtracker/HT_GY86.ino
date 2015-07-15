/*
HT_GY86 v 1.0
Arduino-based headtracker for GY-86 board (MPU6050 + HMC5885L)
based on HATIRE sketch and Mahony algorithm implementation by Davide Gironi

All calibration code is deliberately removed in favor of MPU6050 self-clibration feature.
After turning on FTnoIR wait 30-40 seconds until frame stops moving, then use 'center' feature of FTnoIR.

*/
#include <Wire.h>

#include "mpu6050.h"
#include "hmc5883l.h"


#define SERIAL_BAUD 115200
#define  ARDUINO_LED 13
int pushButton = 2;

typedef struct {
  int16_t  Begin;
  uint16_t Cpt;
  float    gyro[3];   // 12 [Y, P, R]    gyro
  float    acc[3];    // 12 [x, y, z]    Acc
  int16_t  End ;      
} 
_hat;

typedef struct  {
  int16_t Begin  ;   
  uint16_t Code ;     // 2  Code info
  char     Msg[24];   // 24 Message
  int16_t End ;      
} 
_msginfo;

// runtime global variables

_msginfo msg;
_hat hat;

char mag_present=0;
int cntr=0;

long *ptr = 0;
double qw = 1.0f;
double qx = 0.0f;
double qy = 0.0f;
double qz = 0.0f;
double roll = 0.0f;
double pitch = 0.0f;
double yaw = 0.0f;

uint8_t sendData=1;
volatile uint8_t dataAvailable=0;


typedef void (*_update_quat)();
_update_quat update_quat;

// structured messaging
void PrintCodeSerial(uint16_t code,char Msg[24],bool EOL ) {
  msg.Code=code;
  memset(msg.Msg,0x00,24);
  strcpy(msg.Msg,Msg);
  if (EOL) msg.Msg[23]=0x0A;
  // Send HATIRE message to  PC
   Serial.write((byte*)&msg,30);
}


/*
 * update timer
 */
ISR(TIMER1_COMPA_vect) {
  update_quat();
  digitalWrite(ARDUINO_LED, digitalRead(ARDUINO_LED) ^ 1);
  dataAvailable=1;
}

// process input from serial
void serialEvent(){
  char cmd = (char)Serial.read();
  switch (cmd) {
  case 'S':
    PrintCodeSerial(5001,"HAT START",true);
    sendData=1;
    break;      

  case 's':
    PrintCodeSerial(5002,"HAT STOP",true);
    sendData=0;
    break;      
  default:
    break;
  }
}

// the setup routine runs once when you press reset:
void setup() {
  hat.Begin = 0xAAAA;
  hat.Cpt = 0;
  hat.End = 0x5555;
  hat.acc[0]=0;
  hat.acc[1]=0;
  hat.acc[2]=0;

  msg.Begin = 0xAAAA;
  msg.Code = 0;
  msg.End = 0x5555;

  Serial.begin(SERIAL_BAUD);

  
  pinMode(9,OUTPUT);
  digitalWrite(2,HIGH);
  digitalWrite(3,HIGH);  

  // Set all other pins to input, for safety.
  pinMode(0,INPUT);
  pinMode(1,INPUT);
  pinMode(2,INPUT);
  pinMode(3,INPUT);
  pinMode(6,INPUT);
  pinMode(7,INPUT);  
  pinMode(8,INPUT);    

  digitalWrite(0,LOW); // pull-down resistor
  digitalWrite(1,LOW);
  digitalWrite(2,HIGH);
  digitalWrite(3,HIGH);  
  
  pinMode(ARDUINO_LED,OUTPUT);    // Arduino LED
  // make the pushbutton's pin an input:
  pinMode(pushButton, INPUT);
  
  Wire.begin();               // Start I2C

  PrintCodeSerial(2001,"MPU init",true);
  mpu6050_init();
  delay(50);

 PrintCodeSerial(2002,"Enabling bypass",true);
  mpu6050_enable_bypass();

  delay(200);
  mag_present=hmc5883l_detect();
  if(mag_present=='H') {
    PrintCodeSerial(2003,"Magnetometer found",true);
    hmc5883l_init();
    update_quat=&imu10dof01_updateQuaternion;
  } else {
    update_quat=&mpu6050_updateQuaternion;
    PrintCodeSerial(2004,"No magnetometer found",true);
  }
  delay(200);
  Serial.println("Timer init");
  delay(100);

  TIMSK1 &= (1 << TOIE1); // disable timer overflow interrupt
  TIMSK1 &= (1 << OCIE1A); // disable timer compare interrupt
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 256;// 61Hz
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
//  TIMSK1 |= (1 << TOIE1); // enable timer overflow interrupt

}

// the loop routine runs over and over again forever:
void loop() {
  // workeraound for serialEvent bug
  if(Serial.available() > 0)  serialEvent();
  // read the input pin:
  int buttonState = digitalRead(pushButton);

  mpu6050_getQuaternion(&qw, &qx, &qy, &qz);
  mpu6050_getRollPitchYaw(&roll, &pitch, &yaw);
  hat.gyro[0]=yaw*57.2957795;
  hat.gyro[1]=pitch*57.2957795;
  hat.gyro[2]=roll*57.2957795;
  cntr= cntr >= 999 ? 0 : cntr+1;
  hat.Cpt=cntr;
  if((sendData==1) && (dataAvailable==1)) {
    Serial.write((byte*)&hat,30);
    dataAvailable=0;
  }
  delay(10);
}

