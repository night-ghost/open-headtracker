/*
HT_GY86 v 1.0
Arduino-based headtracker for GY-86 board (MPU6050 + HMC5885L)
based on HATIRE sketch and Mahony algorithm implementation by Davide Gironi

All calibration code is deliberately removed in favor of MPU6050 self-clibration feature.
After turning on FTnoIR wait 30-40 seconds until frame stops moving, then use 'center' feature of FTnoIR.

*/
#include <Wire.h>

#include "mpu6050.h"
#include "AK8975.h"

#define SERIAL_BAUD 115200
#define  ARDUINO_LED 13
int pushButton = 2;
bool MagPresent=false;

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
AK8975 mag2(0x0E);

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

/*
extern int16_t aax;
extern int16_t aay;
extern int16_t aaz;
extern int16_t agx;
extern int16_t agy;
extern int16_t agz;
*/
extern float grx, gry, grz;

#define HP_FACTOR 5

float ax, ay, az;
float x=0,y=0,z=0;
int16_t avgx[HP_FACTOR];
int16_t avgy[HP_FACTOR];
int16_t avgz[HP_FACTOR];
int avgp=0;

float vx0=0, vy0=0, vz0=0;
float vx=0, vy=0, vz=0;
float sx=0, sy=0, sz=0;

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

void resetValues() {
  x=0; y=0; z=0;
  vx=0; vy=0; vz=0;
  sx=0; sy=0; sz=0;
  vx0=0; vy0=0; vz0=0;
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
  case 'R':
    PrintCodeSerial(5001,"HAT RESET",true);
    resetValues();
    break;      
  case 's':
    PrintCodeSerial(5002,"HAT STOP",true);
    sendData=0;
    break;      
  case 'I':
    if(MagPresent==true) {
      PrintCodeSerial(5002,"Using magnetometer",true);
    } else {
      PrintCodeSerial(5002,"No magnetometer",true);
    }
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
  mag2.initialize();
  delay(200);

  if(mag2.testConnection()) {
    MagPresent=true;
    PrintCodeSerial(2003,"Magnetometer found",true);
//    hmc5883l_init();
    update_quat=&updateQuaternion_mahony2;
  } else {
    MagPresent=false;
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


void updateOffset() {

  float anorm;
  getAccel(&ax, &ay, &az);
/*
  int i;
  avgx[avgp]=ax;
  avgy[avgp]=ay;
  avgz[avgp]=az;
  if(avgp < (HP_FACTOR-1))
    avgp++;
  else
    avgp=0;

  x=0;y=0;z=0;
  for(i=0;i<HP_FACTOR;i++) {
    x+=avgx[i];
    y+=avgy[i];
    z+=avgz[i];
  }
  x=x/HP_FACTOR;
  y=y/HP_FACTOR;
  z=z/HP_FACTOR;
*/

  x=x+(ax-x)*0.3;
  y=y+(ay-y)*0.3;
  z=z+(az-z)*0.3;
/*
  x=(x+ax)/2;
  y=(y+ay)/2;
  z=(z+az)/2;
*/
  anorm = sqrt(x*x + y*y + z*z);

  if(anorm > 16384.0)
    anorm=16384.0;

  x=(x-anorm*grx*2);///8192.0;
  y=(y-anorm*gry*2);///8192.0;
  z=(z-anorm*grz*2);///8192.0;
 
  vx=x*0.61;
  vy=y*0.61;
  vz=z*0.61;

  sx=(vx0+vx)*0.305;
  sy=(vy0+vy)*0.305;
  sz=(vz0+vz)*0.305;
  vx0=vx0+vx;
  vy0=vy0+vy;
  vz0=vz0+vz;
}


// the loop routine runs over and over again forever:
void loop() {
  // workeraound for serialEvent bug
  if(Serial.available() > 0)  serialEvent();
  // read the input pin:
  int buttonState = digitalRead(pushButton);

  mpu6050_getQuaternion(&qw, &qx, &qy, &qz);
  mpu6050_getRollPitchYaw(&roll, &pitch, &yaw);
  updateOffset();

  hat.gyro[0]=yaw*57.2957795;
  hat.gyro[1]=pitch*57.2957795;
  hat.gyro[2]=roll*57.2957795;
  hat.acc[0]=vx;
  hat.acc[1]=vy;
  hat.acc[2]=vz;

  cntr= cntr >= 999 ? 0 : cntr+1;
  hat.Cpt=cntr;
  if((sendData==1) && (dataAvailable==1)) {
    Serial.write((byte*)&hat,30);
    dataAvailable=0;
  }
  delay(1);
}

uint8_t buf[14];
void updateQuaternion_mahony2() {
	double axg = 0;
	double ayg = 0;
	double azg = 0;
	double gxrs = 0;
	double gyrs = 0;
	double gzrs = 0;
	double mxg = 0;
	double myg = 0;
	double mzg = 0;
        int16_t mx,my,mz;
        int16_t aax = 0, aay = 0, aaz = 0, agx = 0, agy = 0, agz = 0;
/*
	int16_t ax = 0;
	int16_t ay = 0;
	int16_t az = 0;
	int16_t gx = 0;
	int16_t gy = 0;
	int16_t gz = 0;
*/
	//get raw data
	while(1) {
		mpu6050_readBit(MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_DATA_RDY_BIT, (uint8_t *)buf);
		if(buf[0])
			break;
		_delay_us(10);
	}

	mpu6050_readBytes(MPU6050_RA_ACCEL_XOUT_H, 14, (uint8_t *)buf);
    aax = (((int16_t)buf[0]) << 8) | buf[1];
    aay = (((int16_t)buf[2]) << 8) | buf[3];
    aaz = (((int16_t)buf[4]) << 8) | buf[5];
    agx = (((int16_t)buf[8]) << 8) | buf[9];
    agy = (((int16_t)buf[10]) << 8) | buf[11];
    agz = (((int16_t)buf[12]) << 8) | buf[13];

	#if MPU6050_CALIBRATEDACCGYRO == 1
	axg = (double)(aax-MPU6050_AXOFFSET)/MPU6050_AXGAIN;
	ayg = (double)(aay-MPU6050_AYOFFSET)/MPU6050_AYGAIN;
	azg = (double)(aaz-MPU6050_AZOFFSET)/MPU6050_AZGAIN;
	gxrs = (double)(agx-MPU6050_GXOFFSET)/MPU6050_GXGAIN*0.01745329; //degree to radians
	gyrs = (double)(agy-MPU6050_GYOFFSET)/MPU6050_GYGAIN*0.01745329; //degree to radians
	gzrs = (double)(agz-MPU6050_GZOFFSET)/MPU6050_GZGAIN*0.01745329; //degree to radians
	#else
	axg = (double)(aax)/MPU6050_AGAIN;
	ayg = (double)(aay)/MPU6050_AGAIN;
	azg = (double)(aaz)/MPU6050_AGAIN;
	gxrs = (double)(agx)/MPU6050_GGAIN*0.01745329; //degree to radians
	gyrs = (double)(agy)/MPU6050_GGAIN*0.01745329; //degree to radians
	gzrs = (double)(agz)/MPU6050_GGAIN*0.01745329; //degree to radians
	#endif
//    Serial.print("ax=");Serial.print(ax);
//    Serial.print("ay=");Serial.print(ay);
//    Serial.print("az=");Serial.println(az);
	mag2.getHeading(&mx, &my, &mz);

    //compute data
    imu10dof01_mahonyUpdate(gxrs, gyrs, gzrs, axg, ayg, azg, mx, my, mz);
}


