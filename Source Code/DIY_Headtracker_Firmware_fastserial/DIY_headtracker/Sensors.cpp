// File: Sensors.cpp
// Desc: Implementations sensor board functionality.
//-----------------------------------------------------------------------------
//#include <FastSerial.h>
#include <SingleSerial.h>

#include "Arduino.h"
#include "config.h"
#include "functions.h"

//#define TWI_FREQ 250000L
//#include <Wire.h> don't use slow odd library
#include <I2C.h>

#include "sensors.h"

#include "FourtOrderFilter.h"
#include "AP_Math.h"
#include "AP_Common.h"

/*
Reference for basic sensor/IMU understanding/calculation:
http://www.starlino.com/imu_guide.html
http://www.sparkfun.com/datasheets/Sensors/Magneto/Tilt%20Compensated%20Compass.pdf
https://www.loveelectronics.co.uk/Tutorials/13/tilt-compensated-compass-arduino-tutorial
http://www.pololu.com/file/download/LSM303DLH-compass-app-note.pdf?file_id=0J434
*/


// Gyro
//
#define ITG3205_ADDR 0x68    // The address of ITG3205
#define ITG3205_X_ADDR 0x1D  // Start address for x-axis
#define ITG3205_SMPLRT_DIV 0x15 // sample rate
#define ITG3205_DLPF 0x16       // digital filter
#define ITG3205_IRQR 0x17       // interrupt configuration
#define ITG3205_STATUS 0x1A     // status register

#define SCALING_FACTOR 15.1     // Scaling factor - used when converting to angle

// Accelerometer
//
#define ADXL345_ADDR (0x53)  // The adress of ADXL345 
#define ADXL345_X_ADDR (0x32)// Start address for x-axis
#define ACC_SENS 256         // Sensitivity. 13 bit adc, +/- 16 g. Calculated as: (2^13)/(16*2)


// Magnetometer
//
#define HMC_ADDR 0x1E        // The address of HMC5883
#define HMC_X_ADDR (0x03)    // Start address for x-axis. 

#define TO_GRADUS  180/3.14159265
#define FROM_GRADUS  1/TO_GRADUS

#define ConfigRegA           0x00
#define ConfigRegB           0x01
#define MagGain              0x20
#define PositiveBiasConfig   0x11
#define NegativeBiasConfig   0x12
#define NormalOperation      0x10

#define ModeRegister         0x02
#define ContinuousConversion 0x00
#define SingleConversion     0x01

// ConfigRegA valid sample averaging for 5883L
#define SampleAveraging_1    0x00
#define SampleAveraging_2    0x01
#define SampleAveraging_4    0x02
#define SampleAveraging_8    0x03

// ConfigRegA valid data output rates for 5883L
#define DataOutputRate_0_75HZ 0x00
#define DataOutputRate_1_5HZ  0x01
#define DataOutputRate_3HZ    0x02
#define DataOutputRate_7_5HZ  0x03
#define DataOutputRate_15HZ   0x04
#define DataOutputRate_30HZ   0x05
#define DataOutputRate_75HZ   0x06

#define AP_COMPASS_TYPE_HMC5843         0x02
#define AP_COMPASS_TYPE_HMC5883L        0x03

extern void serial_write_S(char c);

byte compass_id;
byte compass_mode=0;

unsigned char ADXL345_ID = 0;
unsigned char ITG3205_ID = 0;
unsigned char HMC_ID = 0;

// Variables defined elsewhere
//
extern long channel_value[];

extern char outputMagAcc;

// Local variables
//
union {
    byte sensorBuffer[10];       // Buffer for bytes read from sensors
    int inp[5];
} sb;

char resetValues = 1;        // Used to reset headtracker/re-center. 

int   accRaw[3];             // Raw readings from accelerometer
float accG[3];               // G-force in each direction
float Radius;                     // Unit vector - total G.

int   gyroRaw[3];            // Raw readings from gyro
float gyroV[3];

int   magRaw[3];             // Raw readings from magnetometer
float magV[3];               // Normalized readings from magnetometer
float MR=1;                  // magnet field normalization, 1 before calibration

float magPosOff[3]; // калибровочные данные магнетометра
float magNegOff[3];
float magGain[3];
float magMGain[3]={0,0,0}; // калибровочная нормировка по осям 

// Final angles for headtracker:
float tiltAngle = 0;       // Tilt angle
float tiltAngleLP = 0;     // Tilt angle with low pass FilterSensorData

float rollAngle = 0;        // Roll angle
float rollAngleLP = 0;     // Roll angle with low pass FilterSensorData

float panAngle = 0;        // Pan angle
float panAngleLP = 0;      // Pan angle with low pass FilterSensorData

// Start values - center position for head tracker
float tiltStart = 0;
float panStart = 0;
float rollStart = 0;


Vector3f compass, accel, gyro;

byte TrackerStarted = 0;

extern bool do_output;

// Settings
//
settings sets;
/*
float tiltRollBeta = 10;
float panBeta = 10;
float gyroWeightTiltRoll = 1/50;
float gyroWeightPan = 1/50;

int servoPanCenter = 1500;
int servoTiltCenter = 1500;
int servoRollCenter = 1500;
int panMaxPulse = 2000;
int panMinPulse = 1000;
int tiltMaxPulse = 2000;
int tiltMinPulse = 1000;
int rollMaxPulse = 2000;
int rollMinPulse = 1000;
float panFactor = 17;
float tiltFactor = 17;
float rollFactor = 17;
unsigned char servoReverseMask = 0;

int   accOffset[3] = {0, 0, 0}; 
float magOffset[3] = {0, 0, 0};
float gyroOff[3] = {0, 0, 0};



unsigned char htChannels[3] = {8, 7, 6}; // pan, tilt, roll
*/

extern float dynFactor;


//
// End settings


// Function used to write to I2C:
static inline void WriteToI2C(byte device, byte address, byte val){
    i2c.write(device, address, val);
/*
    Wire.beginTransmission(device);
    Wire.write(address);
    Wire.write(val);
    Wire.endTransmission();
*/
}

// Function to read from I2C
static inline void ReadFromI2C(byte device, byte address, byte bytesToRead){
/*
    Wire.beginTransmission(device);
    Wire.write(address);
    Wire.endTransmission();
  
    Wire.beginTransmission(device);
    Wire.requestFrom(device, bytesToRead);
   
    uint8_t i = 0;   
    while ( Wire.available() ) {
        sb.sensorBuffer[i++] = Wire.read();
    }
    Wire.endTransmission();
*/
    i2c.read(device, address, bytesToRead, sb.sensorBuffer);

}




void readCompassRaw(void){    // Read x, y, z from magnetometer;
    ReadFromI2C(HMC_ADDR, HMC_X_ADDR, 6);

    magRaw[0] =     sb.sensorBuffer[1] | ((uint16_t)sb.sensorBuffer[0] << 8);

//    if (compass_id == AP_COMPASS_TYPE_HMC5883L) {
//        magRaw[1] = sb.sensorBuffer[3] | ((int)sb.sensorBuffer[2] << 8);
//        magRaw[2] = sb.sensorBuffer[5] | ((int)sb.sensorBuffer[4] << 8);
//    } else {
        magRaw[2] = sb.sensorBuffer[3] | ((uint16_t)sb.sensorBuffer[2] << 8);
        magRaw[1] = sb.sensorBuffer[5] | ((uint16_t)sb.sensorBuffer[4] << 8);
//    }


}


void readAccelRaw(){
    // Read x, y, z acceleration, pack the data.
    ReadFromI2C(ADXL345_ADDR, ADXL345_X_ADDR, 6);

    accRaw[0] = -sb.inp[0]; // ((int)sensorBuffer[0] | ((int)sensorBuffer[1] << 8)) * -1;
    accRaw[1] = -sb.inp[1]; // ((int)sensorBuffer[2] | ((int)sensorBuffer[3] << 8)) * -1;
    accRaw[2] =  sb.inp[2]; //  (int)sensorBuffer[4] | ((int)sensorBuffer[5] << 8);
}

void readGyroRaw(){
#if 0
    byte d;
    do {
        ReadFromI2C(ITG3205_ADDR, ITG3205_STATUS, 1);
        d = sb.sensorBuffer[0];
    } while(! (d & 1)); // wait fore data ready
#endif

    // Read x, y, z from gyro, pack the data
    ReadFromI2C(ITG3205_ADDR, ITG3205_X_ADDR, 6);

    gyroRaw[0] =  ((int)sb.sensorBuffer[1] | ((int)sb.sensorBuffer[0] << 8) ) * -1;		// BIG-ENDIAN :(
    gyroRaw[1] =  ((int)sb.sensorBuffer[3] | ((int)sb.sensorBuffer[2] << 8) ) * -1;
    gyroRaw[2] =  ((int)sb.sensorBuffer[5] | ((int)sb.sensorBuffer[4] << 8) ) *  1;

}

void GyroCalc() {

    readGyroRaw();
    gyroV[0] = gyroRaw[0]-sets.gyroOff[0];
    gyroV[1] = gyroRaw[1]-sets.gyroOff[1];
    gyroV[2] = gyroRaw[2]-sets.gyroOff[2];
}

//--------------------------------------------------------------------------------------
void UpdateSensors()
{
    readAccelRaw();
    readGyroRaw();
    readCompassRaw();
}

//--------------------------------------------------------------------------------------
// Func: SetGyroOffset
// Desc: Sets the gyro offset.
//--------------------------------------------------------------------------------------
void SetGyroOffset()
{
    sets.gyroOff[0] = sets.gyroOff[1] = sets.gyroOff[2] = 0;

#define GYRO_AVG 256

    for (int i = 0; i < GYRO_AVG; i++)  {
	delay(5);
        readGyroRaw();
        for (byte k = 0; k < 3; k++) {
            sets.gyroOff[k] += gyroRaw[k];
        }
    }
 
    sets.gyroOff[0] /= GYRO_AVG;
    sets.gyroOff[1] /= GYRO_AVG;
    sets.gyroOff[2] /= GYRO_AVG;
     
#if (DEBUG)     
    serial_printf_3(PSTR("#Gyro offset measured: %f %f %f\n"), sets.gyroOff[0], sets.gyroOff[1], sets.gyroOff[2]);
#endif 
}


Vector3f correctField(const Vector3f *f, const Vector3f *offsets, const Vector3f *diagonals, const Vector3f *offdiagonals){

    Matrix3f mat(
        diagonals->x,    offdiagonals->x, offdiagonals->y,
        offdiagonals->x,    diagonals->y, offdiagonals->z,
        offdiagonals->y, offdiagonals->z,    diagonals->z
    );

    return  mat * (*f + *offsets);    
}


//--------------------------------------------------------------------------------------
// Func: CalibrateMag
// Desc: 
//--------------------------------------------------------------------------------------
void CalibrateMag() { 
#define AVG_COUNT 32

    magPosOff[0] = magPosOff[1] = magPosOff[2] = magNegOff[0] = magNegOff[1] = magNegOff[2] = 0;

    WriteToI2C(HMC_ADDR,ModeRegister, SingleConversion);
    delay(10);
    WriteToI2C(HMC_ADDR, ConfigRegA, SampleAveraging_1<<5 | DataOutputRate_75HZ<<2 | PositiveBiasConfig);        // force positiveBias (compass should return 715 for all channels)
    delay(10);

    WriteToI2C(HMC_ADDR, ModeRegister, ContinuousConversion);
    
    for (byte i = 0; i < AVG_COUNT; i++)  {
        // read values from the compass
        delay(14);
        readCompassRaw();
	magPosOff[0] += magRaw[0];
	magPosOff[1] += magRaw[1];
	magPosOff[2] += magRaw[2];  
    }
  
  
    WriteToI2C(HMC_ADDR,ModeRegister, SingleConversion);
    delay(10);
    WriteToI2C(HMC_ADDR, ConfigRegA, SampleAveraging_1<<5 | DataOutputRate_75HZ<<2 | NegativeBiasConfig);
    delay(10);
    WriteToI2C(HMC_ADDR,ModeRegister,  ContinuousConversion);

    for (byte i = 0; i < AVG_COUNT; i++)  {
        // read values from the compass
        delay(14);
        readCompassRaw();
	magNegOff[0] += magRaw[0];
        magNegOff[1] += magRaw[1];
	magNegOff[2] += magRaw[2];
    }

    WriteToI2C(HMC_ADDR,ModeRegister, SingleConversion);
    delay(10);
    WriteToI2C(HMC_ADDR, ConfigRegA, compass_mode);
    delay(10);
    WriteToI2C(HMC_ADDR,ModeRegister, ContinuousConversion);

    magGain[0] = -2500.0 * AVG_COUNT / (magNegOff[0] - magPosOff[0]);
    magGain[1] = -2500.0 * AVG_COUNT / (magNegOff[1] - magPosOff[1]);
    magGain[2] = -2500.0 * AVG_COUNT / (magNegOff[2] - magPosOff[2]);


    float magTmp[3]={0,0,0};

    magMGain[0]=magMGain[1]=magMGain[2]=1; // without norma

// calibrate norm
    for (byte i = 0; i < AVG_COUNT; i++)  {
        // read values from the compass
        delay(14);
	getCompass();
	magTmp[0] += magV[0];
	magTmp[1] += magV[1];
	magTmp[2] += magV[2];  
    }

    MR = sqrt(magTmp[0]*magTmp[0] + magTmp[1]*magTmp[1] + magTmp[2]*magTmp[2]) / AVG_COUNT;

/*    magMGain[0] = magGain[0]/(MR * sets.magGain[0]); // калибровочная нормировка
    magMGain[1] = magGain[1]/(MR * sets.magGain[1]);
    magMGain[2] = magGain[2]/(MR * sets.magGain[2]);
*/
    magMGain[0] = magGain[0]/MR; // калибровочная нормировка to 1
    magMGain[1] = magGain[1]/MR;
    magMGain[2] = magGain[2]/MR;

#if (DEBUG)
//    Serial.printf_P(PSTR("Mag type: %i\n"), compass);
    serial_printf_3(PSTR("#Mag cal: %f:%f, %f:"), magNegOff[0] , magPosOff[0], magNegOff[1]);
    serial_printf_3(PSTR("%f, %f:%f, "),         magPosOff[1], magNegOff[2], magPosOff[2]);
    serial_printf_3(PSTR(" gain: %f %f %f\n"),   magGain[0], magGain[1], magGain[2]);
//    Serial.printf_P(PSTR("Mag offset: %f %f %f\n"), sets.magOffset[0], sets.magOffset[1], sets.magOffset[2]);
//    Serial.printf_P(PSTR("Mag gain:   %f %f %f\n"), sets.magGain[0], sets.magGain[1], sets.magGain[2]);

//    Serial.printf_P(PSTR("Mag tmp: %f %f %f\n"), magTmp[0], magTmp[1], magTmp[2]);
//    Serial.printf_P(PSTR("Mag norm: %f\n"), MR);
    serial_printf_3(PSTR("#Mag norm gain: %f %f %f\n"), magMGain[0], magMGain[1], magMGain[2]);
#endif          
}

void getCompass(){
    readCompassRaw();

    if (magRaw[0] == -4096)
	magV[0]=0;
    else
        magV[0] =  magRaw[0] * magMGain[0]; // / MR / sets.magGain[0];
    
    if (magRaw[1] == -4096)
	magV[1]=0;
    else
	magV[1] = -magRaw[1] * magMGain[1];


    if (magRaw[2] == -4096)
	magV[2]=0;
    else
	magV[2] = -magRaw[2] * magMGain[2];


// correct field by calibration data
    compass = correctField(new Vector3f(magV), new Vector3f(sets.magOffset), new Vector3f(sets.magGain), new Vector3f(sets.magDiagOff) );

}


// for calibration


//--------------------------------------------------------------------------------------


#define SCALE_FACTOR_X                                  7.604562738 // mg/LSB
#define SCALE_FACTOR_Y                                  7.604562738 // mg/LSB
#define SCALE_FACTOR_Z                                  7.8125      // mg/LSB
#define GRAVITATIONAL_ACCELERATION                      9.81 //meter/sec/sec

void AccelCalc(){	    // Calculate final angles:
    readAccelRaw();
    
/*    accG[0] = ((float)accRaw[0] + sets.accOffset[0] ) / ACC_SENS;
    accG[1] = ((float)accRaw[1] + sets.accOffset[1] ) / ACC_SENS;
    accG[2] = ((float)accRaw[2] + sets.accOffset[2] ) / ACC_SENS;
*/
/*    accG[0] = ((float)accRaw[0] + sets.accOffset[0] ) * SCALE_FACTOR_X * GRAVITATIONAL_ACCELERATION / 1000.0 / 2 / sets.accGain[0];
    accG[1] = ((float)accRaw[1] + sets.accOffset[1] ) * SCALE_FACTOR_Y * GRAVITATIONAL_ACCELERATION / 1000.0 / 2 / sets.accGain[1];
    accG[2] = ((float)accRaw[2] + sets.accOffset[2] ) * SCALE_FACTOR_Z * GRAVITATIONAL_ACCELERATION / 1000.0 / 2 / sets.accGain[2];
*/

    accG[0] = (float)accRaw[0] * SCALE_FACTOR_X * GRAVITATIONAL_ACCELERATION / 1000.0 / 2;
    accG[1] = (float)accRaw[1] * SCALE_FACTOR_Y * GRAVITATIONAL_ACCELERATION / 1000.0 / 2;
    accG[2] = (float)accRaw[2] * SCALE_FACTOR_Z * GRAVITATIONAL_ACCELERATION / 1000.0 / 2;


// correct data by calibration
    accel = correctField(new Vector3f(accG), new Vector3f(sets.accOffset), new Vector3f(sets.accGain), new Vector3f(sets.accDiagOff) );

}


void CalibrateAccel(){
#define ACCEL_AVG 64
    // So, lets calculate R
    // R^2 = Rx^2+Ry^2+Rz^2    
    float x=0,y=0,z=0;


// skip first measurements
/*    for(byte i=0;i<ACCEL_AVG;i++){
	delay(18);	// 50hz

	AccelCalc();
    }
*/

/*
//    uint8_t count=0;
    for(byte i=0;i<ACCEL_AVG;i++){
	delay(18);	// 50hz

	AccelCalc();

	x+=accel.x;
	y+=accel.y;
	z+=accel.z;	
    }

    Radius = sqrt((x*x) + (y*y) + (z*z)) / ACCEL_AVG;

#if DEBUG
    Serial.printf_P(PSTR("#Accel calibrated R=%f\n"), Radius);
#endif
*/
}


//--------------------------------------------------------------------------------------
// Func: MagCalc
// Desc: Calculates angle from magnetometer data.
//--------------------------------------------------------------------------------------
void CalcMagAngle(){
/* calculated early
    float tilt90 = tiltAngle - 90, roll90 = rollAngle - 90,
        sin_tilt = sin(tilt90 * FROM_GRADUS),
        cos_tilt = cos(tilt90 * FROM_GRADUS),
        sin_roll = sin(roll90 * FROM_GRADUS),
        cos_roll = cos(roll90 * FROM_GRADUS);
*/

}

void MagCalc(){
    
    getCompass();

    CalcMagAngle();
}


// reset normas
void CalibrationStart(){
    sets.magOffset[0]=sets.magOffset[1]=sets.magOffset[2]=0;
    sets.accOffset[0]=sets.accOffset[1]=sets.accOffset[2]=0;

    sets.magGain[0]=sets.magGain[1]=sets.magGain[2]=1;
    sets.accGain[0]=sets.accGain[1]=sets.accGain[2]=1;

    sets.magDiagOff[0]=sets.magDiagOff[1]=sets.magDiagOff[2]=0;
    sets.accDiagOff[0]=sets.accDiagOff[1]=sets.accDiagOff[2]=0;
    
//    magMGain[0]=magMGain[1]=magMGain[2]=1;
}

uint32_t beep_start_time=0;

//--------------------------------------------------------------------------------------
// Func: Filter
// Desc: Filters / merges sensor data. 
//--------------------------------------------------------------------------------------
void FilterSensorData()
{
    // Used to set initial values. 
    if (resetValues) {
#if FATSHARK_HT_MODULE
        digitalWrite(BUZZER, HIGH);
#endif
	digitalWrite(ARDUINO_LED, HIGH); 
        resetValues = 0; 

        tiltStart = tiltAngleLP;
        panStart  = panAngleLP;
        rollStart = rollAngleLP;

//        delay(100); cause loop slow-out
#if DEBUG
        serial_printf_3(PSTR("#Center reset! tilt=%f rol=%f pan=%f\n"), tiltStart, rollStart, panStart);
#endif
	digitalWrite(ARDUINO_LED, LOW); //ready
        
        beep_start_time=millis();
    }

#if FATSHARK_HT_MODULE
    if(beep_start_time && (millis()-beep_start_time)>100){
        beep_start_time=0;
        digitalWrite(BUZZER, LOW);
    }
#endif
    

    // Filter the high frequency noise from vibrations 
    float acc_x = computeFourthOrder(accel.x, &fourthOrder[0]);
    float acc_y = computeFourthOrder(accel.y, &fourthOrder[1]);
    float acc_z = computeFourthOrder(accel.z, &fourthOrder[2]);
    
    Radius = sqrt((acc_x*acc_x) + (acc_y*acc_y) + (acc_z*acc_z)); // to normalize

    float p;
    float angle_x, angle_y;
    float as=0;

    p=-acc_x / Radius;

    if(p <= -1)
        angle_x = -90;
    else if(p >= 1)
        angle_x =  90;
    else
        angle_x = (as=asin(p) ) * TO_GRADUS;

    float v=(acc_y / Radius) / cos(as);

    if(angle_x == -90 || angle_x == 90)
        angle_y=0;
    else if(v <= -1)
        angle_y = -90;
    else if(v >= 1)
        angle_y = 90;
    else
        angle_y = asin(v) * TO_GRADUS;

    float ta = (tiltAngle - 90) * FROM_GRADUS,
	  ra = (rollAngle - 90) * FROM_GRADUS;

    float cos_tilt=cos(ta);
    float sin_tilt=sin(ta);
    float cos_roll=cos(ra);
    float sin_roll=sin(ra);



// tilt/roll compensation
    float mx=0, my=0, mz=0;      // Calculated magnetometer values in all directions with pan/tilt compensation

    mx  = compass.x * cos_tilt + compass.z * sin_tilt;

    my =  compass.x * sin_roll * sin_tilt
        + compass.y * cos_roll
	- compass.z * sin_roll * cos_tilt;

    mz = -compass.x * cos_roll * sin_tilt
        + compass.y * sin_roll
	+ compass.z * cos_roll * cos_tilt;
  
    float angle_z;
  
    if(mx==0 && my==0)
	angle_z=90;
    else {
	// Calculate pan-angle from magnetometer. 
	angle_z = atan2(my, mx) * TO_GRADUS + 90;
    }
    
    angle_z -= panStart;
  
    if (angle_z > 180) {
        angle_z -= 360;
    } else if (angle_z < -180) {
        angle_z += 360; 
    }


    float ga[3];

    // correct gyros
    ga[0] =  gyroV[0] *  cos_tilt + gyroV[2] *  sin_tilt;
    ga[1] =  gyroV[1] *  cos_roll + gyroV[2] * -sin_roll;
    ga[2] =  gyroV[2] *  cos_tilt + gyroV[0] * -sin_tilt + gyroV[1] * sin_roll;


    ga[0] /= (SAMPLERATE * SCALING_FACTOR);
    ga[1] /= (SAMPLERATE * SCALING_FACTOR);
    ga[2] /= (SAMPLERATE * SCALING_FACTOR);


    // uses mainly gyro-data, but uses accelerometer to compensate for drift
/*
    rollAngle = (rollAngle + ga[0]) * sets.gyroWeightTiltRoll  +  angle_y * (1 - sets.gyroWeightTiltRoll);
    tiltAngle = (tiltAngle + ga[1]) * sets.gyroWeightTiltRoll  +  angle_x * (1 - sets.gyroWeightTiltRoll);
    panAngle  = (panAngle  + ga[2]) * sets.gyroWeightPan       +  angle_z * (1 - sets.gyroWeightPan);
*/
    filterAB(rollAngle, ga[0], angle_y, sets.gyroWeightTiltRoll);
    filterAB(tiltAngle, ga[1], angle_x, sets.gyroWeightTiltRoll);
    filterAB(panAngle,  ga[2], angle_z, sets.gyroWeightPan);

/*
    x = (x+a)*k + b*(1-k)
    x = x*k + a*k + b*(1-k)
    x = x + x*(k-1) + a*k + b*(1-k)
    x += x*(k-1) + a*k + b*(1-k)
    x += a*k + b*(1-k) - x*(1-k)
    x += a*k + (b-x)*(1-k)
*/

#ifdef DEBUG
    if(do_output){
        if(outputDbg == 1){ // accel
            serial_printi_3(PSTR("\nraw: %d,%d,%d\n"),  accRaw[0], accRaw[1], accRaw[2]);
            serial_printf_3(PSTR(  "G:   %f,%f,%f R="), accG[0], accG[1], accG[2]);
            serial_print(Radius);

            serial_printf_3(PSTR("\nOK:    %f,%f,%f\n"), accel.x, accel.y, accel.z);
            serial_printf_3(PSTR(  "filter %f,%f,%f\n"), acc_x, acc_y, acc_z);
            serial_printf_3(PSTR(  "ang:   %f,%f,%f\n"), angle_x, angle_y, angle_z);

        }
        if(outputDbg == 2){ // gyro
            serial_printi_3(PSTR("\nraw: %d,%d,%d\n"), gyroRaw[0], gyroRaw[1], gyroRaw[2]);
            serial_printi_3(PSTR(  "OK:  %d,%d,%d\n"), gyroV[0], gyroV[1], gyroV[2]);
            serial_printf_3(PSTR(  "ang: %f,%f,%f\n"), ga[0], ga[1], ga[2]);
        }
        if(outputDbg == 3){ //compass
            serial_printi_3(PSTR("\nraw:  %d,%d,%d\n"), magRaw[0], magRaw[1], magRaw[2]);
            serial_printf_3(PSTR(  "norm: %f,%f,%f\n"), magV[0],   magV[1],   magV[2]);
            serial_printf_3(PSTR(  "ok:   %f,%f,%f\n"), compass.x, compass.y, compass.z);
            serial_printf_3(PSTR(  "m*:   %f,%f,%f\n"), mx, my, mz);    
            Serial.printf_P(PSTR(  "s/c:  %f,%f,%f,%f\n"), sin_roll, cos_roll, sin_tilt, cos_tilt );    
            serial_printf_1(PSTR(  "ang z:%f\n"), angle_z);
        }
        
        if(outputDbg == 4){ //full state
            Serial.printf_P(PSTR("\nroll: %f,%f,%f (0=%f)\n"), ga[0], angle_y, rollAngle, rollStart);
            Serial.printf_P(PSTR(  "tilt: %f,%f,%f (0=%f)\n"), ga[1], angle_x, tiltAngle, tiltStart);
            Serial.printf_P(PSTR(  "pan:  %f,%f,%f (0=%f)\n"), ga[2], angle_z, panAngle,  panStart);
        }
 #if PPM_IN
        if(outputDbg == 5)          testPPM_in();
 #endif
        
    }

    do_output=false;

#endif

    // All low-pass filters
    filter(tiltAngleLP, tiltAngle, sets.tiltRollBeta);
    filter(rollAngleLP, rollAngle, sets.tiltRollBeta);
    filter(panAngleLP,  panAngle,  sets.panBeta);

    if (TrackerStarted)  {

	{
            float angleTemp = (panAngleLP - panStart)  * sets.panFactor * dynFactor + sets.servoPanCenter;
            if ( (angleTemp > sets.panMinPulse) && (angleTemp < sets.panMaxPulse) )  {
                channel_value[sets.htChannels[0]] =  angleTemp;
            }    
	}
	{
            float angleTemp = (tiltAngleLP - tiltStart) *  sets.tiltFactor * dynFactor + sets.servoTiltCenter;
            if ( (angleTemp > sets.tiltMinPulse) && (angleTemp < sets.tiltMaxPulse) ) {
                channel_value[sets.htChannels[1]] =  angleTemp;
            }
        }

	{
            float angleTemp = (rollAngleLP - rollStart) * sets.rollFactor * dynFactor + sets.servoRollCenter;
            if ( (angleTemp > sets.rollMinPulse) && (angleTemp < sets.rollMaxPulse) ) {
                channel_value[sets.htChannels[2]] =  angleTemp;
            }
        }
    }
}

//--------------------------------------------------------------------------------------
// Func: InitSensors
// Desc: Initializes the sensor board sensors.
//--------------------------------------------------------------------------------------
void InitSensors()
{
    i2c.begin();               // Start I2C
    i2c.setSpeed(250000L);
    i2c.timeOut(50); // ms
    
    ReadFromI2C(ITG3205_ADDR, 0x00, 1);
    ITG3205_ID = sb.sensorBuffer[0];
 
#if DEBUG
    serial_printi_1(PSTR("#ITG3205: %d\n"), ITG3205_ID);
#endif 
 
    ReadFromI2C(ADXL345_ADDR, 0x00, 1);
    ADXL345_ID = sb.sensorBuffer[0];
 
#if DEBUG
    serial_printi_1(PSTR("#    ADXL: %d\n"), ADXL345_ID); 
#endif  

    // Accelerometer increase G-range (+/- 16G)
    WriteToI2C(ADXL345_ADDR, 0x31, 0b00001011);
    ReadFromI2C(HMC_ADDR, 0x00, 1);
    HMC_ID = sb.sensorBuffer[0];
 
#if DEBUG
    serial_printi_1(PSTR("#    HMC: %d\n"), HMC_ID); 
#endif  

//    WriteToI2C(ITG3205_ADDR, 22, 0x18 | 2); //Register 22 – DLPF, Full Scale - full range 98hz filter
    WriteToI2C(ITG3205_ADDR, 22, 0x18 | 4); //Register 22 – DLPF, Full Scale - full range 20hz filter
    WriteToI2C(ITG3205_ADDR, ITG3205_IRQR, 1<<5 || // LATCH_INT_EN
                                           1<<4 || // INT_ANYRD_2CLEAR
                                           1 );    // RAW_RDY_EN 


    //  ADXL345 POWER_CTL
    WriteToI2C(ADXL345_ADDR, 0x2D, 0); 
    WriteToI2C(ADXL345_ADDR, 0x2D, 16);
    WriteToI2C(ADXL345_ADDR, 0x2D, 8);

    // HMC5883
    
    uint8_t calibration_gain = 0b00100000 ;//0x20;  - 820
//    uint16_t expected_x = 715;
//    uint16_t expected_yz = 715;
//    float gain_multiple = 1.0;

//    WriteToI2C(HMC_ADDR, 0x00, 0b01111000); //crA - 8avg 75hz normal
    compass_mode = SampleAveraging_8<<5 | DataOutputRate_75HZ<<2 | NormalOperation;
    WriteToI2C(HMC_ADDR, ConfigRegA, compass_mode);
    ReadFromI2C(HMC_ADDR, ConfigRegA, 1);
    byte d = sb.sensorBuffer[0];
    
    if(d == compass_mode) {        // a 5883L supports the sample averaging config

#if DEBUG
//    Serial.print_P(PSTR("compass type HMC5883L\n")); 
#endif  

        compass_id = AP_COMPASS_TYPE_HMC5883L;
//        calibration_gain = 0b01100000; //0x60; - 660
//        calibration_gain = 0b11100000; //  230
        calibration_gain = 0b10000000; //  440
        /*
          note that the HMC5883 datasheet gives the x and y expected
          values as 766 and the z as 713. Experiments have shown the x
          axis is around 766, and the y and z closer to 713.
         */
//        expected_x = 766;
//        expected_yz  = 713;
//        gain_multiple = 660.0 / 1090;  // adjustment for runtime vs calibration gain
        
    } else if ( d == (NormalOperation | DataOutputRate_75HZ<<2)) {
#if DEBUG
//    Serial.print_P(PSTR("compass type HMC5883\n")); 
#endif  
	compass_id = AP_COMPASS_TYPE_HMC5843;
    } else {

	Serial.printf_P(PSTR("Error detecting compass! %x - %x\n"), d, compass_mode);
	compass = 0;
	digitalWrite(ARDUINO_LED, HIGH);
    }

    WriteToI2C(HMC_ADDR, ConfigRegB, calibration_gain); //crB - gain 660
    // Run in continuous mode
    WriteToI2C(HMC_ADDR, ModeRegister, ContinuousConversion); //mode = 0

#if (ALWAYS_CAL_GYRO)
    // Set sensor offset
    SetGyroOffset();
#endif 
}

//--------------------------------------------------------------------------------------
// Func: ResetCenter
// Desc: Utility for resetting tracker center. This is only called during tracker
//       startup. Button press resets are handled during filtering. (Needs refactor)
//--------------------------------------------------------------------------------------

#if 0
void ResetCenter()
{
    resetValues = 0; 


    // Not sure what Dennis is doing here. Giving it
    // time to stabilize, since this is called at setup time?
    for (byte k = 0; k < 16; k++) {
//        UpdateSensors();    
        GyroCalc();
        AccelCalc();
        MagCalc();
        FilterSensorData();
    }
    
    tiltStart = accAngle[0];
    rollStart = accAngle[1];  
    panStart  = magAngle[2];
  
    GyroCalc();
    AccelCalc();
    MagCalc();
    FilterSensorData();    
  
    tiltAngle = accAngle[0];
    rollAngle = accAngle[1];
    panAngle  = magAngle[2];

#if DEBUG
    serial_printf_3(PSTR("Center reset v2! tilt=%f rol=%f pan=%f\n"), tiltStart, rollStart, panStart);
#endif
    
    TrackerStarted = 1;
}
#endif


//--------------------------------------------------------------------------------------
// Func: SensorInfoPrint
// Desc: Prints the mag sensor data.
//--------------------------------------------------------------------------------------
void SensorInfoPrint()
{ 
    serial_printf_3a(PSTR("Mag cal: %f,%f,%f,"), magNegOff[0] - magPosOff[0], magNegOff[1] - magPosOff[1], magNegOff[2] - magPosOff[2]);
   
    serial_printf_3(PSTR("%f,%f,%f\n"),                     magGain[0], magGain[1], magGain[2]);   

    serial_printi_3(PSTR("ADXL345 ID: %d\nITG3205 ID: %d\nHMC ID: %d\n"), (int)ADXL345_ID, (int)ITG3205_ID,  (int)HMC_ID);
}


// ===============================================
//      ---------- Test functions -----------
// ===============================================

#if DEBUG

// output calculated values, output as "csv"

// All sensor output as "csv". 
void testAllSensors() //  not used
{  
    serial_print(accRaw[0]);  
    serial_write_S(',');          
    serial_print(accRaw[1]);  
    serial_write_S(',');              
    serial_print(accRaw[2]);  

    serial_write_S(',');            
    serial_print(gyroRaw[0]);  
    serial_write_S(',');          
    serial_print(gyroRaw[1]);  
    serial_write_S(',');              
    serial_print(gyroRaw[2]);  

    serial_write_S(',');               
    serial_print(magRaw[0]);  
    serial_write_S(',');          
    serial_print(magRaw[1]);  
    serial_write_S(',');              
    serial_print(magRaw[2]);      
    serial_println();
}
#endif

void clearSettings() {
    sets.vers = EEPROM_VERSION;

    sets.tiltRollBeta = 10;
    sets.panBeta = 10;

    sets.gyroWeightTiltRoll = 1/50.;
    sets.gyroWeightPan = 1/50.;

    sets.servoPanCenter = 1500;
    sets.servoTiltCenter = 1500;
    sets.servoRollCenter = 1500;
    sets.panMaxPulse = 2000;
    sets.panMinPulse = 1000;
    sets.tiltMaxPulse = 2000;
    sets.tiltMinPulse = 1000;
    sets.rollMaxPulse = 2000;
    sets.rollMinPulse = 1000;
    sets.panFactor = 17;
    sets.tiltFactor = 17;
    sets.rollFactor = 17;
    sets.servoReverseMask = 0;

    sets.accOffset[0] = sets.accOffset[1] = sets.accOffset[2] = 0;
    sets.magOffset[0] = sets.magOffset[1] = sets.magOffset[2] = 0;
    sets.gyroOff[0]   = sets.gyroOff[1]   = sets.gyroOff[2] = 0;

    sets.htChannels[0] = 8; // pan
    sets.htChannels[1] = 7; // tilt
    sets.htChannels[2] = 6; // roll

    sets.magGain[0]=sets.magGain[1]=sets.magGain[2]=1;
    sets.accGain[0]=sets.accGain[1]=sets.accGain[2]=1;


    sets.magDiagOff[0]=sets.magDiagOff[1]=sets.magDiagOff[2]=0;
    sets.accDiagOff[0]=sets.accDiagOff[1]=sets.accDiagOff[2]=0;

    byte i;
    for(i=0; i<13; i++)
	sets.PpmIn_PpmOut[i] = i;

}


void trackerOutput()
{
/*
  serial_print(tiltAngleLP - tiltStart );
  serial_write_S(',');
  serial_print(rollAngleLP - rollStart );
  serial_write_S(',');  
  serial_print(panAngleLP);
  serial_println();
*/
  
  serial_printf_3a(PSTR("%f,%f,%f\n"),  tiltAngleLP - tiltStart, rollAngleLP - rollStart, panAngleLP );
}

void calMagAccOutput()
{
//    Serial.printf_P(PSTR("%d,%d,%d,%d,%d,%d\n"),  magRaw[0], magRaw[1], magRaw[2], accRaw[0], accRaw[1], accRaw[2]);

// not raw! but normalized 
    serial_printf_3(PSTR("%f,%f,%f,"),   magV[0], magV[1], magV[2]);
    serial_printf_3(PSTR("%f,%f,%f\n"),  accG[0], accG[1], accG[2]);
}


