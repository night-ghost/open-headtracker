// File: Sensors.cpp
// Desc: Implementations sensor board functionality.
//-----------------------------------------------------------------------------
#include <FastSerial.h>

#include "config.h"
#include "Arduino.h"
#include "functions.h"
#include <Wire.h>
#include "MadgwickAHRS.h"

//#include "MadgwickAHRS.c"

/*
Reference for basic sensor/IMU understanding/calculation:
http://www.starlino.com/imu_guide.html
http://www.sparkfun.com/datasheets/Sensors/Magneto/Tilt%20Compensated%20Compass.pdf
https://www.loveelectronics.co.uk/Tutorials/13/tilt-compensated-compass-arduino-tutorial
http://www.pololu.com/file/download/LSM303DLH-compass-app-note.pdf?file_id=0J434


AHRS - https://code.google.com/p/imumargalgorithm30042010sohm/downloads/list
*/


// Gyro
//
#define ITG3205_ADDR 0x68    // The address of ITG3205
#define ITG3205_X_ADDR 0x1D  // Start address for x-axis
#define SCALING_FACTOR 13     // Scaling factor - used when converting to angle
#define ITG3205_SCALE_FACTOR 0.0695652174 // deg/sec/LSB

// Accelerometer
//
#define ADXL345_ADDR (0x53)  // The adress of ADXL345 
#define ADXL345_X_ADDR (0x32)// Start address for x-axis
//#define ACC_SENS 256         // Sensitivity. 13 bit adc, +/- 16 g. Calculated as: (2^13)/(16*2)


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

byte compass;
byte compass_mode=0;

unsigned char ADXL345_ID = 0;
unsigned char ITG3205_ID = 0;
unsigned char HMC_ID = 0;

// Variables defined elsewhere
//
extern long channel_value[];

extern volatile char was_int;

// Local variables
//
union {
    byte sensorBuffer[10];       // Buffer for bytes read from sensors
    int inp[5];
} sb;

char resetValues = 1;        // Used to reset headtracker/re-center. 

int   accRaw[3];             // Raw readings from accelerometer
float accG[3];               // G-force in each direction
float accAngle[3];           // Measured angle from accelerometer
float R;                     // Unit vector - total G.

int   gyroRaw[3];            // Raw readings from gyro
float angle[3];              // Angle from gyro 
float angleRaw[3];           // Temp for angle-calculation
float gyro[3];

int   magRaw[3];             // Raw readings from magnetometer
int   magV[3];               // Normalized readings from magnetometer
float magAngle[3];           // Measured angles from magnetometer
float mx = 0;                // Calculated magnetometer value in x-direction with pan/tilt compensation
float my = 0;                // Calculated magnetometer value in y-direction with pan/tilt compensation

float magPosOff[3]; // калибровочные данные магнетометра
float magNegOff[3];
float magGain[3];

// Final angles for headtracker:
float tiltAngle = 90;       // Tilt angle
float tiltAngleLP = 90;     // Tilt angle with low pass FilterSensorData
float lastTiltAngle = 90;   // Used in low pass FilterSensorData.

float rollAngle = 0;        // Roll angle
float rollAngleLP = 90;     // Roll angle with low pass FilterSensorData
float lastRollAngle = 90;   // Used in low pass FilterSensorData

float panAngle = 90;        // Pan angle
float panAngleLP = 90;      // Pan angle with low pass FilterSensorData
float lastPanAngle = 90;    // Used in low pass FilterSensorData

// Start values - center position for head tracker
float tiltStart = 0;
float panStart = 0;
float rollStart = 0;

char TrackerStarted = 0;

// Servo reversing
char tiltInverse = -1;
char rollInverse = -1;
char panInverse = -1;

// Settings
//
float tiltRollBeta = 0.75;
float panBeta = 0.75;
float gyroWeightTiltRoll = 0.98;
float GyroWeightPan = 0.98;

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

float AEq_1 = 1, AEq_2 = 0, AEq_3 = 0, AEq_4 = 0;  // quaternion orientation of earth frame relative to auxiliary frame
float ESq_1, ESq_2, ESq_3, ESq_4;                  // quaternion describing orientation of sensor relative to earth
float ASq_1, ASq_2, ASq_3, ASq_4;                  // quaternion describing orientation of sensor relative to auxiliary frame


//
// End settings


// Function used to write to I2C:
void WriteToI2C(int device, byte address, byte val){
    Wire.beginTransmission(device);
    Wire.write(address);
    Wire.write(val);
    Wire.endTransmission();
}

// Function to read from I2C
void ReadFromI2C(int device, char address, char bytesToRead){
    Wire.beginTransmission(device);
    Wire.write(address);
    Wire.endTransmission();
  
    Wire.beginTransmission(device);
    Wire.requestFrom(device, bytesToRead);
   
    char i = 0;   
    while ( Wire.available() ) {
        sb.sensorBuffer[i++] = Wire.read();
    }
    Wire.endTransmission();
}


//--------------------------------------------------------------------------------------
// Func: InitSensors
// Desc: Initializes the sensor board sensors.
//--------------------------------------------------------------------------------------
void InitSensors()
{
    ReadFromI2C(ITG3205_ADDR, 0x00, 1);
    ITG3205_ID = sb.sensorBuffer[0];
 
#if DEBUG
//    Serial.printf_P(PSTR("ITG3205: %d\n"), ITG3205_ID);
#endif 
 
    ReadFromI2C(ADXL345_ADDR, 0x00, 1);
    ADXL345_ID = sb.sensorBuffer[0];
 
#if DEBUG
//    Serial.printf_P(PSTR("    ADXL: %d\n"), ADXL345_ID); 
#endif  

    // Accelerometer increase G-range (+/- 4G)
    WriteToI2C(ADXL345_ADDR, 0x31, 0x01);
// 100 Hz measurements
    WriteToI2C(ADXL345_ADDR, 0x2C, 0x0A);

    ReadFromI2C(HMC_ADDR, 0x00, 1);
    HMC_ID = sb.sensorBuffer[0];
 
#if DEBUG
//    Serial.printf_P(PSTR("    HMC: %d\n"), HMC_ID); 
#endif  

//    WriteToI2C(ITG3205_ADDR, 22, 0x18 | 2); //Register 22 – DLPF, Full Scale - full range 98hz filter
    WriteToI2C(ITG3205_ADDR, 22, 0x1D); //Register 22 – DLPF, Full Scale - full range 10hz filter

    //  ADXL345 POWER_CTL
    WriteToI2C(ADXL345_ADDR, 0x2D, 0); 
    WriteToI2C(ADXL345_ADDR, 0x2D, 16);
    WriteToI2C(ADXL345_ADDR, 0x2D, 8); //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register

    // HMC5883
    
    uint8_t calibration_gain = 0x20;
//    uint16_t expected_x = 715;
//    uint16_t expected_yz = 715;
//    float gain_multiple = 1.0;

//    WriteToI2C(HMC_ADDR, 0x00, 0b01111000); //crA - 8avg 75hz normal
    compass_mode = SampleAveraging_8<<5 | DataOutputRate_75HZ<<2 | NormalOperation;
    WriteToI2C(HMC_ADDR, ConfigRegA, compass_mode);
    ReadFromI2C(HMC_ADDR, ConfigRegA, 1);
    byte d = sb.sensorBuffer[0];
    
    if(d == compass_mode) {        // a 5883L supports the sample averaging config
        compass = AP_COMPASS_TYPE_HMC5883L;
        calibration_gain = 0x60;
        /*
          note that the HMC5883 datasheet gives the x and y expected
          values as 766 and the z as 713. Experiments have shown the x
          axis is around 766, and the y and z closer to 713.
         */
//        expected_x = 766;
//        expected_yz  = 713;
//        gain_multiple = 660.0 / 1090;  // adjustment for runtime vs calibration gain
        
    } else if ( d == (NormalOperation | DataOutputRate_75HZ<<2)) {
	compass = AP_COMPASS_TYPE_HMC5843;
    } else {
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



void calMagOutput()
{
    Serial.print((int)magRaw[0]);
    Serial.write(',');
    Serial.print((int)magRaw[1] );
    Serial.write(',');
    Serial.println((int)magRaw[2] );
}

void calAccOutput()
{
    Serial.print((int)accRaw[0] );
    Serial.write(',');
    Serial.print((int)accRaw[1] );
    Serial.write(',');
    Serial.println((int)accRaw[2] );
}

void calMagAccOutput()
{
    Serial.print((int)magRaw[0]);
    Serial.write(',');
    Serial.print((int)magRaw[1]);
    Serial.write(',');
    Serial.print((int)magRaw[2]);
    Serial.write(',');
    Serial.print((int)accRaw[0]);
    Serial.write(',');
    Serial.print((int)accRaw[1]);
    Serial.write(',');
    Serial.println((int)accRaw[2]);
}


void readCompassRaw(void){    // Read x, y, z from magnetometer;
    ReadFromI2C(HMC_ADDR, HMC_X_ADDR, 6);

    magRaw[0] =     sb.sensorBuffer[1] | ((int)sb.sensorBuffer[0] << 8);

    if (compass == AP_COMPASS_TYPE_HMC5883L) {
        magRaw[1] = sb.sensorBuffer[3] | ((int)sb.sensorBuffer[2] << 8);
        magRaw[2] = sb.sensorBuffer[5] | ((int)sb.sensorBuffer[4] << 8);
    } else {
        magRaw[2] = sb.sensorBuffer[3] | ((int)sb.sensorBuffer[2] << 8);
        magRaw[1] = sb.sensorBuffer[5] | ((int)sb.sensorBuffer[4] << 8);
    }


    if (magRaw[0] == -4096 || magRaw[1] == -4096 || magRaw[2] == -4096) {
        // no valid data available
        //return false;
        magRaw[0] = magRaw[1] = magRaw[2] = 0; 
        
    }
}


void readAccelRaw(){
    // Read x, y, z acceleration, pack the data.
    ReadFromI2C(ADXL345_ADDR, ADXL345_X_ADDR, 6);

    accRaw[0] =  sb.inp[0]; // ((int)sensorBuffer[0] | ((int)sensorBuffer[1] << 8)) * -1;
    accRaw[1] =  sb.inp[1]; // ((int)sensorBuffer[2] | ((int)sensorBuffer[3] << 8)) * -1;
    accRaw[2] = -sb.inp[2]; //  (int)sensorBuffer[4] | ((int)sensorBuffer[5] << 8);
}

void readGyroRaw(){
    // Read x, y, z from gyro, pack the data
    ReadFromI2C(ITG3205_ADDR, ITG3205_X_ADDR, 6);

    gyroRaw[0] =  ((int)sb.sensorBuffer[1] | ((int)sb.sensorBuffer[0] << 8) ) * -1;		// BIG-ENDIAN :(
    gyroRaw[1] =  ((int)sb.sensorBuffer[3] | ((int)sb.sensorBuffer[2] << 8) ) * -1;
    gyroRaw[2] =  ((int)sb.sensorBuffer[5] | ((int)sb.sensorBuffer[4] << 8) ) *  1;

}

//--------------------------------------------------------------------------------------
void UpdateSensors()
{
    readAccelRaw();
    readGyroRaw();
    readCompassRaw();
}

void delay_int(){
    was_int=0;
    
    while(!was_int){};
}


//--------------------------------------------------------------------------------------
// Func: SetGyroOffset
// Desc: Sets the gyro offset.
//--------------------------------------------------------------------------------------
void SetGyroOffset()
{
    gyroOff[0] = gyroOff[1] = gyroOff[2] = 0;

#define GYRO_AVG 100

    for (byte i = 0; i < GYRO_AVG; i++)  {
	delay(5);
	//delay_int();
        readGyroRaw();
        for (byte k = 0; k < 3; k++) {
            gyroOff[k] += gyroRaw[k];
        }
    }
 
    for (byte k = 0; k < 3; k++) {
        gyroOff[k] = gyroOff[k] / GYRO_AVG;
    }
     
#if (DEBUG)     
//    Serial.printf_P(PSTR("Gyro offset measured: %f %f %f\n"), gyroOff[0], gyroOff[1], gyroOff[2]);
#endif 
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


    magPosOff[0] /= AVG_COUNT;
    magPosOff[1] /= AVG_COUNT;
    magPosOff[2] /= AVG_COUNT;

    magNegOff[0] /= AVG_COUNT;
    magNegOff[1] /= AVG_COUNT;
    magNegOff[2] /= AVG_COUNT;

    magGain[0] = -2500 / (magNegOff[0] - magPosOff[0]);
    magGain[1] = -2500 / (magNegOff[1] - magPosOff[1]);
    magGain[2] = -2500 / (magNegOff[2] - magPosOff[2]);


#if (DEBUG)
//    Serial.printf_P(PSTR("Mag type: %i\n"), compass);
//    Serial.printf_P(PSTR("Mag cal: %f:%f, %f:%f, %f:%f, gain: %f %f %f\n"), magNegOff[0] , magPosOff[0], magNegOff[1], magPosOff[1], magNegOff[2], magPosOff[2], magGain[0], magGain[1], magGain[2]);
//    Serial.printf_P(PSTR("Mag offset: %f %f %f\n"), magOffset[0], magOffset[1], magOffset[2]);
#endif          
}


//--------------------------------------------------------------------------------------
// Func: GyroCalc
// Desc: Calculate angle from gyro-data
//--------------------------------------------------------------------------------------
void GyroCalc() {
    readGyroRaw();

    gyro[0] = (gyroRaw[0]-gyroOff[0]) * ITG3205_SCALE_FACTOR;
    gyro[1] = (gyroRaw[1]-gyroOff[1]) * ITG3205_SCALE_FACTOR;
    gyro[2] = (gyroRaw[2]-gyroOff[2]) * ITG3205_SCALE_FACTOR;

    
    for (byte i=0; i<3; i++) {
        angleRaw[i] += gyroRaw[i]-gyroOff[i];
        angle[i]     = angleRaw[i] / SAMPLERATE;
    }
}


//--------------------------------------------------------------------------------------
// Func: AccelCalc
// Desc: Calculate angle from accelerometer data
//--------------------------------------------------------------------------------------

#define SCALE_FACTOR_X                                  7.604562738 // mg/LSB
#define SCALE_FACTOR_Y                                  7.604562738 // mg/LSB
#define SCALE_FACTOR_Z                                  7.8125      // mg/LSB
#define GRAVITATIONAL_ACCELERATION                      9.81 //meter/sec/sec

void getAccel(){
    readAccelRaw();
    
    accG[0] = ((float)accRaw[0] + accOffset[0] )* SCALE_FACTOR_X * GRAVITATIONAL_ACCELERATION / 1000.0;
    accG[1] = ((float)accRaw[1] + accOffset[1] )* SCALE_FACTOR_Y * GRAVITATIONAL_ACCELERATION / 1000.0;
    accG[2] = ((float)accRaw[2] + accOffset[2] )* SCALE_FACTOR_Z * GRAVITATIONAL_ACCELERATION / 1000.0;
}

void CalibrateAccel(){
#define ACCEL_AVG 32
    // So, lets calculate R
    // R^2 = Rx^2+Ry^2+Rz^2    
    float x=0,y=0,z=0;
    
    for(byte i=0;i<ACCEL_AVG;i++){
	delay(18);	// 50hz

	getAccel();

	x+=accG[0];
	y+=accG[1];
	z+=accG[2];
    }

    x /= ACCEL_AVG;
    y /= ACCEL_AVG;
    z /= ACCEL_AVG;

    R = sqrt((x*x) + (y*y) + (z*z));

}

void AccelCalc(){	    // Calculate final angles:
    getAccel();

/*    
    for (byte i = 0; i<3; i++) {
	float v=accG[i] / R;

	if(v < -1)
	    accAngle[i] = 180;
	else if(v > 1)
	    accAngle[i] = 0;
	else
	    accAngle[i] = acos(accG[i] / R) * TO_GRADUS;
    }
*/
}

void getCompass(){
    readCompassRaw();

    // Invert 2 axis  
    magRaw[1] *= -1;
    magRaw[2] *= -1;
    
    // Set gain:
    magV[0] = magRaw[0] * magGain[0] - magOffset[0];
    magV[1] = magRaw[1] * magGain[1] - magOffset[1];
    magV[2] = magRaw[2] * magGain[2] - magOffset[2];

}

//--------------------------------------------------------------------------------------
// Func: MagCalc
// Desc: Calculates angle from magnetometer data.
//--------------------------------------------------------------------------------------
void MagCalc(){
    
    getCompass();
    
}

//--------------------------------------------------------------------------------------
// Func: Filter
// Desc: Filters / merges sensor data. 
//--------------------------------------------------------------------------------------

float xtilt;

void FilterSensorData()
{
    int temp = 0;


	MadgwickAHRSupdate(1.0 / SAMPLERATE,  gyro[0] * TO_GRADUS, gyro[1] * TO_GRADUS, gyro[2] * TO_GRADUS, accG[0] / R, -accG[1], -accG[2], magV[0] *0, -magV[1]*0, magV[2]*0);
 
            
       // compute the quaternion conjugate
        ESq_1 = q0;
        ESq_2 = -q1;
        ESq_3 = -q2;
        ESq_4 = -q3;

        // compute the quaternion product
        ASq_1 = ESq_1 * AEq_1 - ESq_2 * AEq_2 - ESq_3 * AEq_3 - ESq_4 * AEq_4;
        ASq_2 = ESq_1 * AEq_2 + ESq_2 * AEq_1 + ESq_3 * AEq_4 - ESq_4 * AEq_3;
        ASq_3 = ESq_1 * AEq_3 - ESq_2 * AEq_4 + ESq_3 * AEq_1 + ESq_4 * AEq_2;
        ASq_4 = ESq_1 * AEq_4 + ESq_2 * AEq_3 - ESq_3 * AEq_2 + ESq_4 * AEq_1;

        // compute the Euler angles from the quaternion
        xtilt =           2 * ASq_2 * ASq_3 - 2 * ASq_1 * ASq_3;
        if(     xtilt >  1) tiltAngle =  180;
        else if(xtilt < -1) tiltAngle = -180;
        else tiltAngle = -asin(xtilt)                                                                       * TO_GRADUS *-1;
        rollAngle = atan2(2 * ASq_3 * ASq_4 - 2 * ASq_1 * ASq_2, 2 * ASq_1 * ASq_1 + 2 * ASq_4 * ASq_4 - 1) * TO_GRADUS;
        panAngle =  atan2(2 * ASq_2 * ASq_3 - 2 * ASq_1 * ASq_4, 2 * ASq_1 * ASq_1 + 2 * ASq_2 * ASq_2 - 1) * TO_GRADUS *-1;

	if (rollAngle < 0)
          rollAngle += 180;
        else
          rollAngle -= 180;

/*
            // compute rotation matrix from quaternion
            r_11 = 2 * ASq_1 * ASq_1 - 1 + 2 * ASq_2 * ASq_2;
            r_12 = 2 * (ASq_2 * ASq_3 + ASq_1 * ASq_4);
            r_13 = 2 * (ASq_2 * ASq_4 - ASq_1 * ASq_3);
            r_21 = 2 * (ASq_2 * ASq_3 - ASq_1 * ASq_4);
            r_22 = 2 * ASq_1 * ASq_1 - 1 + 2 * ASq_3 * ASq_3;
            r_23 = 2 * (ASq_3 * ASq_4 + ASq_1 * ASq_2);
            r_31 = 2 * (ASq_2 * ASq_4 + ASq_1 * ASq_3);
            r_32 = 2 * (ASq_3 * ASq_4 - ASq_1 * ASq_2);
            r_33 = 2 * ASq_1 * ASq_1 - 1 + 2 * ASq_4 * ASq_4;
   

*/

    // Used to set initial values. 
    if (resetValues) {
#if FATSHARK_HT_MODULE
        digitalWrite(BUZZER, HIGH);
#endif
	digitalWrite(ARDUINO_LED, HIGH); 
        resetValues = 0; 

	AEq_1 = ASq_1;
	AEq_2 = ASq_2;
	AEq_3 = ASq_3;
	AEq_4 = ASq_4;

        lastTiltAngle=tiltStart = tiltAngle;
        lastPanAngle=panStart = panAngle;
        lastRollAngle=rollStart = rollAngle;

#if FATSHARK_HT_MODULE
        digitalWrite(BUZZER, LOW);
#endif
	digitalWrite(ARDUINO_LED, LOW); //ready

	TrackerStarted = 1;
    }


    if (TrackerStarted)  {
        // All low-pass filters
        tiltAngleLP = tiltAngle * tiltRollBeta + (1 - tiltRollBeta) * lastTiltAngle;
        lastTiltAngle = tiltAngleLP;
  
        rollAngleLP = rollAngle * tiltRollBeta + (1 - tiltRollBeta) * lastRollAngle;
        lastRollAngle = rollAngleLP;

        panAngleLP = panAngle * panBeta + (1 - panBeta) * lastPanAngle;
        lastPanAngle = panAngleLP;

	{
            float angleTemp = panAngleLP * panInverse * panFactor;
            if ( (angleTemp > -panMinPulse) && (angleTemp < panMaxPulse) )  {
                channel_value[htChannels[0]] = servoPanCenter + angleTemp;
            }    
	}
	{
            float angleTemp = (tiltAngleLP - tiltStart) * tiltInverse * tiltFactor;
            if ( (angleTemp > -tiltMinPulse) && (angleTemp < tiltMaxPulse) ) {
                channel_value[htChannels[1]] = servoTiltCenter + angleTemp;
            }
        }

	{
            float angleTemp = (rollAngleLP - rollStart) * rollInverse * rollFactor;
            if ( (angleTemp > -rollMinPulse) && (angleTemp < rollMaxPulse) ) {
                channel_value[htChannels[2]] = servoRollCenter + angleTemp;
            }
        }
    }
}


void trackerOutput()
{
  Serial.print(tiltAngleLP - tiltStart + 90);
  Serial.write(',');
  Serial.print(rollAngleLP - rollStart + 90);
  Serial.write(',');  
  Serial.println(panAngleLP + 180);
/*
  Serial.print(xtilt);
  Serial.write(',');  
  Serial.print(tiltAngleLP);
  Serial.write(',');  
  Serial.println(tiltAngle);
*/
}


//--------------------------------------------------------------------------------------
// Func: ResetCenter
// Desc: Utility for resetting tracker center. This is only called during tracker
//       startup. Button press resets are handled during filtering. (Needs refactor)
//--------------------------------------------------------------------------------------

/*
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
    
    TrackerStarted = 1;
}
*/

//--------------------------------------------------------------------------------------
// Func: SensorInfoPrint
// Desc: Prints the mag sensor data.
//--------------------------------------------------------------------------------------
void SensorInfoPrint()
{ 
/*
   Serial.printf_P(PSTR("Mag cal: %f,%f,%f,%f,%f,%f\n"), magNegOff[0] - magPosOff[0], magNegOff[1] - magPosOff[1], magNegOff[2] - magPosOff[2], 
					   magGain[0], magGain[1], magGain[2]);   

   Serial.printf_P(PSTR("ADXL345 ID: %d\nITG3205 ID: %d\nHMC ID: %d\n"), (int)ADXL345_ID, (int)ITG3205_ID,  (int)HMC_ID);
*/
//   Serial.printf_P(PSTR("R= %f gyroWeightTiltRoll=%f gyroWeightPan=%f\n"), R, gyroWeightTiltRoll, GyroWeightPan );
}


// ===============================================
//      ---------- Test functions -----------
// ===============================================


void testQuatData(){
/*    Serial.print(AEq_1);  
    Serial.write(' ');          
    Serial.print(AEq_2);  
    Serial.write(' ');              
    Serial.print(AEq_3);  
    Serial.write(' ');              
    Serial.println(AEq_4);  
*/
    Serial.print(q0);  
    Serial.write(' ');          
    Serial.print(q1);  
    Serial.write(' ');              
    Serial.print(q2);  
    Serial.write(' ');              
    Serial.println(q3);  
}

void testQuat2Data(){
/*    Serial.print(ASq_1);  
    Serial.write(' ');          
    Serial.print(ASq_2);  
    Serial.write(' ');              
    Serial.print(ASq_3);  
    Serial.write(' ');              
    Serial.println(ASq_4);  
*/

    Serial.print(rollAngle );  
    Serial.write(' ');          
    Serial.print(tiltAngle );  
    Serial.write(' ');              
    Serial.println(panAngle );  


}

void testAngleData(){
    Serial.print(rollAngleLP );  
    Serial.write(' ');          
    Serial.print(tiltAngleLP );  
    Serial.write(' ');              
    Serial.println(panAngleLP );  
}

void testAccOutput()
{
    Serial.print("RAW: ");  
    Serial.print(accRaw[0]);  
    Serial.write(' ');          
    Serial.print(accRaw[1]);  
    Serial.write(' ');              
    Serial.print(accRaw[2]);  

    Serial.print("\t G: ");
    Serial.print(accG[0]); 
    Serial.write(' ');    
    Serial.print(accG[1]); 
    Serial.write(' '); 
    Serial.print(accG[2]); 

    Serial.print("\t Ang: ");
    Serial.print(accAngle[0]); 
    Serial.write(' ');    
    Serial.print(accAngle[1]); 
    Serial.write(' '); 
    Serial.println(accAngle[2]);
}

void testGyroOutput()
{  
    Serial.print("RAW: ");  
    Serial.print(gyroRaw[0]);  
    Serial.write(',');
    Serial.print(gyroRaw[1]);  
    Serial.write(',');
    Serial.print(gyroRaw[2]);   

    Serial.print("\t Norm: ");  
    Serial.print(gyro[0]);  
    Serial.write(',');          
    Serial.print(gyro[1]);  
    Serial.write(',');
    Serial.print(gyro[2]);   

    Serial.print("\t Ang: ");  
    Serial.print(angle[0]);  
    Serial.write(',');          
    Serial.print(angle[1]);  
    Serial.write(',');              
    Serial.println(angle[2]);

}

void testMagOutput()
{
    Serial.print("RAW: ");  

    Serial.print(magRaw[0]);
    Serial.write(',');
    Serial.print(magRaw[1]);
    Serial.write(',');    
    Serial.print(magRaw[2]);    

    Serial.print("Ang: ");  
    Serial.print(magAngle[0]);  
    Serial.write(',');          
    Serial.print(magAngle[1]);  
    Serial.write(',');              
    Serial.println(magAngle[2]);      
}

void testTiltOutput()
{ 
    Serial.print(angle[1]);
    Serial.write(',');          
    Serial.print(accAngle[0]-tiltStart);
    Serial.write(',');          
    Serial.println(tiltAngle-tiltStart);
}

void testRollOutput()
{
    Serial.print(angle[0]);
    Serial.write(',');          
    Serial.print(accAngle[1]-rollStart);
    Serial.write(',');          
    Serial.println(rollAngle-rollStart);  
}

void testPanOutput()
{
    Serial.print(angle[2]);
    Serial.write(',');                  
    Serial.print(magAngle[2]);
    Serial.write(',');                  
    Serial.println(panAngle);
}

// output calculated values, output as "csv"
void testAllData()
{  
/*    // x
  Serial.print(angle[1]);
  Serial.write(',');          
  Serial.print(accAngle[0]-90);
  Serial.write(',');          
  Serial.print(tiltAngle-90);
  Serial.write(',');      
  
    // y
  Serial.print(angle[0]);
  Serial.write(',');          
  Serial.print(accAngle[1]-90);
  Serial.write(',');          
  Serial.print(rollAngle-90);  
  Serial.write(',');     
    
    // z
  Serial.print(angle[2]);
  Serial.write(',');                  
  Serial.print(magAngle[2]-panStart);
  Serial.write(',');                  
  Serial.println(panAngle);
*/
}

// All sensor output as "csv". 
void testAllSensors()
{  
/*    Serial.print(accRaw[0]);  
    Serial.write(',');          
    Serial.print(accRaw[1]);  
    Serial.write(',');              
    Serial.print(accRaw[2]);  

    Serial.write(',');            
    Serial.print(gyroRaw[0]);  
    Serial.write(',');          
    Serial.print(gyroRaw[1]);  
    Serial.write(',');              
    Serial.print(gyroRaw[2]);  

    Serial.write(',');               
    Serial.print(magRaw[0]);  
    Serial.write(',');          
    Serial.print(magRaw[1]);  
    Serial.write(',');              
    Serial.println(magRaw[2]);      
*/
}

/*
ATMEGA128RFA1
*/