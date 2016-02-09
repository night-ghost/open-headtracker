// File: Sensors.cpp
// Desc: Implementations sensor board functionality.
//-----------------------------------------------------------------------------
//#include <FastSerial.h>
#include <SingleSerial.h>

#include "config.h"
#include "Arduino.h"
#include "functions.h"
#include <Wire.h>
#include "sensors.h"

#include "FourtOrderFilter.h"

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

byte compass;
byte compass_mode=0;

unsigned char ADXL345_ID = 0;
unsigned char ITG3205_ID = 0;
unsigned char HMC_ID = 0;

// Variables defined elsewhere
//
extern long channel_value[];

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
float gyroAngle[3];          // Angle from gyro 
#if DEBUG
float angleRaw[3];           // Temp for angle-calculation
#endif
float gyro[3];

int   magRaw[3];             // Raw readings from magnetometer
float magV[3];               // Normalized readings from magnetometer
float MR=1;                  // magnet field normalization, 1 before calibration
float mx=0, my=0, mz=0;      // Calculated magnetometer values in all directions with pan/tilt compensation
float magAngle[3];           // Measured angles from magnetometer


float magPosOff[3]; // калибровочные данные магнетометра
float magNegOff[3];
float magGain[3];
float magMGain[3]={0,0,0}; // калибровочная нормировка по осям 

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

// Settings
//
settings sets;
/*
float tiltRollBeta = 0.75;
float panBeta = 0.75;
float gyroWeightTiltRoll = 0.98;
float gyroWeightPan = 0.98;

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

// дабы не считать дважды
float sin_tilt, cos_tilt, sin_roll, cos_roll;

extern float dynFactor;


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
void ReadFromI2C(int device, byte address, char bytesToRead){
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

void trackerOutput()
{
  Serial.print(tiltAngleLP - tiltStart );
  Serial.write(',');
  Serial.print(rollAngleLP - rollStart );
  Serial.write(',');  
  Serial.println(panAngleLP );
}
/*
void calMagOutput()
{
    Serial.printf_P(PSTR("%d,%d,%d\n"), magRaw[0], magRaw[1], magRaw[2]);
}

void calAccOutput()
{
    Serial.printf_P(PSTR("%d,%d,%d\n"), accRaw[0], accRaw[1], accRaw[2]);
}
*/


// for calibration
void calMagAccOutput()
{
    Serial.printf_P(PSTR("%d,%d,%d,%d,%d,%d\n"),  magRaw[0], magRaw[1], magRaw[2], accRaw[0], accRaw[1], accRaw[2]);
}



void readCompassRaw(void){    // Read x, y, z from magnetometer;
    ReadFromI2C(HMC_ADDR, HMC_X_ADDR, 6);

    magRaw[0] =     sb.sensorBuffer[1] | ((int)sb.sensorBuffer[0] << 8);

//    if (compass == AP_COMPASS_TYPE_HMC5883L) {
//        magRaw[1] = sb.sensorBuffer[3] | ((int)sb.sensorBuffer[2] << 8);
//        magRaw[2] = sb.sensorBuffer[5] | ((int)sb.sensorBuffer[4] << 8);
//    } else {
        magRaw[2] = sb.sensorBuffer[3] | ((int)sb.sensorBuffer[2] << 8);
        magRaw[1] = sb.sensorBuffer[5] | ((int)sb.sensorBuffer[4] << 8);
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
    Serial.printf_P(PSTR("Gyro offset measured: %f %f %f\n"), sets.gyroOff[0], sets.gyroOff[1], sets.gyroOff[2]);
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

    magTmp[0] /= AVG_COUNT;
    magTmp[1] /= AVG_COUNT;
    magTmp[2] /= AVG_COUNT;

    MR = sqrt(magTmp[0]*magTmp[0] + magTmp[1]*magTmp[1] + magTmp[2]*magTmp[2]);

    magMGain[0] = magGain[0]/(MR * sets.magGain[0]); // калибровочная нормировка
    magMGain[1] = magGain[1]/(MR * sets.magGain[1]);
    magMGain[2] = magGain[2]/(MR * sets.magGain[2]);

#if (DEBUG)
//    Serial.printf_P(PSTR("Mag type: %i\n"), compass);
    Serial.printf_P(PSTR("Mag cal: %f:%f, %f:%f, %f:%f, gain: %f %f %f\n"), magNegOff[0] , magPosOff[0], magNegOff[1], magPosOff[1], magNegOff[2], magPosOff[2], magGain[0], magGain[1], magGain[2]);
//    Serial.printf_P(PSTR("Mag offset: %f %f %f\n"), sets.magOffset[0], sets.magOffset[1], sets.magOffset[2]);
//    Serial.printf_P(PSTR("Mag gain:   %f %f %f\n"), sets.magGain[0], sets.magGain[1], sets.magGain[2]);

//    Serial.printf_P(PSTR("Mag tmp: %f %f %f\n"), magTmp[0], magTmp[1], magTmp[2]);
//    Serial.printf_P(PSTR("Mag norm: %f\n"), MR);
//    Serial.printf_P(PSTR("Mag norm gain: %f %f %f\n"), magMGain[0], magMGain[1], magMGain[2]);
#endif          
}

void getCompass(){
    readCompassRaw();

    if (magRaw[0] == -4096)
	magV[0]=0;
    else
        magV[0] =  (magRaw[0] - sets.magOffset[0]) * magMGain[0]; // / MR / sets.magGain[0];
    
    if (magRaw[1] == -4096)
	magV[1]=0;
    else
	magV[1] = (-magRaw[1] - sets.magOffset[1]) * magMGain[1];


    if (magRaw[2] == -4096)
	magV[2]=0;
    else
	magV[2] = (-magRaw[2] - sets.magOffset[2]) * magMGain[2];

}


//--------------------------------------------------------------------------------------
// Func: GyroCalc
// Desc: Calculate angle from gyro-data
//--------------------------------------------------------------------------------------
void GyroCalc() {
    readGyroRaw();

    gyro[0] = gyroRaw[0]-sets.gyroOff[0];
    gyro[1] = gyroRaw[1]-sets.gyroOff[1];
    gyro[2] = gyroRaw[2]-sets.gyroOff[2];

#if DEBUG
    for (byte i=0; i<3; i++) {
        angleRaw[i]  += gyro[i];
        gyroAngle[i]  = angleRaw[i] / (SAMPLERATE*SCALING_FACTOR);
    }
#endif
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
    
/*    accG[0] = ((float)accRaw[0] + sets.accOffset[0] ) / ACC_SENS;
    accG[1] = ((float)accRaw[1] + sets.accOffset[1] ) / ACC_SENS;
    accG[2] = ((float)accRaw[2] + sets.accOffset[2] ) / ACC_SENS;
*/
    accG[0] = ((float)accRaw[0] + sets.accOffset[0] ) * SCALE_FACTOR_X * GRAVITATIONAL_ACCELERATION / 1000.0 / 2 / sets.accGain[0];
    accG[1] = ((float)accRaw[1] + sets.accOffset[1] ) * SCALE_FACTOR_Y * GRAVITATIONAL_ACCELERATION / 1000.0 / 2 / sets.accGain[1];
    accG[2] = ((float)accRaw[2] + sets.accOffset[2] ) * SCALE_FACTOR_Z * GRAVITATIONAL_ACCELERATION / 1000.0 / 2 / sets.accGain[2];

}

void CalibrateAccel(){
#define ACCEL_AVG 64
    // So, lets calculate R
    // R^2 = Rx^2+Ry^2+Rz^2    
    float x=0,y=0,z=0;

// skip first measurements
/*    for(byte i=0;i<ACCEL_AVG;i++){
	delay(18);	// 50hz

	getAccel();
    }
*/
 
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
	    accAngle[i] = acos(v) * TO_GRADUS;
    }
*/

	float p;
	p=-accG[0] / R;
	if(p < -1)
	    accAngle[0] = -90;
	else if(p > 1)
	    accAngle[0] =  90;
	else
	    accAngle[0] = (p=asin(p) ) * TO_GRADUS;
	
	p=(accG[1] / R) / cos(p);
	
	if(accAngle[0] == -90 || accAngle[0] == 90)
	    accAngle[1]=0;
	else if(p < -1)
	    accAngle[1] = -90;
	else if(p > 1)
	    accAngle[1] = 90;
	else
	    accAngle[1] = asin( p ) * TO_GRADUS;

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


/*
    mx  = magV[0] * cos_tilt
        + magV[1] * sin_tilt;

    my =  magV[0] * sin_roll * sin_tilt
        + magV[2] * cos_roll
        - magV[1] * sin_roll * cos_tilt;
*/
  

// tilt/roll compensation

    mx  = magV[0] * cos_tilt
        + magV[2] * sin_tilt;

    my =  magV[0] * sin_roll * sin_tilt
        + magV[1] * cos_roll
	- magV[2] * sin_roll * cos_tilt;

    mz = -magV[0] * cos_roll * sin_tilt
        + magV[1] * sin_roll
	+ magV[2] * cos_roll * cos_tilt;
  
  
    if(mx==0 && my==0)
	magAngle[2]=90;
    else {
	// Calculate pan-angle from magnetometer. 
//	magAngle[2] = atan2(mx, my) * TO_GRADUS + 90;
	magAngle[2] = atan2(my, mx) * TO_GRADUS + 90;
    }
  
    if (magAngle[2] > 180) {
        magAngle[2] -= 360;
    } else if (magAngle[2] < -180) {
        magAngle[2] += 360; 
    }
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
    
    magMGain[0]=sets.magOffset[1]=sets.magOffset[2]=1;
}


//--------------------------------------------------------------------------------------
// Func: Filter
// Desc: Filters / merges sensor data. 
//--------------------------------------------------------------------------------------
void FilterSensorData()
{
    int temp = 0;

    // Used to set initial values. 
    if (resetValues) {
#if FATSHARK_HT_MODULE
        digitalWrite(BUZZER, HIGH);
#endif
	digitalWrite(ARDUINO_LED, HIGH); 
        resetValues = 0; 


        tiltStart = panStart = rollStart = 0;

	for(byte i=0; i<4; i++){  
            GyroCalc();
            AccelCalc();
            MagCalc();
        
            tiltStart = accAngle[0];
            rollStart = accAngle[1];
            panStart  = magAngle[2];
	}
	

        tiltStart /= 4;
        rollStart /= 4;
        panStart  /= 4;

        panAngle = 0;

//        tiltStart = tiltAngle;
//        panStart =  rollAngle;
//        rollStart = panAngle;


#if DEBUG
    Serial.printf_P(PSTR("Center reset! tilt=%f rol=%f pan=%f\n"), tiltStart, rollStart, panStart);
#endif

#if FATSHARK_HT_MODULE
        digitalWrite(BUZZER, LOW);
#endif
	digitalWrite(ARDUINO_LED, LOW); //ready
    }
    float ta = (tiltAngle - 90) * FROM_GRADUS,
	  ra = (rollAngle - 90) * FROM_GRADUS;


    // Filter the high frequency noise from vibrations 
    accG[0] = computeFourthOrder(accG[0], &fourthOrder[0]);
    accG[1] = computeFourthOrder(accG[1], &fourthOrder[1]);
    accG[2] = computeFourthOrder(accG[2], &fourthOrder[2]);

    cos_tilt=cos(ta);
    sin_tilt=sin(ta);
    cos_roll=cos(ra);
    sin_roll=sin(ra);


    float magA = magAngle[2] - panStart;
    if (magA > 180) {
        magA -= 360;
    } else if (magA < -180) {
        magA += 360; 
    }


    // Simple FilterSensorData, uses mainly gyro-data, but uses accelerometer to compensate for drift
    rollAngle = (rollAngle + (gyro[0] *  cos_tilt +  gyro[2] *  sin_tilt)                          / (SAMPLERATE * SCALING_FACTOR)) * sets.gyroWeightTiltRoll + accAngle[1] * (1 - sets.gyroWeightTiltRoll);
    tiltAngle = (tiltAngle + (gyro[1] *  cos_roll +  gyro[2] * -sin_roll)                          / (SAMPLERATE * SCALING_FACTOR)) * sets.gyroWeightTiltRoll + accAngle[0] * (1 - sets.gyroWeightTiltRoll);
    panAngle  = (panAngle  + (gyro[2] *  cos_tilt + (gyro[0] * -sin_tilt) + ( gyro[1] * sin_roll)) / (SAMPLERATE * SCALING_FACTOR)) * sets.gyroWeightPan      + magA        * (1 - sets.gyroWeightPan);

    if (TrackerStarted)  {
        // All low-pass filters
        tiltAngleLP = tiltAngle * sets.tiltRollBeta + (1 - sets.tiltRollBeta) * lastTiltAngle;
        lastTiltAngle = tiltAngleLP;
  
        rollAngleLP = rollAngle * sets.tiltRollBeta + (1 - sets.tiltRollBeta) * lastRollAngle;
        lastRollAngle = rollAngleLP;

        panAngleLP = panAngle * sets.panBeta        + (1 - sets.panBeta) * lastPanAngle;
        lastPanAngle = panAngleLP;

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
    ReadFromI2C(ITG3205_ADDR, 0x00, 1);
    ITG3205_ID = sb.sensorBuffer[0];
 
#if DEBUG
    Serial.printf_P(PSTR("ITG3205: %d\n"), ITG3205_ID);
#endif 
 
    ReadFromI2C(ADXL345_ADDR, 0x00, 1);
    ADXL345_ID = sb.sensorBuffer[0];
 
#if DEBUG
    Serial.printf_P(PSTR("    ADXL: %d\n"), ADXL345_ID); 
#endif  

    // Accelerometer increase G-range (+/- 16G)
    WriteToI2C(ADXL345_ADDR, 0x31, 0b00001011);
    ReadFromI2C(HMC_ADDR, 0x00, 1);
    HMC_ID = sb.sensorBuffer[0];
 
#if DEBUG
    Serial.printf_P(PSTR("    HMC: %d\n"), HMC_ID); 
#endif  

    WriteToI2C(ITG3205_ADDR, 22, 0x18 | 2); //Register 22 – DLPF, Full Scale - full range 98hz filter

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

        compass = AP_COMPASS_TYPE_HMC5883L;
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
	compass = AP_COMPASS_TYPE_HMC5843;
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
    Serial.printf_P(PSTR("Center reset v2! tilt=%f rol=%f pan=%f\n"), tiltStart, rollStart, panStart);
#endif
    
    TrackerStarted = 1;
}


//--------------------------------------------------------------------------------------
// Func: SensorInfoPrint
// Desc: Prints the mag sensor data.
//--------------------------------------------------------------------------------------
void SensorInfoPrint()
{ 
   Serial.printf_P(PSTR("Mag cal: %f,%f,%f,%f,%f,%f\n"), magNegOff[0] - magPosOff[0], magNegOff[1] - magPosOff[1], magNegOff[2] - magPosOff[2], 
					   magGain[0], magGain[1], magGain[2]);   

   Serial.printf_P(PSTR("ADXL345 ID: %d\nITG3205 ID: %d\nHMC ID: %d\n"), (int)ADXL345_ID, (int)ITG3205_ID,  (int)HMC_ID);

//   Serial.printf_P(PSTR("R= %f gyroWeightTiltRoll=%f gyroWeightPan=%f\n"), R, sets.gyroWeightTiltRoll, sets.gyroWeightPan );
}


// ===============================================
//      ---------- Test functions -----------
// ===============================================

#if DEBUG
void testAccOutput() // dbg1
{
    Serial.printf_P(PSTR("raw: %d,%d,%d\n"), accRaw[0], accRaw[1], accRaw[2]);
    Serial.printf_P(PSTR("G:   %f,%f,%f R=%f\n"), accG[0], accG[1], accG[2],R);
    Serial.printf_P(PSTR("ang: %f,%f,%f\n"), accAngle[0], accAngle[1], accAngle[2]);
}

void testGyroOutput() // dbg2
{  
    Serial.printf_P(PSTR("raw: %d,%d,%d\n"), gyroRaw[0], gyroRaw[1], gyroRaw[2]);
    Serial.printf_P(PSTR("ang: %f,%f,%f\n"), gyroAngle[0], gyroAngle[1], gyroAngle[2]);

}

void testMagOutput() // dbg4
{
    Serial.printf_P(PSTR("raw:  %d,%d,%d\n"), magRaw[0], magRaw[1], magRaw[2]);
    Serial.printf_P(PSTR("norm: %f,%f,%f\n"), magV[0], magV[1], magV[2]);
    Serial.printf_P(PSTR(" m*:  %f,%f,%f\n"), mx, my, mz);
    Serial.printf_P(PSTR("cal:  %f,%f,%f\n"), magAngle[0], magAngle[1], magAngle[2]);
}

void testTiltOutput()
{ 
    Serial.printf_P(PSTR("tilt: %f,%f,%f\n"), gyroAngle[1], accAngle[0]-tiltStart, tiltAngle-tiltStart);
}

void testRollOutput()
{
    Serial.printf_P(PSTR("roll: %f,%f,%f\n"), gyroAngle[0], accAngle[1]-rollStart, rollAngle-rollStart);
}

void testPanOutput()
{
    Serial.printf_P(PSTR("pan: %f,%f,%f\n"), gyroAngle[2], magAngle[2], panAngle);
}

// output calculated values, output as "csv"
void testAllData()
{  
    Serial.printf_P(PSTR("tilt: %f,%f,%f (0=%f)\n"), gyroAngle[1], accAngle[0], tiltAngle, tiltStart);
    Serial.printf_P(PSTR("roll: %f,%f,%f (0=%f)\n"), gyroAngle[0], accAngle[1], rollAngle, rollStart);
    Serial.printf_P(PSTR("pan:  %f,%f,%f (0=%f)\n"), gyroAngle[2], magAngle[2], panAngle,  panStart);
}

// All sensor output as "csv". 
void testAllSensors()
{  
    Serial.print(accRaw[0]);  
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
}
#endif

void clearSettings() {
    sets.vers = EEPROM_VERSION;

    sets.tiltRollBeta = 0.85;
    sets.panBeta = 0.85;
    sets.gyroWeightTiltRoll = 0.98;
    sets.gyroWeightPan = 0.98;

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

    byte i;
    for(i=0; i<13; i++)
	sets.PpmIn_PpmOut[i] = i;

}
