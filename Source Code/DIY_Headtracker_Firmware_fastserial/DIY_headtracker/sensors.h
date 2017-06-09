//-----------------------------------------------------------------------------
// File: Sensors.h
// Desc: Declares sensor-related functions for the project.
//-----------------------------------------------------------------------------
#ifndef sensors_h
#define sensors_h

#include "Arduino.h"
#include "eeprom.h"

extern byte compass_id;

void InitSensors();
void WriteToI2C(byte device, byte address, byte val);
void ReadFromI2C(byte device, byte address, byte bytesToRead);
void UpdateSensors();
void GyroCalc();
void AccelCalc();
void MagCalc();

void SetGyroOffset();
void testPanOutput();
void trackerOutput();
void calMagOutput();
void getCompass();
void calAccOutput(); 
void calMagAccOutput(); // Output both mag and accel in one pass.
void CalibrateMag();
void FilterSensorData();
void ResetCenter();
void SensorInfoPrint();
void CalibrateAccel();

void testAllSensors();
void testRollOutput();
void testAllData();
void testAccOutput();
void testGyroOutput();
void testMagOutput();
void CalcMagAngle();
void clearSettings();
void CalibrationStart();

extern uint8_t outputDbg;
#endif // sensors_h

