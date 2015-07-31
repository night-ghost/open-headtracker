//-----------------------------------------------------------------------------
// File: Sensors.h
// Desc: Declares sensor-related functions for the project.
//-----------------------------------------------------------------------------
#ifndef sensors_h
#define sensors_h

#include "Arduino.h"

extern byte compass;

void InitSensors();
void WriteToI2C(int device, byte address, byte val);
void ReadFromI2C(int device, byte address, char bytesToRead);
void UpdateSensors();
void GyroCalc();
void AccelCalc();
void MagCalc();

void SetGyroOffset();
void testPanOutput();
void trackerOutput();
void calMagOutput();
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
void testQuatData();
void testQuat2Data();

#endif // sensors_h

