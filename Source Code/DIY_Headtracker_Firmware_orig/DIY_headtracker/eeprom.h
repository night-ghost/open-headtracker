#include <Arduino.h>

struct Settings {
    byte vers; // 8

    float tiltRollBeta;
    float panBeta;
    float gyroWeightTiltRoll;
    float gyroWeightPan;
    byte servoReverseMask;
    
    byte pad_6; // 6 unused
  
    int iServoPanCenter;
  
    float tiltFactor;
    float panFactor;
    float rollFactor;

    byte pad_15;// 15 unused

    int servoTiltCenter;
    int servoRollCenter;

    int panMaxPulse;
    int panMinPulse;

    int tiltMaxPulse;
    int tiltMinPulse;
    
    int rollMaxPulse;
    int rollMinPulse;
  
    byte htChannels[3];
  
    // Saving gyro calibration values
    float gyroOff[3]; // 35
    float magOffset[3];
    float accOffset[3];
};

