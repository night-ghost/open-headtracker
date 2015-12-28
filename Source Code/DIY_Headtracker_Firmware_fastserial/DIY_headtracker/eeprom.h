#include <FastSerial.h>

#include <Arduino.h>

#define EEPROM_VERSION 11

#include <EEPROM.h>

#pragma pack(push,1)

struct Settings {
//0
    byte vers; 
    byte servoReverseMask;

    int servoPanCenter;
    int servoTiltCenter;
    int servoRollCenter;

    int panMaxPulse;
    int panMinPulse;

    int tiltMaxPulse;
    int tiltMinPulse;

    int rollMaxPulse;
    int rollMinPulse;

    float tiltRollBeta;
    float panBeta;
    float gyroWeightTiltRoll;
    float gyroWeightPan;
  
    float tiltFactor;
    float panFactor;
    float rollFactor;

    float magOffset[3];
    float gyroOff[3];
    float accOffset[3];

    byte htChannels[4];

// NEW
    byte PpmIn_PpmOut[16];

    float magGain[3];
    float accGain[3];
};

typedef struct Settings settings;

#pragma pack(pop)

extern settings sets;

void ReadSets(void);
void WriteSets(void);
void WriteSets(int addr, int length);
