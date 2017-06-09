
#define EEPROM_VERSION 12

#include <inttypes.h>

#pragma pack(push,1)
struct Settings {
//0
    uint8_t vers; 
    uint8_t servoReverseMask;

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

    uint8_t htChannels[4];

// NEW
    uint8_t PpmIn_PpmOut[16];

    float magGain[3];
    float accGain[3];

    float magDiagOff[3];
    float accDiagOff[3];
};

typedef struct Settings settings;

#pragma pack(pop)

extern settings sets;

void ReadSets(void);
void WriteSets(void);
void WriteSets(uint16_t addr, uint8_t length);
