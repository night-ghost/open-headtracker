#include "eeprom.h"

extern settings sets;
/*

void ReadSets(void){
    int i;
    for(i=0; i<sizeof(settings); i++)
	((byte *)&sets)[i] = EEPROM.read( i ); // EEPROM.read(EEPROM_offs(sets) + i );
}


void WriteSets(void){
    int i;
    for(i=0; i<sizeof(settings); i++) {
Serial.printf_P("ww_write %h %h\n", i, ((byte *)&sets)[i]);
	EEPROM.write( i, ((byte *)&sets)[i] ); // .write(EEPROM_offs(sets) + i,...
}
}

void WriteSets(int addr, int length){
    int i;
    for(i=addr; i<sizeof(settings) && i<length; i++)
	EEPROM.write( i, ((byte *)&sets)[i] ); // .write(EEPROM_offs(sets) + i,...
}
*/



/*
    tiltRollBeta    = (float)EEPROM.read(1) / 100;
    panBeta         = (float)EEPROM.read(2) / 100;
    gyroWeightTiltRoll = (float)EEPROM.read(3) / 100;
    GyroWeightPan   = (float)EEPROM.read(4) / 100;  
  

    // 6 unused

    tiltFactor      = (float)(EEPROM.read(9) + (EEPROM.read(10) << 8)) / 10;
    panFactor       = (float)(EEPROM.read(11) + (EEPROM.read(12) << 8)) / 10;
    rollFactor       = (float)(EEPROM.read(13) + (EEPROM.read(14) << 8)) / 10;  


    unsigned char temp = EEPROM.read(5);
    if ( temp & HT_TILT_REVERSE_BIT ) {
        tiltFactor *= -1;
    }  
    if ( temp & HT_ROLL_REVERSE_BIT ) {
        rollFactor *= -1;
    }
    if ( temp & HT_PAN_REVERSE_BIT ) {
        panFactor *= -1;
    }

    // 15 unused

    servoPanCenter  = EEPROM.read(7) + (EEPROM.read(8) << 8);
    servoTiltCenter = EEPROM.read(16) + (EEPROM.read(17) << 8);
    servoRollCenter = EEPROM.read(18) + (EEPROM.read(19) << 8);  
  
    panMaxPulse   = EEPROM.read(20) + (EEPROM.read(21) << 8);  
    panMinPulse   = EEPROM.read(22) + (EEPROM.read(23) << 8);    
  
    tiltMaxPulse  = EEPROM.read(24) + (EEPROM.read(25) << 8);
    tiltMinPulse  = EEPROM.read(26) + (EEPROM.read(27) << 8);
  
    rollMaxPulse  = EEPROM.read(28) + (EEPROM.read(29) << 8);
    rollMinPulse  = EEPROM.read(30) + (EEPROM.read(31) << 8);

    htChannels[0] = EEPROM.read(32);  
    htChannels[1] = EEPROM.read(33);  
    htChannels[2] = EEPROM.read(34);    
  
    gyroOff[0] = EEPROM.read(35) + (EEPROM.read(36) << 8) - 500; 
    gyroOff[1] = EEPROM.read(37) + (EEPROM.read(38) << 8) - 500; 
    gyroOff[2] = EEPROM.read(39) + (EEPROM.read(40) << 8) - 500;   
  
    magOffset[0] = EEPROM.read(200) + (EEPROM.read(201) << 8) - 2000;
    magOffset[1] = EEPROM.read(202) + (EEPROM.read(203) << 8) - 2000;
    magOffset[2] = EEPROM.read(204) + (EEPROM.read(205) << 8) - 2000;
  
    accOffset[0] = EEPROM.read(206) + (EEPROM.read(207) << 8) - 2000;
    accOffset[1] = EEPROM.read(208) + (EEPROM.read(209) << 8) - 2000;
    accOffset[2] = EEPROM.read(210) + (EEPROM.read(211) << 8) - 2000;
*/

