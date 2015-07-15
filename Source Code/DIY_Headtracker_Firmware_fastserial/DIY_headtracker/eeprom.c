#include "eeprom.h"

#if 0

void ReadSets(void){
    for(int i=0; i<sizeof(Settings); i++)
	((byte *)&sets)[i] = EEPROM.read( i ); // EEPROM.read(EEPROM_offs(sets) + i );
}


void WriteSets(void){
    for(int i=0; i<sizeof(Settings); i++)
	EEPROM.write( i, ((byte *)&sets)[i] ); // .write(EEPROM_offs(sets) + i,...
}

void WriteSets(int addr, int length){
    for(int i=addr; i<sizeof(Settings) && i<length; i++)
	EEPROM.write( i, ((byte *)&sets)[i] ); // .write(EEPROM_offs(sets) + i,...
}




#endif
