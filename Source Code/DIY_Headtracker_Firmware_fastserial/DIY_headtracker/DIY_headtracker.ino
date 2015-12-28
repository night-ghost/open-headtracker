//-----------------------------------------------------------------------------
// Original project by Dennis Frie - 2012 - Dennis.frie@gmail.com
// Discussion: http://www.rcgroups.com/forums/showthread.php?t=1677559
//
// Other contributors to this code:
//  Mark Mansur (Mangus on rcgroups)
//  
// Version history:
// - 0.01 - 0.08 - Dennis Frie - preliminary releases
// - 1.01 - April 2013 - Mark Mansur - code clean-up and refactoring, comments
//      added. Added pause functionality, added settings retrieval commands.
//      Minor optimizations.
//-----------------------------------------------------------------------------

#include <FastSerial.h>

#include <Wire.h>
#include "config.h"
#include "functions.h"
#include "sensors.h"
#include <EEPROM.h>

/*
Channel mapping/config for PPM out:

1 = PPM CHANNEL 1
2 = PPM CHANNEL 2
3 = PPM CHANNEL 3
4 = PPM CHANNEL 4
5 = PPM CHANNEL 5
6 = PPM CHANNEL 6
7 = PPM CHANNEL 7
8 = PPM CHANNEL 8
9 = PPM CHANNEL 9
10 = PPM CHANNEL 10
11 = PPM CHANNEL 11
12 = PPM CHANNEL 12

Mapping example:
$123456789111CH
*/

FastSerialPort0(Serial);

// Local file variables
//
int frameNumber = 0;		    // Frame count since last debug serial output

char serial_data[101];          // Array for serial-data 
unsigned char serial_index = 0; // How many bytes have been received?
char string_started = 0;        // Only saves data if string starts with right byte

/*
char outputMag = 0;             // Stream magnetometer data to host
char outputAcc = 0;             // Stream accelerometer data to host
*/
char outputMagAcc = 0;          // Stream mag and accell data (for calibration on PC)
char outputTrack = 0;	        // Stream angle data to host
char outputDbg=0;

// Keep track of button press
char lastButtonState = 0;           // 0 is not pressed, 1 is pressed
unsigned long buttonDownTime = 0;   // the system time of the press
char pauseToggled = 0;              // Used to make sure we toggle pause only once per hold
char ht_paused = 0;

// External variables (defined in other files)
//
extern char read_sensors;
extern char resetValues;   

// Settings (Defined in sensors.cpp)
extern settings sets;

float dynFactor=1;


// End settings   

//--------------------------------------------------------------------------------------
// Func: setup
// Desc: Called by Arduino framework at initialization time. This sets up pins for I/O,
//       initializes sensors, etc.
//--------------------------------------------------------------------------------------
void setup()
{
    Serial.begin(SERIAL_BAUD);

    pinMode(PPM_OUT_PIN,OUTPUT); 
    
    pinMode(BUTTON_INPUT,INPUT);// Set button pin to input:
    digitalWrite(BUTTON_INPUT,HIGH);// Set internal pull-up resistor. 
  
    digitalWrite(0,LOW); // pull-down resistor
    digitalWrite(1,LOW);
    digitalWrite(2,HIGH);
    digitalWrite(3,HIGH);  
  
    pinMode(ARDUINO_LED,OUTPUT);    // Arduino LED
    digitalWrite(ARDUINO_LED, HIGH);
    

    Wire.begin();               // Start I2C


    byte vers=EEPROM.read(0); // first byte
    
    // If the device have just been programmed, write initial config/values to EEPROM:
    if ( vers != EEPROM_VERSION) {
//#if (DEBUG)
        Serial.printf_P(PSTR("New board (%d) - saving default values!\n"),vers);
//#endif    
	clearSettings();
    
        InitSensors();
        SaveSettings(); // all!
    }
 
    GetSettings();                 // Get settings saved in EEPROM

    InitSensors();                // Initialize I2C sensors
//#if ALWAYS_CAL_GYRO - uses delay() so should be before reinit Timer0
    SetGyroOffset();
//#endif     
    CalibrateAccel();
    CalibrateMag();

    ResetCenter();
//    resetValues=1;


#if FATSHARK_HT_MODULE
    pinMode(BUZZER,OUTPUT);         // Buzzer
    digitalWrite(BUZZER, HIGH);
    // Give it time to be noticed, then turn it off
    delay(100); // Note: only use delay here. This won't work when Timer0 is repurposed later in InitTimerInterrupt
    digitalWrite(BUZZER, LOW);
#endif

//
    InitTimerInterrupt();        // Start timer interrupt (for sensors)
    InitPWMInterrupt();         // Start PWM interrupt  

    digitalWrite(ARDUINO_LED, LOW); //ready

#if DEBUG
//    DebugOutput(); - not works
#endif

    Serial.printf_P(PSTR("$OK!$\n"));

}

int tail_cmp( char *pat){
    char *str=&serial_data[serial_index];
    int len=strlen(pat);
    return !strncmp(str-len,pat,len);
}


void parse_data(byte off, int *valuesReceived, byte n) {
    byte comma_index = 0;
    byte neg=0;

#if DEBUG
        int *ptr0=valuesReceived;
#endif

    for (byte k = 0; k < serial_index - off && comma_index < n; k++) {

#if 0 && DEBUG
        Serial.print(serial_data[k]);
#endif

        // Looking for comma
        if (serial_data[k] == ',') {
    	    if(neg){
    		*valuesReceived = -*valuesReceived;
    		neg=0;
    	    }
            comma_index++;
            valuesReceived++;
        } else if (serial_data[k] == '-') {
    	    neg=1;
        } else  {
            *valuesReceived = *valuesReceived * 10 + (serial_data[k] - '0');
        }
    }


    if(neg){ // the last one
	*valuesReceived = -*valuesReceived;
	neg=0;
    }

#if 0 && DEBUG
                Serial.print("\n>");
                for (unsigned char k = 0; k < comma_index+1; k++) {
                    Serial.print(ptr0[k]); 
                    Serial.write(',');
                }
                Serial.println();
#endif

}

//--------------------------------------------------------------------------------------
// Func: loop
// Desc: Called by the Arduino framework once per frame. Represents main program loop.
//--------------------------------------------------------------------------------------
void loop()
{  
    // Check input button for reset/pause request
    char buttonPressed = (digitalRead(BUTTON_INPUT) == 0);
    byte c;

    if ( buttonPressed && lastButtonState == 0)  { // только нажали 
        resetValues = 1; 
        buttonDownTime = 0;
        lastButtonState = 1;
    }
    
    if ( buttonPressed ) {
        if ( !pauseToggled && (buttonDownTime > BUTTON_HOLD_PAUSE_THRESH) )  {
            // Pause/unpause
            ht_paused = !ht_paused;
            resetValues = 1;
            pauseToggled = 1;
        }
    }  else  {
        pauseToggled = 0;
        lastButtonState = 0;
    }
    
    

#if defined(ADC_PIN)
   int val = analogRead(ADC_PIN);
   
   //0-1023 convert to 0.2..5
   dynFactor = (val / 1023) * (5-0.2)  + 0.2;
#endif
  
    
    // All this is used for communication with GUI 
    //
    if (Serial.available()>0)  {
        if (string_started == 1)  {
            // Read incoming byte
            serial_data[serial_index++] = Serial.read();
#ifdef ECHO
           Serial.write(serial_data[serial_index-1]);
#endif
            // If string ends with "CH" it's channel configuration, that have been received.
            // String must always be 12 chars/bytes and ending with CH to be valid. 
            if (serial_index == 14 && tail_cmp("CH"))  {               // To keep it simple, we will not let the channels be 0-initialized, but
							                // start from 1 to match actual channels. 
                for (unsigned char i = 0; i < 13; i++) {
                    // Update the dedicated PPM-in -> PPM-out array for faster performance.
                    if ((serial_data[i] - 48) < 14) {
                        sets.PpmIn_PpmOut[serial_data[i]-48] = i + 1;
                    }
                }
               
                Serial.printf_P(PSTR("Channel mapping received\n"));

               // Reset serial_index and serial_started
               serial_index = 0;
               string_started = 0;
            }
            
            // Configure headtracker
            else if (tail_cmp("HE"))  {
                // HT parameters are passed in from the PC in this order:
                //
                // 0 tiltRollBeta      
                // 1 panBeta       
                // 2 gyroWeightTiltRoll    
                // 3 gyroWeightPan 
                // 4 tiltFactor        
                // 5 panFactor          
                // 6 rollFactor
                // 7 servoReverseMask
                // 8 servoPanCenter
                // 9 panMinPulse 
                // 10 panMaxPulse
                // 11 servoTiltCenter
                // 12 tiltMinPulse
                // 13 tiltMaxPulse
                // 14 servoRollCenter
                // 15 rollMinPulse
                // 16 rollMaxPulse
                // 17 htChannels[0]  // pan            
                // 18 htChannels[1]  // tilt 
                // 19 htChannels[2]  // roll         
             
                // Parameters from the PC client need to be scaled to match our local
                // expectations

                Serial.printf_P(PSTR("HT config received: "));
           
                int valuesReceived[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

		parse_data(2, valuesReceived, 20);

                sets.tiltRollBeta  = (float)valuesReceived[0] / 100;  
                sets.panBeta       = (float)valuesReceived[1] / 100;
                sets.gyroWeightTiltRoll = (float)valuesReceived[2] / 100;
                sets.gyroWeightPan = (float)valuesReceived[3] / 100;
                sets.tiltFactor    = (float)valuesReceived[4] / 10;
                sets.panFactor     = (float)valuesReceived[5] / 10;
                sets.rollFactor    = (float)valuesReceived[6] / 10;

                sets.servoReverseMask = (unsigned char)valuesReceived[7];

                if(sets.servoReverseMask & HT_PAN_REVERSE_BIT) 
                    sets.panFactor *= -1;
                
                if(sets.servoReverseMask & HT_ROLL_REVERSE_BIT) 
                    sets.rollFactor *= -1; 
                
                if(sets.servoReverseMask & HT_TILT_REVERSE_BIT) 
                    sets.tiltFactor *= -1;
                

                sets.servoPanCenter = valuesReceived[8];
                sets.panMinPulse = valuesReceived[9];
                sets.panMaxPulse = valuesReceived[10];
         
                sets.servoTiltCenter = valuesReceived[11];
                sets.tiltMinPulse = valuesReceived[12];
                sets.tiltMaxPulse = valuesReceived[13];

                sets.servoRollCenter = valuesReceived[14];
                sets.rollMinPulse = valuesReceived[15];
                sets.rollMaxPulse = valuesReceived[16];
     
                sets.htChannels[0] = valuesReceived[17];
                sets.htChannels[1] = valuesReceived[18];
                sets.htChannels[2] = valuesReceived[19];

#if 0 &&  (DEBUG)
                Serial.println(sets.htChannels[0]);
                Serial.println(sets.htChannels[1]);
                Serial.println(sets.htChannels[2]);
#endif

                SaveSettings();

                serial_index = 0;
                string_started = 0;
            } // end configure headtracker

            // Debug info
            else if (tail_cmp("DEBUG")) {
                DebugOutput();
                serial_index = 0;
                string_started = 0; 
            }

            // Firmware version requested
            else if (tail_cmp("VERS")) {
                Serial.printf_P(PSTR("FW: %S\n"), FIRMWARE_VERSION_FLOAT);
                serial_index = 0;
                string_started = 0; 
            }
          
            // Start mag and accel data stream
            else if (tail_cmp("CMAS"))  {  
        	CalibrationStart();
                outputMagAcc = 1;
//                outputMag = 0;
//                outputAcc = 0;
                outputTrack = 0;
                serial_index = 0;
                string_started = 0;
            }        

            // Stop mag and accel data stream
            else if (tail_cmp("CMAE")) {
                outputMagAcc = 0;
//                outputMag = 0;
                outputTrack = 0;
//                outputAcc = 0;
                serial_index = 0;
                string_started = 0;
            }        

            // Start tracking data stream
            else if (tail_cmp("PLST")) {
                outputTrack = 1;
                outputMagAcc = 0;
//                outputMag = 0;
//                outputAcc = 0;
                serial_index = 0;
                string_started = 0; 
            }        

            // Stop tracking data stream
            else if (tail_cmp("PLEN")) {
                outputTrack = 0;
//                outputMag = 0;
//                outputAcc = 0;
                outputMagAcc = 0;
                serial_index = 0;
                string_started = 0; 
            }

            // reset center - like button press
            else if (tail_cmp("RESE")) {
                resetValues=1;
                serial_index = 0;
                string_started = 0; 
                Serial.printf_P(PSTR("Center set!"));
            }          

            // Save RAM settings to EEPROM
            else if (tail_cmp("SAVE")) {
                SaveSettings();
                serial_index = 0;
                string_started = 0; 
            }          

          
            // Calibrate gyro
/*            else if (tail_cmp("CALG")) { 
                //SetGyroOffset(); - uses delay() so should be before reinit Timer0
//              SaveSettings();
               
		Serial.printf_P(PSTR("Gyro offset measured: %f,%f,%f\n"), sets.gyroOff[0], sets.gyroOff[1], sets.gyroOff[2]);

                serial_index = 0;
                string_started = 0; 
            }
*/
#if DEBUG
            else if (tail_cmp("DBG1"))  {  
                outputDbg = 1;
                serial_index = 0;
                string_started = 0; 
            }        

            else if (tail_cmp("DBG2"))  {  
                outputDbg = 2;
                serial_index = 0;
                string_started = 0; 
            }        


            else if (tail_cmp("DBG3")) {
                outputDbg = 3;
                serial_index = 0;
                string_started = 0; 
            }


            else if (tail_cmp("DBG4")) {
                outputDbg = 4;
                serial_index = 0;
                string_started = 0; 
            }        

#if PPM_IN
            else if (tail_cmp("DBGC")) {
                outputDbg = 5;
                serial_index = 0;
                string_started = 0; 
            }        

#endif
            // Stop debug data stream 
            else if (tail_cmp("DBGE")) {  
        	outputDbg=0;
                serial_index = 0;
                string_started = 0; 
            }
#endif

            // Store magnetometer offset
            else if (tail_cmp("MAG")) {
                Serial.println(serial_data);
                int valuesReceived[5] = {0,0,0,0,0};

		parse_data(3, valuesReceived, 5);

                // Y and Z are swapped on purpose - no more!.
                sets.magOffset[0] = valuesReceived[0]/10.0;
                sets.magOffset[1] = valuesReceived[1]/10.0;
                sets.magOffset[2] = valuesReceived[2]/10.0;

                serial_index = 0;
                string_started = 0; 

                //SaveMagData();                
                WriteSets(offsetof(settings, magOffset ), sizeof(sets.magOffset));
#if DEBUG
		Serial.printf_P(PSTR("Mag offset stored %f,%f,%f\n"),  sets.magOffset[0], sets.magOffset[1], sets.magOffset[2]);
#endif
            }

            // Store magnetometer gain
            else if (tail_cmp("MGA")) {
                Serial.println(serial_data);
                int valuesReceived[5] = {0,0,0,0,0};

		parse_data(3, valuesReceived, 5);

                // Y and Z are swapped on purpose - no more!.
                sets.magGain[0] = valuesReceived[0]/10000.0;
                sets.magGain[1] = valuesReceived[1]/10000.0;
                sets.magGain[2] = valuesReceived[2]/10000.0;

                serial_index = 0;
                string_started = 0; 

                //SaveMagGain();                
                WriteSets(offsetof(settings, magGain ), sizeof(sets.magGain));
#if DEBUG
		Serial.printf_P(PSTR("Mag gain stored %f,%f,%f\n"),  sets.magGain[0], sets.magGain[1], sets.magGain[2]);
#endif
            }

            // Store accelerometer offset
            else if (tail_cmp("ACC")) {
                Serial.println(serial_data);
                int valuesReceived[5] = {0,0,0,0,0};

		parse_data(3, valuesReceived, 5);

                sets.accOffset[0] = valuesReceived[0]/10.0;
                sets.accOffset[1] = valuesReceived[1]/10.0;
                sets.accOffset[2] = valuesReceived[2]/10.0;

                serial_index = 0;
                string_started = 0; 

                //SaveAccelData();
                WriteSets(offsetof(settings, accOffset ), sizeof(sets.accOffset));
#if DEBUG
		Serial.printf_P(PSTR("Acc offset stored %f,%f,%f\n"),  sets.accOffset[0], sets.accOffset[1], sets.accOffset[2]);
#endif
            }

            // Store accelerometer gain
            else if (tail_cmp("ACG")) {
                Serial.println(serial_data);
                int valuesReceived[5] = {0,0,0,0,0};

		parse_data(3, valuesReceived, 5);

                sets.accGain[0] = valuesReceived[0]/10000.0;
                sets.accGain[1] = valuesReceived[1]/10000.0;
                sets.accGain[2] = valuesReceived[2]/10000.0;

                serial_index = 0;
                string_started = 0; 

//                SaveAccelGain();
                WriteSets(offsetof(settings, accGain ), sizeof(sets.accGain));
#if DEBUG
		Serial.printf_P(PSTR("Acc gain stored %f,%f,%f\n"),  sets.accGain[0], sets.accGain[1], sets.accGain[2]);
#endif
            }

            // Retrieve settings
            else if (tail_cmp("GSET")) {
                // Get Settings. Scale our local values to
                // real-world values usable on the PC side.
                //
                Serial.printf_P(PSTR("$SET$")); // something recognizable in the stream

		char c=',';
                Serial.print(sets.tiltRollBeta * 100);
                Serial.write(c);
                Serial.print(sets.panBeta * 100);
                Serial.write(c);
                Serial.print(sets.gyroWeightTiltRoll * 100);  
                Serial.write(c);
                Serial.print(sets.gyroWeightPan * 100);
                Serial.write(c);
                Serial.print(sets.tiltFactor * 10);
                Serial.write(c);
                Serial.print(sets.panFactor * 10);
                Serial.write(c);
                Serial.print(sets.rollFactor * 10);
                Serial.write(c);
                Serial.print(sets.servoReverseMask);
                Serial.write(c);
                Serial.print(sets.servoPanCenter);
                Serial.write(c);
                Serial.print(sets.panMinPulse);
                Serial.write(c);
                Serial.print(sets.panMaxPulse);
                Serial.write(c);
                Serial.print(sets.servoTiltCenter);
                Serial.write(c);
                Serial.print(sets.tiltMinPulse);
                Serial.write(c);
                Serial.print(sets.tiltMaxPulse);
                Serial.write(c);
                Serial.print(sets.servoRollCenter);
                Serial.write(c);
                Serial.print(sets.rollMinPulse);
                Serial.write(c);
                Serial.print(sets.rollMaxPulse);
                Serial.write(c);
                Serial.print(sets.htChannels[0]);
                Serial.write(c);
                Serial.print(sets.htChannels[1]);
                Serial.write(c);
                Serial.println(sets.htChannels[2]);

//                Serial.printf_P(PSTR("Settings Retrieved!\n"));

                serial_index = 0;
                string_started = 0;
            }
            else if (serial_index > 100)
            {
                // If more than 100 bytes have been received, the string is not valid.
                // Reset and "try again" (wait for $ to indicate start of new string). 
                serial_index = 0;
                string_started = 0;
            }
        }
        else if ((c=Serial.read()) == '$')
        {
            string_started = 1;
#ifdef ECHO
            Serial.write('$');
#endif
        }else {
#ifdef ECHO
	    Serial.write(c);
#endif
    	}
    } // serial port input

    // if "read_sensors" flag is set high, read sensors and update
    if (read_sensors == 1 && ht_paused == 0) {
        GyroCalc();
        AccelCalc();
        MagCalc();
        FilterSensorData();

        // Only output this data every X frames.
        if (frameNumber++ >= SERIAL_OUTPUT_FRAME_INTERVAL) {
            if (outputTrack)            trackerOutput();
            else if (outputMagAcc)      calMagAccOutput();
//            else if (outputMag)         calMagOutput(); 
//            else if (outputAcc)         calAccOutput();
            
#if DEBUG
            if(outputDbg == 1)		testAccOutput();
            if(outputDbg == 2)		testGyroOutput();
            if(outputDbg == 3)		testMagOutput();
            if(outputDbg == 4)		testAllData();
 #if PPM_IN
            if(outputDbg == 5)		testPPM_in();
 #endif
            outputDbg=0; // only once
#endif
            
            frameNumber = 0; 
        }

        // Will first update read_sensors when everything is done.  
        read_sensors = 0;
    }
}

//--------------------------------------------------------------------------------------
// Func: SaveSettings
// Desc: Saves device settings to EEPROM for retrieval at boot-up.
//--------------------------------------------------------------------------------------

inline void SaveSettings()
{  
    WriteSets();
    Serial.printf_P(PSTR("Settings saved!\n"));
}


//--------------------------------------------------------------------------------------
// Func: GetSettings
// Desc: Retrieves device settings from EEPROM.
//--------------------------------------------------------------------------------------
inline void GetSettings()
{  

    ReadSets();
}



//--------------------------------------------------------------------------------------
// Func: DebugOutput
// Desc: Outputs useful device/debug information to the serial port.
//--------------------------------------------------------------------------------------
void DebugOutput()
{
    Serial.printf_P(PSTR("\n\n\n------ Debug info------\n"));


    Serial.printf_P(PSTR("FW Version: %S"),FIRMWARE_VERSION_FLOAT);

    Serial.printf_P(PSTR("Mag type: %i\n"), compass);

/*    
    Serial.printf_P(PSTR("tiltRollBeta: %f\n"),sets.tiltRollBeta);
    Serial.printf_P(PSTR("panBeta: %f\n"),sets.panBeta);
 
    Serial.printf_P(PSTR("gyroWeightTiltRoll: %f\n"),sets.gyroWeightTiltRoll);
    Serial.printf_P(PSTR("GyroWeightPan: %f\n"), sets.gyroWeightPan); 
    Serial.printf_P(PSTR("servoPanCenter: %d\n"),sets.servoPanCenter); 
    Serial.printf_P(PSTR("servoTiltCenter: %d\n"),sets.servoTiltCenter); 
    Serial.printf_P(PSTR("servoRollCenter: %d\n"), sets.servoRollCenter); 

    Serial.printf_P(PSTR("tiltFactor: %f\n"),sets.tiltFactor);
    Serial.printf_P(PSTR("panFactor: %f\n"), sets.panFactor);
    Serial.printf_P(PSTR("rollFactor: %f\n"), sets.rollFactor);
*/
    Serial.printf_P(PSTR("Gyro offset stored %f,%f,%f\n"), sets.gyroOff[0], sets.gyroOff[1], sets.gyroOff[2]);
 
    Serial.printf_P(PSTR("Mag offset stored %f,%f,%f\n"),  sets.magOffset[0], sets.magOffset[1], sets.magOffset[2]);
    Serial.printf_P(PSTR("Mag gain stored %f,%f,%f\n"),  sets.magGain[0], sets.magGain[1], sets.magGain[2]);
 
    Serial.printf_P(PSTR("Acc offset stored %f,%f,%f\n"),  sets.accOffset[0], sets.accOffset[1], sets.accOffset[2]);
    Serial.printf_P(PSTR("Acc gain stored %f,%f,%f\n"),  sets.accGain[0], sets.accGain[1], sets.accGain[2]);
#if PPM_IN
    Serial.printf_P(PSTR("Input channels detected: %d\n"),  channelsDetected);
#endif
    SensorInfoPrint();
}




void ReadSets(void){
    int i;
    for(i=0; i<sizeof(settings); i++)
	((byte *)&sets)[i] = EEPROM.read( i ); // EEPROM.read(EEPROM_offs(sets) + i );
}


void WriteSets(void){
    ht_paused=1;

    int i;
//    noInterrupts();
    for(i=0; i<sizeof(settings); i++) {
#if 0 && DEBUG
        Serial.print( i);
        Serial.write(' ');
        Serial.println(((byte *)&sets)[i]); 
#endif
	EEPROM.write( i, ((byte *)&sets)[i] ); // .write(EEPROM_offs(sets) + i,...
    }
//    interrupts();
    ht_paused=0;
}

void WriteSets(int addr, int length){
    ht_paused=1;

#if 0 && DEBUG
Serial.printf_P(PSTR("WriteSets addr=%d len=%d size=%d\n"), addr, length, sizeof(settings));
#endif
    int i;

//    noInterrupts();
    for(i=0; (addr+i)<sizeof(settings) && i<length; i++) {
	EEPROM.write( addr+i, ((byte *)&sets)[addr+i] ); // .write(EEPROM_offs(sets) + i,...
#if 0 && DEBUG
        Serial.print( addr+i);
        Serial.write(' ');
        Serial.println(((byte *)&sets)[addr+i]); 
#endif
    }
//    interrupts();
    ht_paused=0;
}
