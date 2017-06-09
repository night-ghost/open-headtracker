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

//#include <FastSerial.h>
#include <SingleSerial.h>
#include <Arduino.h>

#include "config.h"
#include "functions.h"
#include "sensors.h"

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


//FastSerialPort0(Serial);
SingleSerialPort(Serial);

// Local file variables
//
int frameNumber = 0;		    // Frame count since last debug serial output


#define SERIAL_BUF_LEN 100
char serial_data[SERIAL_BUF_LEN+1];          // Array for serial-data 
uint8_t in_count = 0; // How many bytes have been received?
char string_started = 0;        // Only saves data if string starts with right byte

/*
char outputMag = 0;             // Stream magnetometer data to host
char outputAcc = 0;             // Stream accelerometer data to host
*/
char outputMagAcc = 0;          // Stream mag and accell data (for calibration on PC)
char outputTrack = 0;	        // Stream angle data to host
uint8_t outputDbg=0;

// Keep track of button press
char lastButtonState = 0;           // 0 is not pressed, 1 is pressed
unsigned long buttonDownTime = 0;   // the system time of the press
char pauseToggled = 0;              // Used to make sure we toggle pause only once per hold
byte ht_paused = 0;

// External variables (defined in other files)
//
extern char read_sensors;
extern char resetValues;   

// Settings (Defined in sensors.cpp)
extern settings sets;

float dynFactor=1;
bool do_output=false;
uint8_t start_count = SAMPLERATE / 2; // reset after 0.5 seconds

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
  
    pinMode(ARDUINO_LED,OUTPUT);    // Arduino LED
    digitalWrite(ARDUINO_LED, HIGH);

#ifdef HEARTBEAT_LED
    pinMode(HEARTBEAT_LED,OUTPUT);    // Arduino LED
    digitalWrite(HEARTBEAT_LED, LOW);
#endif
    
    byte vers= eeprom_read_byte(0); // first byte
    
    // If the device have just been programmed, write initial config/values to EEPROM:
    if ( vers != EEPROM_VERSION) {
        serial_printi_1(PSTR("New board (%d) - saving default values!\n"),vers);
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

    resetValues=1;

#if FATSHARK_HT_MODULE
    pinMode(BUZZER,OUTPUT);         // Buzzer
    digitalWrite(BUZZER, HIGH);    // Give it time to be noticed, then turn it off
    delay(100); 
//    serial_print_P(PSTR("#beep\n"));

    digitalWrite(BUZZER, LOW);
#endif


    InitTimerInterrupt();        // Start timer interrupt (for sensors)
    InitPWMInterrupt();         // Start PWM interrupt  

    digitalWrite(ARDUINO_LED, LOW); //ready

    serial_print_P(PSTR("$OK!$\n"));
}

int tail_cmp(const char *pat){
    uint8_t inp_len = strlen(serial_data);
    char *str=&serial_data[inp_len];
    uint8_t len=strlen_P(pat);

    if(len>=inp_len) return false;
    
//Serial.printf_P(PSTR("Compare %s-%S\n"), str-len, pat);
    return !strncmp_P(str-len,pat,len);
}


uint8_t parse_data(byte off, float *valuesReceived, byte n) {
    byte comma_index = 0;

#if DEBUG
        float *ptr0=valuesReceived;
#endif

    for (byte k = 0; k < in_count - off && comma_index < n; k++) {

#if 0 && DEBUG
        serial_print(serial_data[k]);
#endif

        *valuesReceived = atof(&serial_data[k]); // parse number

        while(serial_data[k]){
            if(serial_data[k++]==',') break; // skip parsed number
        }
        
        if(serial_data[k]==0) break;
        
        comma_index++;
        valuesReceived++;
    }

    return comma_index;
}




void parseCalibData(float div, uint16_t offs, uint8_t size){
    float valuesReceived[5] = {0,0,0,0,0};

    parse_data(3, valuesReceived, 5);

    float *fp = (float *)((byte *)(&sets) + offs);

    fp[0] = float_div(valuesReceived[0],div);
    fp[1] = float_div(valuesReceived[1],div);
    fp[2] = float_div(valuesReceived[2],div);

    in_count = 0;
    string_started = 0; 

    WriteSets(offs, size );
}


uint32_t max_loop_time=0;






/////////////////////
static const PROGMEM char cmd01[] = "CH"; 
#define CMD_CH 1
static const PROGMEM char cmd02[] = "HE";
#define CMD_HE 2
static const PROGMEM char cmd03[] = "DEBUG";
#define CMD_DEBUG 3
static const PROGMEM char cmd04[] = "VERS";
#define CMD_VERS 4
static const PROGMEM char cmd05[] = "CMAS";
#define CMD_CMAS 5
static const PROGMEM char cmd06[] = "CMAE";
#define CMD_CMAE 6
static const PROGMEM char cmd07[] = "PLST";
#define CMD_PLST 7
static const PROGMEM char cmd08[] = "PLEN";
#define CMD_PLEN 8
static const PROGMEM char cmd09[] = "RESE";
#define CMD_RESE 9
static const PROGMEM char cmd10[] = "SAVE";
#define CMD_SAVE 10
static const PROGMEM char cmd11[] = "MAG";
#define CMD_MAG 11
static const PROGMEM char cmd12[] = "MGA";
#define CMD_MGA 12
static const PROGMEM char cmd13[] = "MDA";
#define CMD_MDA 13
static const PROGMEM char cmd14[] = "ACC";
#define CMD_ACC 14
static const PROGMEM char cmd15[] = "ACG";
#define CMD_ACG 15
static const PROGMEM char cmd16[] = "ACD";
#define CMD_ACD 16
static const PROGMEM char cmd17[] = "GSET";
#define CMD_GSET 17

static const PROGMEM char cmd18[] = "DBG1";
#define CMD_DBG1 18
static const PROGMEM char cmd19[] = "DBG2";
#define CMD_DBG2 19
static const PROGMEM char cmd20[] = "DBG3";
#define CMD_DBG3 20
static const PROGMEM char cmd21[] = "DBG4";
#define CMD_DBG4 21
static const PROGMEM char cmd22[] = "DBGE";
#define CMD_DBGE 22
static const PROGMEM char cmd23[] = "DBGC";
#define CMD_DBGC 23
static const PROGMEM char cmd24[] = "CLR";
#define CMD_CLR 24

static const PROGMEM char * const commands[]={
    cmd01, cmd02, cmd03, cmd04, cmd05,
    cmd06, cmd07, cmd08, cmd09, cmd10,
    cmd11, cmd12, cmd13, cmd14, cmd15,
    cmd16, cmd17, cmd18, cmd19, cmd20,
    cmd21, cmd22, cmd23, cmd24, 
};

#define NUM_COMMANDS (sizeof( commands)/sizeof(char *))

////////////////////


uint8_t parse_command(){
    for(uint8_t i=0; i<NUM_COMMANDS;i++){
        if( tail_cmp(pgm_read_word(&commands[i])) ) return i+1; // not 0
    }
    return 0;
}








//--------------------------------------------------------------------------------------
// Func: loop
// Desc: Called by the Arduino framework once per frame. Represents main program loop.
//--------------------------------------------------------------------------------------
void loop()
{  

    interrupts();
    
    
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
  uint16_t val = analogRead(ADC_PIN);
   
   //0-1023 convert to 0.2..5
   dynFactor = (val / 1023) * (5-0.2)  + 0.2;
#endif
  
    
// All this is used for communication with GUI 

    
    bool got_string=false;

    while(Serial.available_S()) {
        char c = Serial.read_S();

//Serial.printf_P(PSTR("\n#<%x\n"),c);

        switch(c) {
        case '\r':
            break;
    
        case '\n':
            if(in_count) got_string=true;
            in_count=0;
            break;

        default:
            if(in_count >= SERIAL_BUF_LEN) in_count=0;

            serial_data[in_count++] = c;
            serial_data[in_count]=0; // closed string
        }
    }
    

#if defined(DEBUG) && 0

    if (got_string){
            Serial.printf_P(PSTR("\n#<%s\n"),serial_data);
    }
#endif
    
    
    if (got_string && serial_data[0] == '$')  { // command
        switch(parse_command()) {
        case 0: // not found            
            Serial.printf_P(PSTR("\n#bad command: %s\n"),serial_data);
            break;

        case CMD_CLR: // clear EEPROM to defaults
            clearSettings();
            WriteSets();
            serial_print_P(PSTR("EEPROM cleared\n"));

            break;
        
        case CMD_CH: {  // If string ends with "CH" it's channel configuration, that have been received.
                    // String must always be 12 chars/bytes and ending with CH to be valid. 

                float valuesReceived[12] = {0,0,0,0,0,0,0,0,0,0,0,0,};

		uint8_t n = parse_data(2, valuesReceived, 12);
							                // start from 1 to match actual channels. 
                for (uint8_t i = 0; i < 12; i++) {
                    // Update the dedicated PPM-in -> PPM-out array for faster performance.
                    sets.PpmIn_PpmOut[(int)valuesReceived[i]] = i + 1;                    
                }
               
                serial_print_P(PSTR("Channel mapping received\n"));
            }
            break;
            
        case CMD_HE: {           // Configure headtracker
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

                serial_print_P(PSTR("HT config received: "));
           
                float valuesReceived[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

		parse_data(2, valuesReceived, 20);

                sets.tiltRollBeta       = valuesReceived[0];  
                sets.panBeta            = valuesReceived[1];
                sets.gyroWeightTiltRoll = 1/valuesReceived[2];
                sets.gyroWeightPan      = 1/valuesReceived[3];
                sets.tiltFactor         = float_div(valuesReceived[4], 10);
                sets.panFactor          = float_div(valuesReceived[5], 10);
                sets.rollFactor         = float_div(valuesReceived[6], 10);

                sets.servoReverseMask = (uint8_t)valuesReceived[7];

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

            } // end configure headtracker

            break;
            
        case CMD_DEBUG:          // Debug info
            DebugOutput();
            break;
            
        case CMD_VERS:            // Firmware version requested
            Serial.printf_P(PSTR("FW: %S\n"), FIRMWARE_VERSION);
            break;
            
        case CMD_CMAS:             // Start mag and accel data stream
            CalibrationStart(); // reset normas
            outputMagAcc = 1;
            outputTrack = 0;
            break;
            
        case CMD_CMAE:             // Stop mag and accel data stream
            outputMagAcc = 0;
            outputTrack = 0;
            break;
            
        case CMD_PLST:             // Start tracking data stream
            outputTrack = 1;
            outputMagAcc = 0;
            break;
            
        case CMD_PLEN:            // Stop tracking data stream
            outputTrack = 0;
            outputMagAcc = 0;
            
            break;
            
        case CMD_RESE:             // reset center - like button press
            resetValues=1;
            serial_print_P(PSTR("Center set!"));
            break;
            
        case CMD_SAVE:             // Save RAM settings to EEPROM
            SaveSettings();
            break;
          
            // Calibrate gyro
/*            else if (tail_cmp("CALG")) { 
                //SetGyroOffset(); - uses delay() so should be before reinit Timer0
//              SaveSettings();
               
		Serial.printf_P(PSTR("Gyro offset measured: %f,%f,%f\n"), sets.gyroOff[0], sets.gyroOff[1], sets.gyroOff[2]);
            }
            break;
*/
#if DEBUG
            
        case CMD_DBG1:
            outputDbg = 1;
            break;
            
        case CMD_DBG2: 
            outputDbg = 2;
            break;
            
        case CMD_DBG3:
            outputDbg = 3;
            break;
            
        case CMD_DBG4:
            outputDbg = 4;
            break;
            
#if PPM_IN            
        case CMD_DBGC:
            outputDbg = 5;
            break;
#endif
            
        case CMD_DBGE:            // Stop debug data stream 
            outputDbg=0;
            break;
#endif
            
        case CMD_MAG: {            // Store magnetometer offset
                parseCalibData(10, offsetof(settings, magOffset ), sizeof(sets.magOffset));
                
                /*
                float valuesReceived[5] = {0,0,0,0,0};

		parse_data(3, valuesReceived, 5);

                // Y and Z are swapped on purpose - no more!.
                sets.magOffset[0] = valuesReceived[0]/10.0;
                sets.magOffset[1] = valuesReceived[1]/10.0;
                sets.magOffset[2] = valuesReceived[2]/10.0;

                WriteSets(offsetof(settings, magOffset ), sizeof(sets.magOffset));
                */
#if DEBUG
		serial_printf_3(PSTR("Mag offset stored %f,%f,%f\n"),  sets.magOffset[0], sets.magOffset[1], sets.magOffset[2]);
#endif
            }

            break;
            
        case CMD_MGA: {           // Store magnetometer gain

                parseCalibData(10000, offsetof(settings, magGain ), sizeof(sets.magGain));
            /*
                float valuesReceived[5] = {0,0,0,0,0};

		parse_data(3, valuesReceived, 5);

                sets.magGain[0] = valuesReceived[0]/10000.0;
                sets.magGain[1] = valuesReceived[1]/10000.0;
                sets.magGain[2] = valuesReceived[2]/10000.0;

                WriteSets(offsetof(settings, magGain ), sizeof(sets.magGain));
            */
#if DEBUG
		serial_printf_3(PSTR("Mag gain stored %f,%f,%f\n"),  sets.magGain[0], sets.magGain[1], sets.magGain[2]);
#endif
            }
            break;
            
        case CMD_MDA: {            // Store magnetometer diag offs
                parseCalibData(10000, offsetof(settings, magDiagOff ), sizeof(sets.magDiagOff));
                
            /*
                float valuesReceived[5] = {0,0,0,0,0};

		parse_data(3, valuesReceived, 5);

                sets.magDiagOff[0] = valuesReceived[0]/10000.0;
                sets.magDiagOff[1] = valuesReceived[1]/10000.0;
                sets.magDiagOff[2] = valuesReceived[2]/10000.0;

                WriteSets(offsetof(settings, magDiagOff ), sizeof(sets.magDiagOff));
            */
#if DEBUG
		serial_printf_3(PSTR("Mag diag offs stored %f,%f,%f\n"),  sets.magDiagOff[0], sets.magDiagOff[1], sets.magDiagOff[2]);
#endif
            }
            break;
            
        case CMD_ACC: {            // Store accelerometer offset

                parseCalibData(10, offsetof(settings, accOffset ), sizeof(sets.accOffset));

            /*
                float valuesReceived[5] = {0,0,0,0,0};

		parse_data(3, valuesReceived, 5);

                sets.accOffset[0] = valuesReceived[0]/10.0;
                sets.accOffset[1] = valuesReceived[1]/10.0;
                sets.accOffset[2] = valuesReceived[2]/10.0;

                WriteSets(offsetof(settings, accOffset ), sizeof(sets.accOffset));
            */
            
#if DEBUG
		serial_printf_3(PSTR("Acc offset stored %f,%f,%f\n"),  sets.accOffset[0], sets.accOffset[1], sets.accOffset[2]);
#endif
            }

            break;
            
        case CMD_ACG: {            // Store accelerometer gain
            
                parseCalibData(10000, offsetof(settings, accGain ), sizeof(sets.accGain));
                
            /*                
                float valuesReceived[5] = {0,0,0,0,0};

		parse_data(3, valuesReceived, 5);

                sets.accGain[0] = valuesReceived[0]/10000.0;
                sets.accGain[1] = valuesReceived[1]/10000.0;
                sets.accGain[2] = valuesReceived[2]/10000.0;

                in_count = 0;
                string_started = 0; 

//                SaveAccelGain();
                WriteSets(offsetof(settings, accGain ), sizeof(sets.accGain));
            */
#if DEBUG
		serial_printf_3(PSTR("Acc gain stored %f,%f,%f\n"),  sets.accGain[0], sets.accGain[1], sets.accGain[2]);
#endif
            }
            break;
            
        case CMD_ACD: {
                parseCalibData(10000, offsetof(settings, accDiagOff), sizeof(sets.accDiagOff));
                
            /*
                float valuesReceived[5] = {0,0,0,0,0};

		parse_data(3, valuesReceived, 5);

                sets.accDiagOff[0] = valuesReceived[0]/10000.0;
                sets.accDiagOff[1] = valuesReceived[1]/10000.0;
                sets.accDiagOff[2] = valuesReceived[2]/10000.0;

                in_count = 0;
                string_started = 0; 

//                SaveAccelGain();
                WriteSets(offsetof(settings, accDiagOff ), sizeof(sets.accDiagOff));
            */
#if DEBUG
		serial_printf_3(PSTR("Acc gain stored %f,%f,%f\n"),  sets.accDiagOff[0], sets.accDiagOff[1], sets.accDiagOff[2]);
#endif
            }

            break;
            
        case CMD_GSET: {        // Retrieve settings
                // Get Settings. Scale our local values to
                // real-world values usable on the PC side.
                //
                float wgr, wgp;
                
                wgr = (sets.gyroWeightTiltRoll==0) ?10: 1/sets.gyroWeightTiltRoll;
                wgp = (sets.gyroWeightPan==0)      ?10: 1/sets.gyroWeightPan;
                
                serial_printf_3a(PSTR("$SET$%f,%f,%f,"),  sets.tiltRollBeta, sets.panBeta, wgr);
                serial_printf_3a(PSTR("%f,%f,%f,"),       wgp, sets.tiltFactor * 10, sets.panFactor * 10);
                serial_printf_3a(PSTR("%f,%6.0f,%6.0f,"), sets.rollFactor * 10, sets.servoReverseMask, sets.servoPanCenter);
                serial_printf_3a(PSTR("%6.0f,%6.0f,%6.0f,"),  sets.panMinPulse, sets.panMaxPulse, sets.servoTiltCenter);
                serial_printf_3a(PSTR("%6.0f,%6.0f,%6.0f,"),  sets.tiltMinPulse, sets.tiltMaxPulse, sets.servoRollCenter);
                serial_printf_3a(PSTR("%6.0f,%6.0f,%6.0f,"),  sets.rollMinPulse, sets.rollMaxPulse, sets.htChannels[0]);
                serial_printf_3a(PSTR("%6.0f,%6.0f,%6.0f\n"), sets.htChannels[1], sets.htChannels[2], 0);
            }
            break;
        } // switch
    } // if($)


    // if "read_sensors" flag is set high, read sensors and update
    if (read_sensors == 1 && ht_paused == 0) {

        uint32_t t=micros();

        GyroCalc();
        AccelCalc();
        MagCalc();

#ifdef HEARTBEAT_LED
        digitalWrite(HEARTBEAT_LED, HIGH);
#endif


        FilterSensorData(); // main calculations

        // Will first update read_sensors when everything is done.  
        read_sensors = 0;


        t = micros()-t;
        if(t>max_loop_time) max_loop_time=t;

        if(start_count){
            start_count--;
            if(start_count==0) resetValues=1; // auto-reset after start
            
        }

#ifdef HEARTBEAT_LED
        digitalWrite(HEARTBEAT_LED, LOW);
#endif

        // Only output this data every X frames.
        frameNumber++;

        extern uint8_t time_out;
        extern uint32_t isr_time;
        if(time_out) {
            serial_printl_1(PSTR("\n#loop timeout! loop=%ld\n"), max_loop_time);
//            serial_printl_1(PSTR(" isr time=%ld\n"), isr_time);
            max_loop_time=0;
            time_out=0;
        } else {        
            //if(Serial.tx_empty()) { // at full speed - but GUI can't consume such stream :(
            if (frameNumber >= SERIAL_OUTPUT_FRAME_INTERVAL && (outputTrack || outputMagAcc)) {    
                frameNumber=0;
                if (outputTrack)       trackerOutput();
                if (outputMagAcc)      calMagAccOutput();
            }
        }    

#if DEBUG
        if (frameNumber >= SERIAL_DEBUG_FRAME_INTERVAL) {
            do_output=true;

//            outputDbg=0; // only once
            frameNumber = 0; 
        }
#endif
        
    }
}

//--------------------------------------------------------------------------------------
// Func: SaveSettings
// Desc: Saves device settings to EEPROM for retrieval at boot-up.
//--------------------------------------------------------------------------------------

inline void SaveSettings()
{  
    WriteSets();
    serial_print_P(PSTR("Settings saved!\n"));
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
    serial_print_P(PSTR("\n\n\n------ Debug info------\n"));


    Serial.printf_P(PSTR("FW Version: %S\n"),FIRMWARE_VERSION);

    serial_printl_1(PSTR("loop time: %ld\n"), max_loop_time);
    max_loop_time=0;

    serial_printi_1(PSTR("Mag type: %i\n"), compass_id);

    serial_printf_3(PSTR("Gyro offset stored %f,%f,%f\n"), sets.gyroOff[0], sets.gyroOff[1], sets.gyroOff[2]);
 
    serial_printf_3(PSTR("Mag offset stored %f,%f,%f\n"),  sets.magOffset[0], sets.magOffset[1], sets.magOffset[2]);
    serial_printf_3(PSTR("Mag gain stored %f,%f,%f\n"),  sets.magGain[0], sets.magGain[1], sets.magGain[2]);
    serial_printf_3(PSTR("Mag diag stored %f,%f,%f\n"),  sets.magDiagOff[0], sets.magDiagOff[1], sets.magDiagOff[2]);
 
    serial_printf_3(PSTR("Acc offset stored %f,%f,%f\n"),  sets.accOffset[0], sets.accOffset[1], sets.accOffset[2]);
    serial_printf_3(PSTR("Acc gain stored %f,%f,%f\n"),  sets.accGain[0], sets.accGain[1], sets.accGain[2]);
    serial_printf_3(PSTR("Acc diag stored %f,%f,%f\n"),  sets.accDiagOff[0], sets.accDiagOff[1], sets.accDiagOff[2]);

#if PPM_IN
    serial_printi_1(PSTR("Input channels detected: %d\n"),  channelsDetected);
#endif
    SensorInfoPrint();
}




void ReadSets(void){
    uint16_t  i;
    for(i=0; i<sizeof(settings); i++)
	((byte *)&sets)[i] = eeprom_read_byte((byte *) i ); // EEPROM.read(EEPROM_offs(sets) + i );
}


void WriteSets(void){
    ht_paused=1;

    uint16_t i;
    for(i=0; i<sizeof(settings); i++) {
	eeprom_write_byte( (byte *)i, ((byte *)&sets)[i] ); // .write(EEPROM_offs(sets) + i,...
    }
    ht_paused=0;
}

void WriteSets(uint16_t addr, uint8_t length){
    ht_paused=1;

    uint8_t  i;

    for(i=0; (addr+i)<sizeof(settings) && i<length; i++) {
	eeprom_write_byte( (byte *)(addr+i), ((byte *)&sets)[addr+i] ); // .write(EEPROM_offs(sets) + i,...
    }
    ht_paused=0;
}
