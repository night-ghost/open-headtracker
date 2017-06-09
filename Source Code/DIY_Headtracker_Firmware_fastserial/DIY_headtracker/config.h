//-----------------------------------------------------------------------------
// File: Config.h
// Desc: Compilation configuration file. Defines user-configurable settings
//       as well as project-wide development #defines for configuration.
//-----------------------------------------------------------------------------
#ifndef config_h
#define config_h

//-----------------------------------------------------------------------------
// These are things you can change before you compile, to enable/disable
// features.
//

// Define for extra debug serial info
#define DEBUG 1

#define SAMPLERATE 100       // Samplerate of sensors (in hz, samples per second)

#define PPM_OUT_PIN 9// PPM out on ICP1 - pin9


#define PPM_IN  0    // Set to 1 to enable PPM input, 0 to disable. 
#define PPM_IN_PIN 2 // PPM in on ICP0 - pin2

//Connect A0 (analog_0) to pin2 (middle) of a 2K potentiometer, pin1 to +VCC, pin3 to GND.
// #define ADC_PIN A0


#define BUTTON_INPUT 11// PB4 (16) Center/pause input button pin number
#define ARDUINO_LED  13// LED pin
#define BUZZER       4 // PD4 (2) Pin definition for LED and buzzer (Fatshark goggles)

#define HEARTBEAT_LED 5 // PD5 (9)

// Button hold time for pause/unpause
#define BUTTON_HOLD_PAUSE_THRESH    1500    // 1.5 second button hold required to pause/unpause tracking.

// Set to 1 to enable support for positive shift PPM equipment, 0 for negative.
#define POSITIVE_SHIFT_PPM  1

// Fatshark headset headtracker module support. Set to 1 to enable.
// See: http://www.rcgroups.com/forums/showpost.php?p=23051198&postcount=573
#define FATSHARK_HT_MODULE 1

//
// -- End of user-configurable parameters.
//-----------------------------------------------------------------------------

//=============================================================================

//-----------------------------------------------------------------------------
// The user generally won't need to touch any of the following
//

// Firmware Version, e.g. X.YY
#define FIRMWARE_VERSION  PSTR("1.05")

// Number of PPM channels out. 1 - 12 channels supported. 
#define NUMBER_OF_CHANNELS (10u)


// Output serial data to host evern X frames
#define SERIAL_OUTPUT_FRAME_INTERVAL    20
#define SERIAL_DEBUG_FRAME_INTERVAL    50



// Serial communication speed. 
//#define SERIAL_BAUD 57600
#define SERIAL_BAUD 115200

// Sensor board update-rate. Not done yet. 
#define UPDATE_RATE 50

// Dead-time between each channel in the PPM-stream. 
#define DEAD_TIME 800

// Sets the frame-length 
#define FRAME_LENGTH (5003u + NUMBER_OF_CHANNELS * 5000u)

// TOP (timer rollover) used for PPM pulse time measurement
#define TOP (5003u + NUMBER_OF_CHANNELS * 5000u)

// Set to 0, stored gyro calibration is used. If 1, gyro is calibrated at powerup  
#define ALWAYS_CAL_GYRO 0


#define PPM_IN_MIN 900  // 0.5 ms
#define PPM_IN_MAX 4000 // 2 ms

// Settings stuff
//
#define HT_TILT_REVERSE_BIT     0x01
#define HT_ROLL_REVERSE_BIT     0x02
#define HT_PAN_REVERSE_BIT      0x04

#endif

