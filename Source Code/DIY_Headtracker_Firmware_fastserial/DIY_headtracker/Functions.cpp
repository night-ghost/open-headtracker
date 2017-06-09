//-----------------------------------------------------------------------------
// File: Functions.cpp
// Desc: Implementations of PPM-related functions for the project.
//-----------------------------------------------------------------------------

//#include <FastSerial.h>
#include <SingleSerial.h>
#include "Arduino.h"

#include "config.h"
#include "functions.h"
#include "sensors.h"


// Local variables
unsigned int pulseTime = 0; 
unsigned int lastTime = 0; 
unsigned int timeRead; 
int channelsDetected = 0;
char channel = 0;

char state = 0; // PPM signal high or Low?
char read_sensors = 0;

// external variables
extern unsigned long buttonDownTime;

extern void serial_write_S(char c);

// Sensor_board,   x,y,z
int acc_raw[3];
int gyro_raw[3];
int mag_raw[3];

// список коммутации каналов

long channel_value[13] = {1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500};

#if PPM_IN
int channelValues[20];  // for input PPM
#endif

unsigned char channel_number = 1;
int8_t shift = 0;
uint8_t time_out = 0;

extern byte ht_paused;

//--------------------------------------------------------------------------------------
// Func: PrintPPM
// Desc: Prints the channel value represented in the stream. Debugging assistant. 
//--------------------------------------------------------------------------------------
#if DEBUG
void PrintPPM(){
  for (byte j = 1; j < 13; j++)  {
      Serial.print(channel_value[j]);
      serial_write_S(',');
  }
  Serial.println();
}
#endif

//--------------------------------------------------------------------------------------
// Func: InitPWMInterrupt
// Desc: 
//--------------------------------------------------------------------------------------
void InitPWMInterrupt(){
  
    TCCR1A = 
       (0 << WGM10) |
       (0 << WGM11) |
       (0 << COM1A1) |
       (1 << COM1A0) | // Toggle pin om compare-match
       (0 << COM1B1) |
       (0 << COM1B0);  
  
    TCCR1B =
        (1 << ICNC1)| // Input capture noise canceler - set to active 
        (1 << ICES1)| // Input capture edge select. 1 = rising, 0 = falling. We will toggle this, doesn't matter what it starts at        
        (0 << CS10) | // Prescale 8  
        (1 << CS11) | // Prescale 8  
        (0 << CS12) | // Prescale 8
        (0 << WGM13)|    
        (1 << WGM12); // CTC mode (Clear timer on compare match) with ICR1 as top.           
    
    // Not used in this case:
    TCCR1C =
        (0 << FOC1A)| // No force output compare (A)
        (0 << FOC1B); // No force output compare (B)
        
    
    TIMSK1 = 
        (PPM_IN << ICIE1) | // Enable input capture interrupt    
        (1 << OCIE1A) |     // Interrupt on compare A
        (0 << OCIE1B) |     // Disable interrupt on compare B    
        (0 << TOIE1);          

    // OCR1A is used to generate PPM signal and later reset counter (to control frame-length)
    OCR1A = DEAD_TIME;    


#if (DEBUG)    
    serial_print_P(PSTR("PWM interrupt initialized\n"));
#endif
}

//--------------------------------------------------------------------------------------
// Func: InitTimerInterrupt
// Desc: Initializes timer interrupts.
//--------------------------------------------------------------------------------------

#define PRESCALE0 1024
#define TIMER_CLOCK_FREQ0 16000000.0/PRESCALE0 // 16MHz -> 16KHz

void InitTimerInterrupt()
{  
    TCCR2A = 
       (0 << WGM20) |
       (1 << WGM21) |  // CTC mode - reset on OCR2A
       (0 << COM2A1) |
       (0 << COM2A0) |
       (0 << COM2B1) |
       (0 << COM2B0);  
   
    // 61 hz update-rate:
    TCCR2B =
        (0 << FOC2A)| // 
        (0 << FOC2B)| // 
        (1 << CS20) | // Prescale 1024 
        (1 << CS21) | // Prescale 1024  
        (1 << CS22) | // Prescale 1024
        (0 << WGM22);
  
    TIMSK2 = 
        (0 << OCIE2B) |
        (1 << OCIE2A) | // interrupt on compare
        (0 << TOIE2);   // no interrupt on overflow

    int cnt= ((TIMER_CLOCK_FREQ0/SAMPLERATE)+0.5);
    if(cnt>255) cnt=255;

    OCR2B = (byte)cnt;
    OCR2A = (byte)cnt;

#if (DEBUG)    
    serial_printi_1(PSTR("Timer interrupt initialized: %d\n"),cnt);
#endif
}



//--------------------------------------------------------------------------------------
// Func: TIMER2_COMPA_vect
// Desc: Timer 2 compare A vector Sensor-interrupt. We query sensors on a timer, not
//          during every loop.
//--------------------------------------------------------------------------------------
uint32_t last_isr_t=0;
uint32_t isr_time = 0;

ISR(TIMER2_COMPA_vect){
    // Reset counter - should be changed to CTC timer mode. 
//    TCNT2 = 0;

#if 0
    uint32_t t=millis();
    isr_time = t - last_isr_t;
    last_isr_t=t;
#endif

    if (!ht_paused && read_sensors == 1) {
        time_out++;
        digitalWrite(ARDUINO_LED, HIGH);
/* cause hangups
        if (time_out > 10) {
#if DEBUG
            serial_print_P(PSTR("Timing problem!!!\n")); 
#endif
            time_out = 0;  
        }
*/
    } else {
        digitalWrite(ARDUINO_LED, LOW);
    }
    read_sensors = 1;
    buttonDownTime += 16; // every 16 milliseconds, at 61 hz.
}


//--------------------------------------------------------------------------------------
// Func: TIMER1_COMPA_vect
// Desc: Timer 1 compare A vector
//--------------------------------------------------------------------------------------
ISR(TIMER1_COMPA_vect)
{

#if 1
  static boolean state = true;
  
  TCNT1 = 0;
  
  if(state) {  //start pulse
     TCCR1A = 
            (0 << WGM10) |
            (0 << WGM11) |
            (0 << COM1A1) |
            (POSITIVE_SHIFT_PPM << COM1A0) |
            (0 << COM1B1) |
            (0 << COM1B0); 
 
    
    OCR1A = DEAD_TIME;
    state = false;
  }
  else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb=0;
    static unsigned int calc_rest=0;
   
            TCCR1A = 
            (0 << WGM10) |
            (0 << WGM11) |
            (1 << COM1A1) |
            (1 << COM1A0) |
            (0 << COM1B1) |
            (0 << COM1B0);  
    
    state = true;

    if(cur_chan_numb >= NUMBER_OF_CHANNELS){
      cur_chan_numb = 1;
      calc_rest += DEAD_TIME;// 
      OCR1A = (FRAME_LENGTH - calc_rest);
      calc_rest = 0;
    } else{
      OCR1A = (channel_value[cur_chan_numb] - DEAD_TIME);
      calc_rest += channel_value[cur_chan_numb];
      cur_chan_numb++;
    }     
  }

 
#else
    if (OCR1A == FRAME_LENGTH) {
        TCCR1A = 
            (0 << WGM10) |
            (0 << WGM11) |
            (1 << COM1A1) |
            (1 << COM1A0) |
            (0 << COM1B1) |
            (0 << COM1B0);   
  
        channel_number = 1;
        OCR1A = DEAD_TIME;
  
        TCCR1B &= ~(1 << WGM12);
    }   else {
        if (channel_number == 1)  {
            // After first time, when pin have been set high, we toggle the pin at each interrupt
            TCCR1A = 
                (0 << WGM10) |
                (0 << WGM11) |
                (0 << COM1A1) |
                (POSITIVE_SHIFT_PPM << COM1A0) |
                (0 << COM1B1) |
                (0 << COM1B0);   
        }
                  
        if ((channel_number - 1) < NUMBER_OF_CHANNELS * 2) {
            if ((channel_number-1) % 2 == 1) {
                OCR1A += DEAD_TIME; 
            } else {
                OCR1A += channel_value[(channel_number + 1) / 2];
            }
            channel_number++;
        } else {
            // We have to use OCR1A as top too, as ICR1 is used for input capture and OCR1B can't be
            // used as top. 
            OCR1A = FRAME_LENGTH;
            TCCR1B |= (1 << WGM12);
        }
    }
#endif
}  


#if PPM_IN

//--------------------------------------------------------------------------------------
// Func: DetectPPM
// Desc: 
//--------------------------------------------------------------------------------------
void DetectPPM(){  
    // If a new frame is detected
    if (pulseTime > 5500) {
        // Save total channels detected
        channelsDetected = channel; 
     
        // Reset channel-count
        channel = 0; 
    }  else if (channel < 20 && pulseTime > PPM_IN_MIN && pulseTime < PPM_IN_MAX) {
        // If the pulse is recognized as servo-pulse
        if ( (channel + 1) != sets.htChannels[0] &&
             (channel + 1) != sets.htChannels[1] &&
             (channel + 1) != sets.htChannels[2] )  {
            channelValues[channel++] = pulseTime; // input data
            channel_value[sets.PpmIn_PpmOut[channel]] = pulseTime; // for output
        }
    } else if (pulseTime > PPM_IN_MIN) {
        channel++;
    }
}  

void testPPM_in(){
    serial_print_P(PSTR("Ch: "));
    for(byte i=0; i<channelsDetected;i++){
	serial_printi_1(PSTR("%d "), channelValues[i]);
    }
    serial_write_S(10);

}

//--------------------------------------------------------------------------------------
// Func: TIMER1_CAPT_vect
// Desc: The interrupt vector used when an edge is detected Interrupt vector, see page
//      57 Interrupt for input capture
//--------------------------------------------------------------------------------------
ISR(TIMER1_CAPT_vect) {
    // Disable interrupt first, to avoid multiple interrupts causing hanging/restart,
    // or just weird behavior:
    TIMSK1 &= ~(1 << ICIE1);
    
    state = TCCR1B & (1 << ICES1);
    
    // Toggle interrupt to detect falling/rising edge:
    TCCR1B ^= (1<<ICES1);
    
    // Read the time-value stored in ICR1 register (will be the time copied from TCNT1 at input-capture event). 
    timeRead = ICR1;    
    
    pulseTime = timeRead; 
    
    // Check if the timer have reached top/started over:
    if (lastTime > pulseTime){
        pulseTime += (TOP - lastTime); 
    } else {
        // Subtract last time to get the time:
        pulseTime -= lastTime;
    }
    
    // Save current timer-value to be used in next interrupt:
    lastTime = timeRead;
    
    // If we are detecting a PPM input signal
    DetectPPM(); 

    // Enable interrupt again:
    TIMSK1 |= (1 << ICIE1); 
}

#endif





void NOINLINE filter( float &dst, const float val, const byte k){ // комплиментарный фильтр 1/k
    if(dst==0 || k==0 || k==1) dst=val;
    else
        //dst = (val * k) + dst * (1.0 - k); 
        //dst = val * k + dst - dst*k;
        //dst = (val-dst)*k + dst;
        dst+=(val-dst)/k;
}

void NOINLINE filter( float &dst, const float val, const float &k){ // комплиментарный фильтр 1/k
    if(dst==0 || k==0 || k==1) dst=val;
    else
        //dst = (val * k) + dst * (1.0 - k); 
        //dst = val * k + dst - dst*k;
        //dst = (val-dst)*k + dst;
        dst+=(val-dst)/k;
}


//  x = (x+a)*k + b*(1-k)
//  x += a*k + (b-x)*(1-k),  k=1-z
//  x += a*(1-z) + (b-x)*(1-(1-z))
//  x += a*(1-z) + (b-x)*z
//  x += a - a*z + (b-x)*z
//  x += a + (b-x-a)*z

void NOINLINE filterAB(float &dst, const float &a, const float b, const float &z)
{
    if(dst==0) dst = b;
    else {
        dst += a + (b-dst-a)*z;
    }
}

void serial_write_S(char c){ Serial.write(c); }
void serial_print(int i) { Serial.print(i); }
void serial_print(float i) { Serial.print(i); }
void serial_print(uint16_t i) { Serial.print(i); }
void serial_print_P(PGM_P f) { Serial.printf_P(f); }
void serial_printi_1(PGM_P f, uint16_t i) { Serial.printf_P(f,i); }
void serial_printl_1(PGM_P f, uint32_t i) { Serial.printf_P(f,i); }
void serial_printf_1(PGM_P f, float l) { Serial.printf_P(f,l); }

void serial_printf_3(PGM_P f, float &f1, float &f2, float &f3) { Serial.printf_P(f,f1,f2,f3); };
void serial_printf_3a(PGM_P f, float f1, float f2, float f3) { Serial.printf_P(f,f1,f2,f3); };
void serial_printi_3(PGM_P f, uint16_t f1, uint16_t f2, uint16_t f3) { Serial.printf_P(f,f1,f2,f3); };

void serial_println() { serial_println(); }

float NOINLINE float_div(float &f, uint8_t div){   return f/div; }
float NOINLINE float_div(float &f, float &div){    return f/div; }

