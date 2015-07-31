//-----------------------------------------------------------------------------
// File: Functions.cpp
// Desc: Implementations of PPM-related functions for the project.
//-----------------------------------------------------------------------------

#include <FastSerial.h>

#include "config.h"
#include "Arduino.h"
#include "functions.h"
#include "sensors.h"
#include <Wire.h>


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

// Sensor_board,   x,y,z
int acc_raw[3];
int gyro_raw[3];
int mag_raw[3];

// список коммутации каналов
//unsigned char PpmIn_PpmOut[13] = {0,1,2,3,4,5,6,7,8,9,10,11,12};

long channel_value[13] = {1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500};

#if PPM_IN
int channelValues[20];  // for input PPM
#endif

unsigned char channel_number = 1;
char shift = 0;
char time_out = 0;

//--------------------------------------------------------------------------------------
// Func: PrintPPM
// Desc: Prints the channel value represented in the stream. Debugging assistant. 
//--------------------------------------------------------------------------------------
#if DEBUG
void PrintPPM(){
  for (char j = 1; j < 13; j++)  {
      Serial.print(channel_value[j]);
      Serial.write(',');
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
        (1 << OCIE1A) | // Interrupt on compare A
        (0 << OCIE1B) | // Disable interrupt on compare B    
        (0 << TOIE1);          

    // OCR1A is used to generate PPM signal and later reset counter (to control frame-length)
    OCR1A = DEAD_TIME;    


#if (DEBUG)    
    Serial.printf_P(PSTR("PWM interrupt initialized\n"));
#endif
}

//--------------------------------------------------------------------------------------
// Func: InitTimerInterrupt
// Desc: Initializes timer interrupts.
//--------------------------------------------------------------------------------------

#define PRESCALE0 1024
#define TIMER_CLOCK_FREQ0 16000000.0/PRESCALE0 // 16MHz

void InitTimerInterrupt()
{  
  
    TCCR0A = 
       (0 << WGM00) |
       (1 << WGM01) |
       (0 << COM0A1) |
       (0 << COM0A0) |
       (0 << COM0B1) |
       (0 << COM0B0);  
   
    // 61 hz update-rate:
    TCCR0B =
        (0 << FOC0A)| // 
        (0 << FOC0B)| // 
        (1 << CS00) | // Prescale 1024 
        (0 << CS01) | // Prescale 1024  
        (1 << CS02) | // Prescale 1024
        (0 << WGM02);
  
    TIMSK0 = 
        (0 << OCIE0B) |
        (1 << OCIE0A) |
        (1 << TOIE0);       

    int cnt= ((TIMER_CLOCK_FREQ0/SAMPLERATE)+0.5);
    if(cnt>255) cnt=255;

//    OCR0B = 64 * 2;.
//    OCR0A = 64 * 2;
    OCR0B = (byte)cnt;
    OCR0A = (byte)cnt;
    
//    OCR0B = 64 * 2; 
//    OCR0A = 64 * 2;

#if (DEBUG)    
    Serial.printf_P(PSTR("Timer interrupt initialized: %d\n"),cnt);
#endif
}


#define PRESCALE 8
#define TIMER_CLOCK_FREQ 16000000.0/PRESCALE // 16MHz
    //Установка Таймера2.
    //Конфигурирует 8-битный Таймер2 ATMega168 для выработки прерывания
    //с заданной частотой.
    //Возвращает начальное значение таймера, которое должно быть загружено в TCNT2
    //внутри вашей процедуры ISR.
    //Смотри пример использования ниже.
byte  SetupTimer2(float timeoutFrequency){
    unsigned char result; //Начальное значение таймера.
 
    //Подсчет начального значения таймера
    result=(int)((256.0-(TIMER_CLOCK_FREQ/timeoutFrequency))+0.5);
    //257 на самом деле должно быть 256, но я получил лучшие результаты с 257.
 
    //Установки Таймер2: Делитель частоты /8, режим 0
    //Частота = 16MHz/8 = 2Mhz или 0.5 мкс
    //Делитель /8 дает нам хороший рабочий диапазон
    //так что сейчас мы просто жестко запрограммируем это.
    
    TCCR2A = 0;
    TCCR2B = 0<<CS22 | 1<<CS21 | 0<<CS20;
 
    //Подключение прерывания по переполнению Timer2
    TIMSK2 = 1<<TOIE2;
 
    //загружает таймер для первого цикла
    TCNT2=256 - result + 1;
 
    return result;
}

//--------------------------------------------------------------------------------------
// Func: TIMER0_COMPA_vect
// Desc: Timer 0 compare A vector Sensor-interrupt. We query sensors on a timer, not
//          during every loop.
//--------------------------------------------------------------------------------------
ISR(TIMER0_COMPA_vect){
    // Reset counter - should be changed to CTC timer mode. 
    TCNT0 = 0;

    if (read_sensors == 1) {
        time_out++;
        if (time_out > 10) {
#if DEBUG
        Serial.println("Timing problem!!!"); 
#endif
            time_out = 0;  
            digitalWrite(ARDUINO_LED, HIGH);
        }
    }
    read_sensors = 1;
    buttonDownTime += 16; // every 16 milliseconds, at 61 hz.
}

#if DEBUG
//--------------------------------------------------------------------------------------
// Func: TIMER1_OVF_vect
// Desc: Timer 1 overflow vector - only here for debugging/testing, as it should always
//      be reset/cleared before overflow. 
//--------------------------------------------------------------------------------------
ISR(TIMER1_OVF_vect)
{
    Serial.printf_P(PSTR("Timer 1 OVF\n"));
}

#endif

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
            channelValues[channel++] = pulseTime;
            channel_value[sets.PpmIn_PpmOut[channel]] = pulseTime;
        }
    } else if (pulseTime > PPM_IN_MIN) {
        channel++;
    }
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

