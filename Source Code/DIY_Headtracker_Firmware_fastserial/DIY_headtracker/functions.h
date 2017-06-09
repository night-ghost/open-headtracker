//-----------------------------------------------------------------------------
// File: Functions.h
// Desc: Declares PPM-related functions for the project.
//-----------------------------------------------------------------------------
#ifndef functions_h
#define functions_h

#define NOINLINE __attribute__ ((noinline))

void InitTimerInterrupt();
void InitPWMInterrupt();
void DetectPPM();
void PrintPPM();
void DetectPPM();
void testPPM_in();

void NOINLINE filter( float &dst, const float val, const byte k);
void NOINLINE filter( float &dst, const float val, const float &k);
void NOINLINE filterAB(float &dst, const float &a, const float b, const float &k);

void serial_write_S(char c);
void serial_print(int i);
void serial_print(float i);
void serial_print(uint16_t i);
void serial_print_P(PGM_P f);
void serial_printi_1(PGM_P f, uint16_t i);
void serial_printl_1(PGM_P f, uint32_t i);
void serial_printf_1(PGM_P f, float l);

void serial_printf_3(PGM_P f, float &f1, float &f2, float &f3);
void serial_printf_3a(PGM_P f, float f1, float f2, float f3);
void serial_printi_3(PGM_P f, uint16_t f1, uint16_t f2, uint16_t f3);

void serial_println();

float NOINLINE float_div(float &f, uint8_t div);
float NOINLINE float_div(float &f, float &div);

extern int channelsDetected;
#endif

