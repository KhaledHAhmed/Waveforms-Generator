//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration: -

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#ifndef PROJECT_H_
#define PROJECT_H_

#include <stdint.h>
#include <stdint.h>

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
typedef struct signal
{
    char *signal_Name;
    uint8_t Amplitude;
    uint32_t Frequency;
    uint32_t Phase;
    uint8_t Offset;
    uint8_t DAC_SEL;

}Signal;

typedef enum _DAC
{
    DACA = 0,
    DACB = 1
} DAC;


void initHw();
void initTimer();
void sendData(uint16_t Data);
void setDacVoltage (DAC DAC_SEL, float voltage);    // DAC output voltage
void setOpampVoltageOut (DAC DAC_SEL, float voltage); // OPAMP output voltage
void sinusoidalFunction (DAC DAC_SEL, uint32_t Phase, uint8_t offset, uint8_t Amplitude); // sin function
//void timer1Isr();
#endif
