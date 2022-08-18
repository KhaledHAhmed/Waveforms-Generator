/*//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration: -

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "gpio.h"
#include "spi1.h"
#include "wait.h"
#include "nvic.h"
#include "uart0.h"
#include "adc0.h"
#include "adc1.h"
#include "project.h"


// Pin
#define LDAC PORTD,2
#define AIN2_INPUTA PORTE,1
#define AIN1_INPUTB PORTE,2
#define LUT_SIZE 4095
#define PI 22/7
#define DAC_A_OFFSET -0.0023
#define DAC_B_OFFSET -0.0023

// **** Debug variables ****
#define GREEN_LED PORTF,3

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------
uint16_t LUT_DATA_A [LUT_SIZE];
uint16_t LUT_DATA_B [LUT_SIZE];
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Initialize SPI1 interface
    initSpi1(USE_SSI_FSS);
    setSpi1BaudRate(1e6, 40e6);
    setSpi1Mode(1, 1);
    //setSpi1Mode(0, 0);

     //Enable clocks
     enablePort(PORTD);
     enablePort(PORTE);
     enablePort(PORTF);

     selectPinPushPullOutput(LDAC);
     enablePinPullup(LDAC);

    selectPinAnalogInput(AIN2_INPUTA);
    selectPinAnalogInput(AIN1_INPUTB);

   // ******************************
   selectPinPushPullOutput(GREEN_LED);

   // ******************************

}

void initTimer()
{
    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    _delay_cycles(3);

    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 40000000;//9766;   // 40000000;   // ?????? // set load value to 9766 (9765.625) for 4096 Hz interrupt rate     // set load value to 40e6 for 1 Hz interrupt rate ///////////
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);
}

void timer1Isr()  // call lut function //////////////////
{
    setPinValue(GREEN_LED, !getPinValue(GREEN_LED));
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
}

void sinusoidalFunction (DAC DAC_SEL, uint32_t Phase, uint8_t offset, uint8_t Amplitude)
{
    uint8_t i;
    float sinVoltage;
    for (i = 0; i < LUT_SIZE; i++)
    {
        sinVoltage = Amplitude * sin(((i * 2 * PI )/ LUT_SIZE)  +  Phase ) + offset;
        if (DAC_SEL == DACA)
        {
            LUT_DATA_A [i] = (uint16_t)((-sinVoltage * 1.024)/5) + 1.024;
        }
        else if (DAC_SEL == DACB)
        {
            LUT_DATA_B [i] =  (uint16_t)((-sinVoltage * 1.024)/5) + 1.024;
        }
    }

}

void sendData(uint16_t Data)
{
    _delay_cycles(2);       // 25ns * 2 = 50ns (min 40ns) Before CS Fall
    writeSpi1Data(Data);
    readSpi1Data();
   // waitMicrosecond(1);   // min 40ns (1us = 1000ns)
    _delay_cycles(2);       // 25ns * 2 = 50ns (min 40ns)
    setPinValue(LDAC, 0);
    //waitMicrosecond(1); // min 100ns (1us = 1000ns)
    _delay_cycles(5);       // 25ns * 5 = 125ns (min 100ns)
    setPinValue(LDAC, 1);
}

void setDacVoltage (DAC DAC_SEL, float voltage)
{

    uint16_t R = 0;
    uint16_t D = 0;
    uint16_t DAC = 0;

    if (DAC_SEL == DACA)
    {
        voltage = voltage + DAC_A_OFFSET;
        DAC = 0x3 << 12;
        //DAC = 0x3000;
    }
    else if (DAC_SEL == DACB)
    {
        voltage = voltage + DAC_B_OFFSET;
        DAC = 0xB << 12;
        //DAC = 0xB000;
    }

    if (voltage < 0 )
    {
        voltage = 0;
    }
    if (voltage > 2.048 )
    {
        voltage = 2.048;
    }


    R = (voltage * 4096) / 2.048;
    D = (DAC) | (R & 0x0FFF);

    sendData(D);
}

void setOpampVoltageOut (DAC DAC_SEL, float voltage)
{
    float DACvolts;
    if (voltage < -5 )
       {
           voltage = -5;
       }
       if (voltage > 5 )
       {
           voltage = 5;
       }

       DACvolts = ((-voltage * 1.024)/5) + 1.024;

       setDacVoltage (DAC_SEL, DACvolts);
}
*/
