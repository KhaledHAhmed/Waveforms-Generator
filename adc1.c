// ADC0 Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// ADC1 SS2

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "adc1.h"

#define ADC_CTL_DITHER          0x00000040

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initAdc1Ss2()
{
    // Enable clocks
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R1;
    _delay_cycles(16);

    // Configure ADC
    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN2;                // disable sample sequencer 3 (SS3) for programming
    ADC1_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC1_PC_R = ADC_PC_SR_1M;                        // select 1Msps rate
    ADC1_EMUX_R = ADC_EMUX_EM2_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
    ADC1_SSCTL2_R = ADC_SSCTL2_END0;                 // mark first sample as the end
    ADC1_ACTSS_R |= ADC_ACTSS_ASEN2;                 // enable SS3 for operation
}

// Set SS3 input sample average count
void setAdc1Ss2Log2AverageCount(uint8_t log2AverageCount)
{
    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN2;                // disable sample sequencer 3 (SS3) for programming
    ADC1_SAC_R = log2AverageCount;                   // sample HW averaging
    if (log2AverageCount == 0)
        ADC1_CTL_R &= ~ADC_CTL_DITHER;               // turn-off dithering if no averaging
    else
        ADC1_CTL_R |= ADC_CTL_DITHER;                // turn-on dithering if averaging
    ADC1_ACTSS_R |= ADC_ACTSS_ASEN2;                 // enable SS3 for operation
}

// Set SS2 analog input
void setAdc1Ss2Mux(uint8_t input)
{
    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN2;                // disable sample sequencer 3 (SS3) for programming
    ADC1_SSMUX2_R = input;                           // Set analog input for single sample
    ADC1_ACTSS_R |= ADC_ACTSS_ASEN2;                 // enable SS3 for operation
}

// Request and read one sample from SS2
int16_t readAdc1Ss2()
{
    ADC1_PSSI_R |= ADC_PSSI_SS2;                     // set start bit
    while (ADC1_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    while (ADC1_SSFSTAT2_R & ADC_SSFSTAT2_EMPTY);
    return ADC1_SSFIFO2_R;                           // get single result from the FIFO
}
