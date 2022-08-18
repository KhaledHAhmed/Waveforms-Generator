// ADC1  Library

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

#ifndef ADC1_H_
#define ADC1_H_

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initAdc1Ss2();
void setAdc1Ss2Log2AverageCount(uint8_t log2AverageCount);
void setAdc1Ss2Mux(uint8_t input);
int16_t readAdc1Ss2();

#endif
