//Project CSE 4342 Embedded System II
// Khaled Ahmed

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL with LCD/Temperature Sensor
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz
// Stack:           4096 bytes (needed for sprintf)

// Hardware configuration:
//
//   AIN3/PE0
//   AIN2/PE1
//   LDAC/PD2
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdlib.h>
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


// Pin
#define LDAC PORTD,2
#define AIN2_INPUTA PORTE,1
#define AIN1_INPUTB PORTE,2
#define LUT_SIZE 2048
#define DAC_A_OFFSET  0
#define DAC_B_OFFSET  0
#define REF_FREQUENCY 40


//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
typedef enum _DAC
{
    DACA = 0,
    DACB = 1
} DAC;

typedef enum _DIFFERENTIAL
{
    OFF = 0,
    ON = 1
} DIFFERENTIAL;

typedef enum _LEVEL
{
    L_OFF = 0,
    L_ON = 1
} LEVEL;

uint16_t LUT_DATA_A [LUT_SIZE];
uint16_t LUT_DATA_B [LUT_SIZE];
uint16_t LUT_DATA_C [LUT_SIZE];
int N_cycles_A = 0;
int N_cycles_B = 0;
float countA = 0;
float countB = 0;
float Step_Size_A;
float Step_Size_B;
DAC DAC_SELECT_C;
DIFFERENTIAL differential = OFF;
LEVEL  level = L_OFF;
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------


void initHw();
void initTimer();
void sendData(uint16_t Data);
void sendDACsData(uint16_t DataA, uint16_t DataB);
void setDacVoltage (DAC DAC_SEL, float voltage);    // DAC output voltage
void setOpampVoltageOut (DAC DAC_SEL, float voltage); // OPAMP output voltage
void sinusoidalFunction (DAC DAC_SEL, float Frequency, float Amplitude, float offset, float Phase); // sine wave function
void squareFunction (DAC DAC_SEL, float Frequency, float Amplitude, float offset, float Phase);     // square wave function
void triangleFunction (DAC DAC_SEL, float Frequency, float Amplitude, float offset, float Phase);   // triangle wave function
void sawtoothFunction (DAC DAC_SEL, float Frequency, float Amplitude, float offset, float Phase);   // sawtooth wave function
uint16_t calcDACDataForOpampVoltage(DAC DAC_SEL, float voltage);
void timer1Isr();


// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Initialize SPI1 interface
    initSpi1(USE_SSI_FSS);
    setSpi1BaudRate(20e6, 40e6);
    setSpi1Mode(1, 1);


    //Enable clocks
    enablePort(PORTD);
    enablePort(PORTE);

    selectPinPushPullOutput(LDAC);
    enablePinPullup(LDAC);

    selectPinAnalogInput(AIN2_INPUTA);
    selectPinAnalogInput(AIN1_INPUTB);

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
    TIMER1_TAILR_R = 488;                            // set load value
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);
}

void timer1Isr()  // call lut function
{


    if(differential == ON)
    {
        if (N_cycles_A == -1)
        {

            sendData( LUT_DATA_A [(uint16_t)countA]);
            sendData( LUT_DATA_C [(uint16_t)countA]);
            countA = (countA + Step_Size_A);
            countA = fmod(countA, LUT_SIZE);
        }
        else if (N_cycles_A > 0)
        {
            sendData( LUT_DATA_A [(uint16_t)countA]);
            sendData( LUT_DATA_C [(uint16_t)countA]);
            countA = (countA + Step_Size_A);
            if (countA > LUT_SIZE)
            {
                // Finished 1 Period
                countA = fmod(countA, LUT_SIZE);
                N_cycles_A--;
            }
        }
    }
    else
    {
        if (N_cycles_A == -1)
        {

            sendData( LUT_DATA_A [(uint16_t)countA]);
            countA = (countA + Step_Size_A);
            countA = fmod(countA, LUT_SIZE);
        }
        else if (N_cycles_A > 0)
        {
            sendData( LUT_DATA_A [(uint16_t)countA]);
            countA = (countA + Step_Size_A);
            if (countA > LUT_SIZE)
            {
                // Finished 1 Period
                countA = fmod(countA, LUT_SIZE);
                N_cycles_A--;
            }
        }

        if (N_cycles_B == -1)
        {

            sendData( LUT_DATA_B [(uint16_t)countB]);
            countB = (countB + Step_Size_B);
            countB = fmod(countB, LUT_SIZE);
        }
        else if (N_cycles_B > 0)
        {
            sendData( LUT_DATA_B [(uint16_t)countB]);
            countB = (countB + Step_Size_B);
            if (countB > LUT_SIZE)
            {
                // Finished 1 Period
                countB = fmod(countB, LUT_SIZE);
                N_cycles_B--;
            }
        }
    }

    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
}

void sinusoidalFunction (DAC DAC_SEL, float Frequency, float Amplitude, float offset, float Phase)
{
    uint16_t i;
    float sinVoltage;

    for (i = 0; i < LUT_SIZE; i++)
    {
        sinVoltage = Amplitude * sin((( (float)i * 2.0 * M_PI )/ LUT_SIZE) + (Phase * M_PI)) + offset;//+  Phase ) + offset;

        if (DAC_SEL == DACA)
        {
            Step_Size_A = Frequency/REF_FREQUENCY;
            LUT_DATA_A [i] = calcDACDataForOpampVoltage(DACA, sinVoltage);
            LUT_DATA_C [i] = calcDACDataForOpampVoltage(DACB, -sinVoltage);
        }
        else if (DAC_SEL == DACB)
        {
            Step_Size_B = Frequency/REF_FREQUENCY;
            LUT_DATA_B [i] =   calcDACDataForOpampVoltage(DACB, sinVoltage);
        }
    }
}

void squareFunction (DAC DAC_SEL, float Frequency, float Amplitude, float offset, float Phase)
{
    uint16_t i;
    float sinVoltage;

    for (i = 0; i < LUT_SIZE; i++)
    {
        sinVoltage = Amplitude * sin((( (float)i * 2.0 * M_PI )/ LUT_SIZE) + (Phase * M_PI)) + offset;//+  Phase ) + offset;

        if (sinVoltage <= 0)
        {
            sinVoltage = -1 * Amplitude + offset;
        }
        else
        {
            sinVoltage = 1 * Amplitude + offset;
        }

        if (DAC_SEL == DACA)
        {
            Step_Size_A = Frequency/REF_FREQUENCY;
            LUT_DATA_A [i] = calcDACDataForOpampVoltage(DACA, sinVoltage);
            LUT_DATA_C [i] = calcDACDataForOpampVoltage(DACB, -sinVoltage);

        }
        else if (DAC_SEL == DACB)
        {
            Step_Size_B = Frequency/REF_FREQUENCY;
            LUT_DATA_B [i] =  calcDACDataForOpampVoltage(DACB, sinVoltage);
        }
    }
}

void triangleFunction (DAC DAC_SEL, float Frequency, float Amplitude, float offset, float Phase)
{
    uint16_t i;
    float sinVoltage;

    for (i = 0; i < LUT_SIZE; i++)
    {
        sinVoltage = (2/M_PI) * Amplitude * asin (sin((((float)i * 2.0 * M_PI )/ LUT_SIZE) + (Phase * M_PI))) + offset;//+  Phase ) + offset;

        if (DAC_SEL == DACA)
        {
            Step_Size_A = Frequency/REF_FREQUENCY;
            LUT_DATA_A [i] = calcDACDataForOpampVoltage(DACA, sinVoltage);
            LUT_DATA_C [i] = calcDACDataForOpampVoltage(DACB, -sinVoltage);

        }
        else if (DAC_SEL == DACB)
        {
            Step_Size_B = Frequency/REF_FREQUENCY;
            LUT_DATA_B [i] =  calcDACDataForOpampVoltage(DACB, sinVoltage);
        }
    }
}

void sawtoothFunction (DAC DAC_SEL, float Frequency, float Amplitude, float offset, float Phase)
{
    uint16_t i;
    float sinVoltage;

    for (i = 0; i < LUT_SIZE; i++)
    {
        sinVoltage = (2/M_PI) * Amplitude * atan (tan((((float)i * M_PI )/ LUT_SIZE) + (Phase * M_PI))) + offset;//+  Phase ) + offset;

        if (DAC_SEL == DACA)
        {
            Step_Size_A = Frequency/REF_FREQUENCY;
            LUT_DATA_A [i] = calcDACDataForOpampVoltage(DACA, sinVoltage);
            LUT_DATA_C [i] = calcDACDataForOpampVoltage(DACB, -sinVoltage);
        }
        else if (DAC_SEL == DACB)
        {
            Step_Size_B = Frequency/REF_FREQUENCY;
            LUT_DATA_B [i] =  calcDACDataForOpampVoltage(DACB, sinVoltage);
        }
    }
}

void sendData(uint16_t Data)
{
    _delay_cycles(2);       // 25ns * 2 = 50ns (min 40ns) Before CS Fall
    writeSpi1Data(Data);
    readSpi1Data();

    _delay_cycles(2);       // 25ns * 2 = 50ns (min 40ns)
    setPinValue(LDAC, 0);

    _delay_cycles(5);       // 25ns * 5 = 125ns (min 100ns)
    setPinValue(LDAC, 1);
}

void setDacVoltage (DAC DAC_SEL, float voltage)
{

    int16_t R = 0;
    uint16_t D = 0;
    uint16_t DAC = 0;
    char str[100];

    if (voltage < 0 )
    {
        voltage = 0;
    }
    if (voltage > 2.048 )
    {
        voltage = 2.048;
    }

    if (DAC_SEL == DACA)
    {
        //voltage = voltage + DAC_A_OFFSET;
        R = (2005.1 * voltage) - 7.7708;
        if (R < 0) R = 0;
        if (R > 4095) R = 4095;
        DAC = 0x3 << 12;
        D = (DAC) | (R & 0x0FFF);

    }
    else if (DAC_SEL == DACB)
    {
        //voltage = voltage + DAC_B_OFFSET;
        R = (2005.1 * voltage) - 7.7708;
        DAC = 0xB << 12;
        D = (DAC) | (R & 0x0FFF);

    }

    if (voltage < 0 )
    {
        voltage = 0;
    }
    if (voltage > 2.048 )
    {
        voltage = 2.048;
    }

    sprintf(str, "D:    %4u, R:    %4u, R:    %4f\n", D,R,voltage );
    putsUart0(str);

    sendData(D);
}

void setOpampVoltageOut (DAC DAC_SEL, float voltage)
{
    float DACvolts;
    char str[100];
    if (voltage < -5 )
    {
        voltage = -5;
    }
    if (voltage > 5 )
    {
        voltage = 5;
    }

    DACvolts = (voltage - 5) / -5;
    sprintf(str, "DACvolts:    %4f, voltage:    %4f\n", DACvolts,voltage );
    putsUart0(str);

    setDacVoltage (DAC_SEL, DACvolts);
}


uint16_t calcDACDataForOpampVoltage(DAC DAC_SEL, float voltage)
{

    float DACvolts;
    int16_t R = 0;
    uint16_t D = 0;
    uint16_t DAC = 0;

    if (voltage < -5 )
    {
        voltage = -5;
    }
    if (voltage > 5 )
    {
        voltage = 5;
    }

    DACvolts = ((-0.00009 * pow(voltage,3)) + (0.0002 * pow(voltage,2)) - (0.1888 * voltage) +  1.0172); // most recent

    if (DACvolts < 0 )
    {
        DACvolts = 0;
    }
    if (DACvolts > 2.048 )
    {
        DACvolts = 2.048;
    }

    if (DAC_SEL == DACA)
    {
        R = (2005.1 * DACvolts) - 7.7708;
        if (R < 0) R = 0;
        if (R > 4095) R = 4095;
        DAC = 0x3 << 12;
        D = (DAC) | (R & 0x0FFF);

    }
    else if (DAC_SEL == DACB)
    {
        voltage = voltage + DAC_B_OFFSET;
        DAC = 0xB << 12;
        R = (DACvolts * 4096) / 2.048;
        D = (DAC) | (R & 0x0FFF);
    }

    return D;

}


//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{

    char strInput[MAX_CHARS+1];
    char str[100];
    char *DAC_str;
    char *token;
    int cycles_A;
    int cycles_B;
    float Frequency;
    float offset;
    float Amplitude;
    float Phase;
    float DcVoltage;
    DAC DAC_SELECT;
    DAC DACSELECT;
    bool valid;
    bool ok;
    bool DC;

    // Initialize hardware
    initHw();
    initTimer();
    initUart0();
    initAdc0Ss3();
    initAdc1Ss2();

    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);

    // Use AIN2 input with N=4 hardware sampling
    setAdc0Ss3Mux(2);
    setAdc0Ss3Log2AverageCount(2);

    // Use AIN1 input with N=4 hardware sampling
    setAdc1Ss2Mux(1);
    setAdc1Ss2Log2AverageCount(2);

    putsUart0("Welcome to the Project\n");
    DC = false;
    while (true)
    {
        putsUart0("Please enter a command or write help to see all the commands \n");
        // getsUart0(& data);
        getsUart0(strInput, MAX_CHARS);
        // putsUart0(data.buffer);

        token = strtok(strInput, " \r\n");
        ok = token != NULL;
        valid = false;


        if (strcmp(token, "dc") == 0)
        {
            valid = true;
            DC = true;

            // DAC Select (DACA or 0 (outA) ////  DACB or 1 (outB ))
            token = strtok(NULL, " ,");
            ok = ok && token != NULL;

            if ((strcmp(token, "daca") == 0) || (strcmp(token, "DACA") == 0) || (strcmp(token, "0") == 0))
            {
                DACSELECT = DACA;
                DAC_str = "DAC A";
            }
            else if ((strcmp(token, "dacb") == 0) || (strcmp(token, "DACB") == 0) || (strcmp(token, "1") == 0))
            {
                DACSELECT = DACB;
                DAC_str = "DAC B";
            }
            else
            {
                DACSELECT = DACA;
                DAC_str = "DAC A as default setting";
            }

            // Dc Voltage
            token = strtok(NULL, " ,");
            ok = ok && token != NULL;
            DcVoltage = atof(token);

            if (ok)
            {
                sprintf(str, "Output voltage: %2f  in %s\n", DcVoltage, DAC_str );
                putsUart0(str);

                setOpampVoltageOut (DACSELECT, DcVoltage);
            }
            else
            {
                putsUart0("Error in write command arguments\n");
            }

        }
        else if (strcmp(token, "cycles") == 0)
        {
            valid = true;
            char *ncycle;

            token = strtok(NULL, " ");
            ok = ok && token != NULL;

            if ((strcmp(token, "daca") == 0) || (strcmp(token, "DACA") == 0) || (strcmp(token, "0") == 0))
            {
                DAC_SELECT_C = DACA;
                DAC_str = "DAC A";

                // Number of cycles
                token = strtok(NULL, " ,");
                ok = ok && token != NULL;

                if (ok)
                {
                    if(strcmp(token, "continuous") == 0 || strcmp(token, "c") == 0)
                    {
                        N_cycles_A = -1;
                        ncycle = token;
                    }
                    else
                    {
                        N_cycles_A = atof(token);
                        ncycle = token;
                    }

                }
            }
            else if ((strcmp(token, "dacb") == 0) || (strcmp(token, "DACB") == 0) || (strcmp(token, "1") == 0))
            {
                DAC_SELECT_C = DACB;
                DAC_str = "DAC B";

                // Number of cycles
                token = strtok(NULL, " ,");
                ok = ok && token != NULL;

                if (ok)
                {
                    if(strcmp(token, "continuous") == 0 || strcmp(token, "c") == 0)
                    {
                        N_cycles_B = -1;
                        ncycle = token;
                    }
                    else
                    {
                        N_cycles_B = atof(token);
                        ncycle = token;
                    }

                }
            }
            else // If DACA or DACB not entered or entered but incorrect
            {
                DAC_SELECT_C = DACA;
                DAC_str = "DAC A as default setting";
                DAC_SELECT_C = DACA;
                N_cycles_A = -1;
            }

            if (ok)
            {
                sprintf(str, "Wave with %s cyclesA %d  cyclesB %d  \n", ncycle, N_cycles_A, N_cycles_B);
                putsUart0(str);
                countA = 0;
                countB = 0;
                cycles_A = N_cycles_A;
                cycles_B = N_cycles_B;
            }
            else
            {
                putsUart0("Error in write command arguments\n");
            }

        }
        else if (strcmp(token, "sine") == 0)
        {
            valid = true;
            DC = false;
            // DAC Select (DACA or 0 (outA) ////  DACB or 1 (outB ))
            token = strtok(NULL, " ");
            ok = ok && token != NULL;

            if ((strcmp(token, "daca") == 0) || (strcmp(token, "DACA") == 0) || (strcmp(token, "0") == 0))
            {
                DAC_SELECT = DACA;
                DAC_str = "DAC A";
            }
            else if ((strcmp(token, "dacb") == 0) || (strcmp(token, "DACB") == 0) || (strcmp(token, "1") == 0))
            {
                DAC_SELECT = DACB;
                DAC_str = "DAC B";
            }
            else
            {
                DAC_SELECT = DACA;
                DAC_str = "DAC A as default setting";
            }
            // Frequency
            token = strtok(NULL, " ,");
            ok = ok && token != NULL;
            Frequency = atof(token);

            // Amplitude
            token = strtok(NULL, " ,");
            ok = ok && token != NULL;
            Amplitude = atof(token);

            // Offset
            token = strtok(NULL, " \r\n");
            // Determine if offset set else default offset = 0 v
            if (strlen(token) > 0)
            {
                offset = atof(token);
            }
            else
            {
                offset = 0;
            }

            // Phase
            token = strtok(NULL, " \r\n");
            // Determine if Phase set else default Phase = 0 v
            if (strlen(token) > 0)
            {
                Phase = atof(token);
            }
            else
            {
                Phase = 0;
            }


            if (ok)
            {
                putsUart0("Sine wave with: \n");
                sprintf(str,"- Frequency = %2f \n", Frequency);
                putsUart0(str);
                sprintf(str,"- Amplitude = %2f \n", Amplitude);
                putsUart0(str);
                sprintf(str,"- Offset = %2f \n", offset);
                putsUart0(str);
                sprintf(str,"- Phase = %2f \n", Phase);
                putsUart0(str);

                // call sine function
                // do some stuff here
                sinusoidalFunction (DAC_SELECT, Frequency, Amplitude, offset, Phase);
                countA = 0;
                countB = 0;
            }
            else
            {
                putsUart0("Error in write command arguments\n");
            }

        }
        else if (strcmp(token, "square") == 0)
        {
            valid = true;
            DC = false;
            // DAC Select (DACA or 0 (outA) ////  DACB or 1 (outB ))
            token = strtok(NULL, " ");
            ok = ok && token != NULL;

            if ((strcmp(token, "daca") == 0) || (strcmp(token, "DACA") == 0) || (strcmp(token, "0") == 0))
            {
                DAC_SELECT = DACA;
                DAC_str = "DAC A";
            }
            else if ((strcmp(token, "dacb") == 0) || (strcmp(token, "DACB") == 0) || (strcmp(token, "1") == 0))
            {
                DAC_SELECT = DACB;
                DAC_str = "DAC B";
            }
            else
            {
                DAC_SELECT = DACA;
                DAC_str = "DAC A as default setting";
            }

            // Frequency
            token = strtok(NULL, " ");
            ok = ok && token != NULL;
            Frequency = atof(token);

            // Amplitude
            token = strtok(NULL, " ");
            ok = ok && token != NULL;
            Amplitude = atof(token);

            // Offset
            token = strtok(NULL, " \r\n");
            // Determine if offset set else default offset = 0 v
            if (strlen(token) > 0)
            {
                offset = atof(token);
            }
            else
            {
                offset = 0;
            }

            // Phase
            token = strtok(NULL, " \r\n");
            // Determine if Phase set else default Phase = 0 v
            if (strlen(token) > 0)
            {
                Phase = atof(token);
            }
            else
            {
                Phase = 0;
            }

            if (ok)
            {
                putsUart0("“square wave with: \n");
                sprintf(str,"- Frequency = %2f \n", Frequency);
                putsUart0(str);
                sprintf(str,"- Amplitude = %2f \n", Amplitude);
                putsUart0(str);
                sprintf(str,"- Offset = %2f \n", offset);
                putsUart0(str);
                sprintf(str,"- Phase = %2f \n", Phase);
                putsUart0(str);


                // call square function
                squareFunction (DAC_SELECT, Frequency, Amplitude, offset, Phase);
                countA = 0;
                countB = 0;
            }
            else
            {
                putsUart0("Error in write command arguments\n");
            }

        }
        else if (strcmp(token, "triangle") == 0)
        {
            valid = true;
            DC = false;
            // DAC Select (DACA or 0 (outA) ////  DACB or 1 (outB ))
            token = strtok(NULL, " ");
            ok = ok && token != NULL;

            if ((strcmp(token, "daca") == 0) || (strcmp(token, "DACA") == 0) || (strcmp(token, "0") == 0))
            {
                DAC_SELECT = DACA;
                DAC_str = "DAC A";
            }
            else if ((strcmp(token, "dacb") == 0) || (strcmp(token, "DACB") == 0) || (strcmp(token, "1") == 0))
            {
                DAC_SELECT = DACB;
                DAC_str = "DAC B";
            }
            else
            {
                DAC_SELECT = DACA;
                DAC_str = "DAC A as default setting";
            }

            // Frequency
            token = strtok(NULL, " ");
            ok = ok && token != NULL;
            Frequency = atof(token);

            // Amplitude
            token = strtok(NULL, " ");
            ok = ok && token != NULL;
            Amplitude = atof(token);

            // Offset
            token = strtok(NULL, " \r\n");
            // Determine if offset set else default offset = 0 v
            if (strlen(token) > 0)
            {
                offset = atof(token);
            }
            else
            {
                offset = 0;
            }

            // Phase
            token = strtok(NULL, " \r\n");
            // Determine if Phase set else default Phase = 0 v
            if (strlen(token) > 0)
            {
                Phase = atof(token);
            }
            else
            {
                Phase = 0;
            }

            if (ok)
            {
                putsUart0("triangle wave with: \n");
                sprintf(str,"- Frequency = %2f \n", Frequency);
                putsUart0(str);
                sprintf(str,"- Amplitude = %2f \n", Amplitude);
                putsUart0(str);
                sprintf(str,"- Offset = %2f \n", offset);
                putsUart0(str);
                sprintf(str,"- Phase = %2f \n", Phase);
                putsUart0(str);


                // call square function
                triangleFunction (DAC_SELECT, Frequency, Amplitude, offset, Phase);
                countA = 0;
                countB = 0;
            }
            else
            {
                putsUart0("Error in write command arguments\n");
            }

        }
        else if (strcmp(token, "sawtooth") == 0)
        {
            valid = true;
            DC = false;
            // DAC Select (DACA or 0 (outA) ////  DACB or 1 (outB ))
            token = strtok(NULL, " ");
            ok = ok && token != NULL;

            if ((strcmp(token, "daca") == 0) || (strcmp(token, "DACA") == 0) || (strcmp(token, "0") == 0))
            {
                DAC_SELECT = DACA;
                DAC_str = "DAC A";
            }
            else if ((strcmp(token, "dacb") == 0) || (strcmp(token, "DACB") == 0) || (strcmp(token, "1") == 0))
            {
                DAC_SELECT = DACB;
                DAC_str = "DAC B";
            }
            else
            {
                DAC_SELECT = DACA;
                DAC_str = "DAC A as default setting";
            }

            // Frequency
            token = strtok(NULL, " ");
            ok = ok && token != NULL;
            Frequency = atof(token);

            // Amplitude
            token = strtok(NULL, " ");
            ok = ok && token != NULL;
            Amplitude = atof(token);

            // Offset
            token = strtok(NULL, " \r\n");
            // Determine if offset set else default offset = 0 v
            if (strlen(token) > 0)
            {
                offset = atof(token);
            }
            else
            {
                offset = 0;
            }

            // Phase
            token = strtok(NULL, " \r\n");
            // Determine if Phase set else default Phase = 0 v
            if (strlen(token) > 0)
            {
                Phase = atof(token);
            }
            else
            {
                Phase = 0;
            }

            if (ok)
            {
                putsUart0("sawtooth wave with: \n");
                sprintf(str,"- Frequency = %2f \n", Frequency);
                putsUart0(str);
                sprintf(str,"- Amplitude = %2f \n", Amplitude);
                putsUart0(str);
                sprintf(str,"- Offset = %2f \n", offset);
                putsUart0(str);
                sprintf(str,"- Phase = %2f \n", Phase);
                putsUart0(str);


                // call sawtooth function
                sawtoothFunction (DAC_SELECT, Frequency, Amplitude, offset, Phase);
                countA = 0;
                countB = 0;
            }
            else
            {
                putsUart0("Error in write command arguments\n");
            }

        }
        else if (strcmp(token, "stop") == 0)
        {
            valid = true;

            N_cycles_A = cycles_A;
            N_cycles_B = cycles_B;

            TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer
        }
        else if (strcmp(token, "run") == 0)
        {
            valid = true;

            TIMER1_CTL_R |= TIMER_CTL_TAEN;                 // turn-on timer
        }
        else if (strcmp(token, "pause") == 0)
        {
            valid = true;
            TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer
        }
        else if (strcmp(token, "differential") == 0 || strcmp(token, "d") == 0) //////take d out
        {
            valid = true;
            char str [20];
            char *onoff;
            token = strtok(NULL, " ");
            ok = ok && token != NULL;
            if ((strcmp(token, "ON") == 0) || (strcmp(token, "on") == 0))
            {
                differential = ON;
                onoff = "ON";
            }
            else  if ((strcmp(token, "OFF") == 0) || (strcmp(token, "off") == 0))
            {
                differential = OFF;
                onoff = "OFF";
            }

            if (ok)
            {
                sprintf(str,"Differential Mode %s \n", onoff);
                putsUart0(str);
                countA = 0;
                countB = 0;
            }
            else
            {
                putsUart0("Error in write command arguments\n");
            }
        }
        else if (strcmp(token, "reset") == 0)
        {
            valid = true;
            //reset the Microcontroller (Red board)
            NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
        }
        else if (strcmp(token, "voltage") == 0)
        {
            valid = true;

            uint16_t rawA;
            uint16_t rawB;
            float Vin;

            token = strtok(NULL, " ");
            ok = ok && token != NULL;

            if ((strcmp(token, "IN1") == 0) || (strcmp(token, "in1") == 0))
            {
                rawA = readAdc0Ss3();
                Vin = ((float) rawA * 5.0) / 4096.0;
                sprintf(str,"Voltage IN1  %2.2f \n", Vin);
                putsUart0(str);
            }
            else  if ((strcmp(token, "IN2") == 0) || (strcmp(token, "in2") == 0))
            {
                rawB = readAdc1Ss2();
                Vin = ((float) rawB * 5.0) / 4096.0;
                sprintf(str,"Voltage IN2  %2.2f \n", Vin);
                putsUart0(str);
            }
            else
            {
                putsUart0("Error in write command arguments\n");
            }

        }
        else if (strcmp(token, "gain") == 0)
        {
            valid = true;
            uint16_t rawA;
            uint16_t rawB;
            int n;
            float VinA;
            float VinB;
            float GaindB;
            float FREQ1;
            float FREQ2;
            float Freq = 0;


            // Frequency 1
            token = strtok(NULL, " ");
            ok = ok && token != NULL;
            FREQ1 = atof(token);

            // Frequency 2
            token = strtok(NULL, " ");
            ok = ok && token != NULL;
            FREQ2 = atof(token);

            if (ok)
            {
                sinusoidalFunction (DACA, FREQ1, 4,  0, 0); // sine wave function
                N_cycles_A = -1;

                putsUart0("---------------------------\n");
                putsUart0("| Frequency  |  Gain (dB) |\n");
                putsUart0("---------------------------\n");

                if (FREQ1 == 0)
                   n =0;
                else
                   n = log10(FREQ1);

                Freq = FREQ1;

                while (n <= 100)
                {
                    Step_Size_A = Freq/REF_FREQUENCY;
                    _delay_cycles(50);


                    rawA = readAdc0Ss3();
                    VinA = (rawA * 3.3) / 4096;

                    rawB = readAdc1Ss2();
                    VinB = (rawB * 3.3) / 4096;

                    GaindB = 20 *  log10(VinB/VinA);

                    sprintf(str,"|%7.2f     |   %3.2f  |\n", Freq, GaindB); //, n, steps );
                    putsUart0(str);

                    Freq = (1000 * n) / 10;
                    n++;
                    fflush(stdout);
                }
                sinusoidalFunction (DACA, 0, 0,  0, 0); // sine wave function
            }
            else
            {
                putsUart0("Error in write command arguments\n");
            }

        }
        else if (strcmp(token, "level") == 0)
        {
            valid = true;

            if (DC && (DACSELECT == DACA))
            {
                char str [20];
                char *onoff;
                token = strtok(NULL, " ");
                ok = ok && token != NULL;
                if ((strcmp(token, "ON") == 0) || (strcmp(token, "on") == 0))
                {
                    level = L_ON;
                    onoff = "ON";
                }
                else  if ((strcmp(token, "OFF") == 0) || (strcmp(token, "off") == 0))
                {
                    level = L_OFF;
                    onoff = "OFF";
                }

                if (ok )
                {
                    float adcVoltageIn;
                    uint16_t rawA = readAdc0Ss3();
                    adcVoltageIn = (rawA * 3.3) / 4096;

                    sprintf(str,"Level Mode %s \n", onoff);
                    putsUart0(str);

                    sprintf(str,"Voltage OUT 1 %2.2f \n", DcVoltage);
                    putsUart0(str);

                    sprintf(str,"IN 1 voltages  %2.2f \n", adcVoltageIn);
                    putsUart0(str);

                    sprintf(str," %2.2f voltages drop \n", DcVoltage - adcVoltageIn);
                    putsUart0(str);

                }
                else
                {
                    putsUart0("Error in write command arguments\n");
                }
            }
            else
            {
                putsUart0("Level command supported with DC signal and resistive load, on DAC A\n");
            }

        }
        else if (strcmp(token, "help") == 0)
        {
            valid = true;
            putsUart0("Commands:\n");
            putsUart0("    dc         OUT, Voltage  \n");
            putsUart0("    sine       OUT, FREQ, AMP, [OFS] [PH] \n");
            putsUart0("    square     OUT, FREQ, AMP, [OFS] [PH]  \n");
            putsUart0("    sawtooth   OUT, FREQ, AMP, [OFS] [PH] \n");
            putsUart0("    triangle   OUT, FREQ, AMP, [OFS] [PH] \n");
            putsUart0("    cycles     [N]    or [continuous] \n");
            putsUart0("    stop       stop wave form and start from time = 0\n");
            putsUart0("    run        run the last configured waveform or 0V\n");
            putsUart0("    pause      stop display the waveform \n");
            putsUart0("    differential    [ON] or [OFF] \n");
            putsUart0("    voltage    IN \n");
            putsUart0("    Level      [ON] or [OFF] \n");
            putsUart0("    gain       FREQ1, FREQ2 \n");


            putsUart0("    Extra detail in the above commands: \n");
            putsUart0("    OUT   DACA (0) or DACB (1) \n");
            putsUart0("    FREQ  Frequency (Hz)\n");
            putsUart0("    AMP   Amplitude (V)\n");
            putsUart0("    [OFS] Offset   [optional] (default OFS is 0V.) \n");
            putsUart0("    [PH]  Phase   [optional] (default PH is 0V.) \n");
            putsUart0("    [PH] = 0.25  =>> pi/4 \n");
            putsUart0("    [PH] = 0.5   =>> pi/2 \n");
            putsUart0("    [PH] = 1.0   =>> pi\n");
            putsUart0("    [PH] = 2.0   =>> 2*pi \n");
        }

        if (!valid)
            putsUart0("Invalid command\n");

        putsUart0(" \n");
    }

}
