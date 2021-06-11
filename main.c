

#include "msp.h"

#include "msp.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "nRF24_driver.h"           // eusci_b2


#include <driverlib.h>
#include "HAL_I2C.h"
#include "HAL_BQ27441.h"
#include <math.h>


void TA3_N_IRQHandler();
void setupTimerA3();
void ADC14_IRQHandler();
void ADC_Init();

const int SIZE = 5;
float sum = 0;
float ArrayBadc[SIZE] = {0}; //fill with 0's initially
float ArrayTadc[SIZE] = {0}; //fill with 0's initially
float ArrayRBadc[SIZE] = {0}; //fill with 0's initially
float ArrayRTadc[SIZE] = {0}; //fill with 0's initially

volatile int second_count;

int Index_ArrRB = 0;    //right bicep
int Index_ArrRT = 0;    //right tricep
int Index_ArrLT = 0;    //Left tricep
int Index_ArrLB = 0;    //left bicep


// P8.5 = A20
// P5.5 = A0
// P5.4 = A1
// P5.2 = A3
float P8_5_Voltage; //bicep
float P5_5_Voltage; //tricep
float P5_4_Voltage; //Rbicep
float P5_2_Voltage; //Rtricep

//void CS_init(void);
//void GPIO_init(void);
//char *itoa(int, char*, int);


int res = 0;


int Err[50];    //Array of possible errors

void main(void)
{

    char rf_data[13] = "12345678BBEE";    // data used by the rf

    rf_data[12] = 0;
    int i = 0;



    MAP_WDT_A_holdTimer(); //Halting the Watchdog

    //        GPIO_init();
    //        CS_init();


    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);    // red led initialization
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

    MAP_SysTick_setPeriod(6000000);
    MAP_SysTick_enableModule();


    nRF24_init();       // initiate rf transceiver

    // enable interrupts
    MAP_Interrupt_disableMaster();
    MAP_SysTick_enableInterrupt();
    //        MAP_Interrupt_enableMaster();

    //        __delay_cycles(1000000);
    //
    //        /* Initialize I2C */
    //        I2C_initGPIO();
    //        I2C_init();

    //EMG sensor code
    setupTimerA3(); //call the timerA3 set up function to initialize
    ADC_Init();     //setup EMG sensor readings
    MAP_Interrupt_enableMaster();

    float averageLB = 0; //initialize it to be 0
    float averageLT = 0;
    float averageRB = 0; //initialize it to be 0
    float averageRT = 0;


    while(1)
    {
        /*********** BICEP DATA*******************************/
        if(Index_ArrLB)   //only index when read completes
        {
            //Bicep Indexing and average
            //                ArrayBadc[4]= ArrayBadc[3];
            //                ArrayBadc[3]= ArrayBadc[2];
            //                ArrayBadc[2]= ArrayBadc[1];
            //                ArrayBadc[1]= ArrayBadc[0];
            //                ArrayBadc[0]= P5_5_Voltage; //shift new value

            sum = 0;
            for(i = SIZE-1; i >= 1; i--)
            {
                ArrayBadc[i] = ArrayBadc[i-1];
                sum += ArrayBadc[i];
            }
            ArrayBadc[0] = P5_5_Voltage; //shift new value
            sum += ArrayBadc[0];

            averageLB = sum/(SIZE);//take the individual pieces add them up and divide by the size of

            Index_ArrLB = 0;

        }

        if(Index_ArrLT)
        {
            //Tricep Indexing and average
            //                ArrayTadc[4]= ArrayTadc[3];
            //                ArrayTadc[3]= ArrayTadc[2];
            //                ArrayTadc[2]= ArrayTadc[1];
            //                ArrayTadc[1]= ArrayTadc[0];
            //                ArrayTadc[0]= P8_5_Voltage; //shift new value

            sum = 0;
            for(i = SIZE-1; i >= 1; i--)
            {
                ArrayTadc[i] = ArrayTadc[i-1];
                sum += ArrayTadc[i];
            }
            ArrayTadc[0] = P8_5_Voltage; //shift new value
            sum += ArrayTadc[0];

            //                sum =  (ArrayTadc[4]+ArrayTadc[3]+ArrayTadc[2]+ArrayTadc[1]+ArrayTadc[0]);
            averageLT = sum/(SIZE);//take the indivudal pieces add them up and divide by the size of

            Index_ArrLT = 0;
        }

        if(Index_ArrRB)
        {

            //RBicep Indexing and average
            //                ArrayRBadc[4]= ArrayRBadc[3];
            //                ArrayRBadc[3]= ArrayRBadc[2];
            //                ArrayRBadc[2]= ArrayRBadc[1];
            //                ArrayRBadc[1]= ArrayRBadc[0];
            //                ArrayRBadc[0]= P5_4_Voltage; //shift new value

            sum = 0;
            for(i = SIZE-1; i >= 1; i--)
            {
                ArrayRBadc[i] = ArrayRBadc[i-1];
                sum += ArrayRBadc[i];
            }
            ArrayRBadc[0] = P5_4_Voltage; //shift new value
            sum += ArrayRBadc[0];

            //                sum =  (ArrayRBadc[4]+ArrayRBadc[3]+ArrayRBadc[2]+ArrayRBadc[1]+ArrayRBadc[0]);
            averageRB = sum/(SIZE);//take the indivudal pieces add them up and divide by the size of

            Index_ArrRB = 0;
        }

        if(Index_ArrRT)
        {
            //RTricep Indexing and average
            //                ArrayRTadc[4]= ArrayRTadc[3];
            //                ArrayRTadc[3]= ArrayRTadc[2];
            //                ArrayRTadc[2]= ArrayRTadc[1];
            //                ArrayRTadc[1]= ArrayRTadc[0];
            //                ArrayRTadc[0]= P5_2_Voltage; //shift new value

            sum = 0;
            for(i = SIZE-1; i >= 1; i--)
            {
                ArrayRTadc[i] = ArrayRTadc[i-1];
                sum += ArrayRTadc[i];
            }
            ArrayRTadc[0] = P5_2_Voltage; //shift new value
            sum += ArrayRTadc[0];
            //                sum =  (ArrayRTadc[4]+ArrayRTadc[3]+ArrayRTadc[2]+ArrayRTadc[1]+ArrayRTadc[0]);
            averageRT = sum/(SIZE);//take the indivudal pieces add them up and divide by the size of

            Index_ArrRT = 0;
        }

        //prepare and send rf data
        if(res == 1)
        {

            //                short result16 = 0;
            char str[3];


            //                /* Read State Of Charge */
            //                if(!BQ27441_read16(STATE_OF_CHARGE, &result16, 1000))
            //                    Err[0] = 1;
            //                else
            //                {
            //                    sprintf(str, "%2.2d", (unsigned short)result16);
            //                    Err[0] = 0;
            //                }
            //                rf_data[8] = str[0];
            //                rf_data[9] = str[1];

            // reset second_count

            //load elements as characters for rf_data
            sprintf(str, "%2.2f", averageLB);
            rf_data[0] = str[0];
            rf_data[1] = str[1];
            sprintf(str, "%2.2f", averageLT);
            rf_data[2] = str[0];
            rf_data[3] = str[1];
            sprintf(str, "%2.2f", averageRB);
            rf_data[4] = str[0];
            rf_data[5] = str[1];
            sprintf(str, "%2.2f", averageRT);
            rf_data[6] = str[0];
            rf_data[7] = str[1];

            res = 0;

            // red led on
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);

            // send and write the data
            nRF24_send(13, (char *)rf_data);

            // debugging statements
            printf(rf_data); printf("\n\r");


            // red led off
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);


        }
    }



}




void SysTick_Handler(void)
{
    // this function should be called every 10ms (100Hz) for the fatfs
    res = 1;
}
void PORT5_IRQHandler(void)
{
    uint32_t status;

    // get status and clear interrupt flag
    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P5);
    P5IFG = 0;

    // set rf_irq with interrupt value
    if (status & BIT0)
    {
        rf_irq |= RF24_IRQ_FLAGGED;
    }

}


///* Initializes Clock System */
//void CS_init()
//{
//    /* Set the core voltage level to VCORE1 */
//    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);
//
//    /* Set 2 flash wait states for Flash bank 0 and 1*/
//    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
//    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);
//
//    /* Initializes Clock System */
//    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);
//    MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
//    MAP_CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
//    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
//    MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
//}
//
//
///* Initializes GPIO */
//void GPIO_init()
//{
//    /* Terminate all GPIO pins to Output LOW to minimize power consumption */
//    MAP_GPIO_setAsOutputPin(GPIO_PORT_PA, PIN_ALL16);
//    MAP_GPIO_setAsOutputPin(GPIO_PORT_PB, PIN_ALL16);
//    MAP_GPIO_setAsOutputPin(GPIO_PORT_PC, PIN_ALL16);
//    MAP_GPIO_setAsOutputPin(GPIO_PORT_PD, PIN_ALL16);
//    MAP_GPIO_setAsOutputPin(GPIO_PORT_PE, PIN_ALL16);
//    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_PA, PIN_ALL16);
//    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_PB, PIN_ALL16);
//    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_PC, PIN_ALL16);
//    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_PD, PIN_ALL16);
//    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_PE, PIN_ALL16);
//}

void ADC14_IRQHandler()
/*
 * purpose set up the ADC14-IRQhandler to track the change in voltages from the EMG data readings

 */
{
    uint32_t flags = ADC14->IFGR0; //reads flags and clears
    if(flags&BIT(10)) // if interrupt flag
    {
        P8_5_Voltage = ADC14->MEM[10]/16383.0*3.3;
        Index_ArrLT = 1;
    }
    if(flags&BIT(11)) // if interrupt flag //BIT 11 is BICEP
    {
        P5_5_Voltage = ADC14->MEM[11]/16383.0*3.3;
        Index_ArrLB = 1;
    }
    if(flags&BIT(12)) // if interrupt flag //BIT 12 is Right TRICEP
    {
        P5_4_Voltage = ADC14->MEM[12]/16383.0*3.3;
        Index_ArrRB = 1;
    }

    if(flags&BIT(13)) // if interrupt flag //BIT 11 is Right BICEP
    {
        P5_2_Voltage = ADC14->MEM[13]/16383.0*3.3;
        Index_ArrRT = 1;
    }

}

void setupTimerA3()
{
    //Four TimerAs exist, using the first one
    // Set up a 1 second timer
    // bits 15-10 are reserved = 000000
    // bits 9-8 are for clock = 10 (SMCLK)
    // bits 7-6 are for divider of 1 = 00
    // bits 5-4 are for mode, up mode = 01
    // bit 3 is reserved = 0
    // bit 2 is clear count value TAR = 1
    // bit 1 is for interrupt enable = 1
    // bit 0 is for the interrupt flag = 0
    TIMER_A3->CTL = 0b0000001000010110;
    TIMER_A3->CCR[0] = 29999; // 10 ms to start

    TIMER_A3->CTL &= ~BIT0;  // Clear flag
    NVIC_EnableIRQ(TA3_N_IRQn);
}

void TA3_N_IRQHandler()
{
    ADC14->CTL0 |= BIT0; //Start the Analog Conversion
    TIMER_A3->CTL &= ~BIT0;
}

void ADC_Init()
{
    P8->SEL0 |=  BIT5; // -- // SEL == 00 -> GPIO
    P8->SEL1 |=  BIT5;//  --SEL == 01 -> TimerA
    P8->DIR  &= ~BIT5;//  --  SEL == 11 -> Analog Input

    P5->SEL0 |=  BIT5;
    P5->SEL1 |=  BIT5;
    P5->DIR  &= ~BIT5;

    P5->SEL0 |=  BIT4;//  -- can change in exam or quiz
    P5->SEL1 |=  BIT4;//  -- can change in exam or quiz
    P5->DIR  &= ~BIT4;//  -- can change in exam or quiz

    P5->SEL0 |=  BIT2;//  -- can change in exam or quiz
    P5->SEL1 |=  BIT2;//  -- can change in exam or quiz
    P5->DIR  &= ~BIT2;//  -- can change in exam or quiz

    ADC14->CTL0 = 0b11000101111000100111011110010000;

    ADC14->CTL1 = 0b010100000000000110000;

    ADC14->MCTL[10] = 20;  //read A20  -- can change in exam or quiz
    ADC14->MCTL[11] = 0;  //read A0  -- can change in exam or quiz
    ADC14->MCTL[12] = 1;  //read A1  -- can change in exam or quiz
    ADC14->MCTL[13] = 3;  //read A3  -- can change in exam or quiz
    ADC14->MCTL[30] = 0 | BIT7;  // End of Sequence.  Read 21 analog inputs to make it slow.

    //MEM[0]
    // 15-0   Contains results of read

    //IE0
    // Interrupt on MCTL[10] and MCTL[11] being completed
    ADC14->IER0 = BIT(10)|BIT(11)|BIT(12)|BIT(13);  //-- can change in exam or quiz
    //IE1
    // All zeros to not interrupt enable
    ADC14->IER1 = 0;

    ADC14->CTL0 |= BIT1; //Enable the Analog Conversion

    NVIC_EnableIRQ(ADC14_IRQn);
}
