

/*
 * Name:            Senior Project Team 11
 * File:            main.c
 * Description:     This code is to be used to control the harness used in the EMG Controlled Robotic Limb Project.
 *                  It performs i2c communication with the BOOSTXL-BATPAKMKII as well as SPI with an nrf24l01+ module.
 *                  This also performs ADC conversions and moving averaging with Myoware EMG Sensors.
 *
 */

#include "msp.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "nRF24_driver.h"

#include <driverlib.h>
#include "HAL_I2C.h"
#include "HAL_BQ27441.h"
#include <math.h>

void TA3_N_IRQHandler();
void setupTimerA3();
void ADC14_IRQHandler();
void ADC_Init();

const int SIZE = 8;                 //Controls period over which is averaged, size*10ms is the period
float sum = 0;                      //Initialize sum for moving average
float ArrayBadc[SIZE] = {0};         //fill with 0's initially
float ArrayTadc[SIZE] = {0};        //fill with 0's initially
float ArrayRBadc[SIZE] = {0};       //fill with 0's initially
float ArrayRTadc[SIZE] = {0};       //fill with 0's initially

volatile int second_count;          //counter used to take battery readings

//Used to index through moving average arrays
int Index_ArrRB = 0;
int Index_ArrRT = 0;
int Index_ArrLT = 0;
int Index_ArrLB = 0;


// P4.7 = A6
// P5.5 = A0
// P5.4 = A1
// P5.2 = A3
float P4_7_Voltage; //Sensor 2
float P5_5_Voltage; //Sensor 1
float P5_4_Voltage; //Sensor 3
float P5_2_Voltage; //Sensor 4

void GPIO_init(void);


int res = 0;                //used for sending RF data every 100ms.

//bits of possible errors
uint16_t Err = 0x0000;

void main(void)
{
        // data used by the rf
        char rf_data[13] = "12345670000";
        short result16 = 0;

        rf_data[12] = 0;

        int i = 0;
        int batteryread_delay = 0;


        /* Halting the Watchdog  */
        MAP_WDT_A_holdTimer();

        GPIO_init();

        // red led initialization
        MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);


        MAP_SysTick_setPeriod(300000);     //100ms second systick count, change this in order to send RF data faster
        MAP_SysTick_enableModule();

        __delay_cycles(1000000);            //delay to allow proper setup of RF transmitter
        // initiate rf transceiver
        nRF24_init();

        // enable interrupts
        MAP_Interrupt_disableMaster();
        MAP_SysTick_enableInterrupt();

        __delay_cycles(1000000);

        /* Initialize I2C */
        I2C_initGPIO();
        I2C_init();

        //EMG sensor code
        setupTimerA3(); //call the timerA3 set up function to initialize
        ADC_Init();     //setup EMG sensor readings

        float averageLB = 0; //initialize it to be 0
        float averageLT = 0;
        float averageRB = 0; //initialize it to be 0
        float averageRT = 0;

        //Boosterpack read setup, refer to examples in resource explorer for more in depth setup
        BQ27441_initConfig();
        BQ27441_initOpConfig();
        BQ27441_control(BAT_INSERT, 63);

        MAP_Interrupt_enableMaster();

        while(1)
        {
            /*********** BICEP DATA*******************************/
            if(Index_ArrLB)   //only index when read completes
            {
                //Bicep Indexing and average

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

                sum = 0;
                for(i = SIZE-1; i >= 1; i--)
                {
                    ArrayTadc[i] = ArrayTadc[i-1];
                    sum += ArrayTadc[i];
                }
                ArrayTadc[0] = P4_7_Voltage; //shift new value
                sum += ArrayTadc[0];

                averageLT = sum/(SIZE);//take the individual pieces add them up and divide by the size of

                Index_ArrLT = 0;
            }

            if(Index_ArrRB)
            {


                sum = 0;
                for(i = SIZE-1; i >= 1; i--)
                {
                    ArrayRBadc[i] = ArrayRBadc[i-1];
                    sum += ArrayRBadc[i];
                }
                ArrayRBadc[0] = P5_4_Voltage; //shift new value
                sum += ArrayRBadc[0];

                averageRB = sum/(SIZE);//take the individual pieces add them up and divide by the size of

                Index_ArrRB = 0;
            }

            if(Index_ArrRT)
            {

                sum = 0;
                for(i = SIZE-1; i >= 1; i--)
                {
                    ArrayRTadc[i] = ArrayRTadc[i-1];
                    sum += ArrayRTadc[i];
                }
                ArrayRTadc[0] = P5_2_Voltage; //shift new value
                sum += ArrayRTadc[0];

                averageRT = sum/(SIZE);//take the individual pieces add them up and divide by the size of

                Index_ArrRT = 0;
            }

            /***************prepare and send rf data*****************/
            if(res == 1)
            {
                batteryread_delay += 1;
                char str[3];

                //read battery status every half minute
                if(batteryread_delay >= 30)
                {
                    batteryread_delay = 0;

                    /* Read State Of Charge */
                    if(!BQ27441_read16(STATE_OF_CHARGE, &result16, 63))
                    {
                        Err |= 0b10000;                             //flag error if cannot read battery
                    }
                    else
                    {
                        Err &= ~0b10000;
                        sprintf(str, "%2.2d", (unsigned short)result16);            //print out the battery charge
                        rf_data[8] = str[0];
                        rf_data[9] = str[1];
                    }
                }


                //load elements as characters for rf_data
                sprintf(str, "%2.2f", averageLB/3.3*99);
                rf_data[0] = str[0];
                rf_data[1] = str[1];
                sprintf(str, "%2.2f", averageLT/3.3*99);
                rf_data[2] = str[0];
                rf_data[3] = str[1];
                sprintf(str, "%2.2f", averageRB/3.3*99);
                rf_data[4] = str[0];
                rf_data[5] = str[1];
                sprintf(str, "%2.2f", averageRT/3.3*99);
                rf_data[6] = str[0];
                rf_data[7] = str[1];

                rf_data[10] = (Err & 0xFF00)>>8;                //Send upper 8 bits for errors
                rf_data[11] = Err;                              //Send lower 8 bits for errors

                res = 0;

                // red led on
                MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);

                // send and write the data
                nRF24_send(13, (char *)rf_data);

                // debugging statements, uncomment to see RF data in the console
//                printf(rf_data); printf("\n\r");


                // red led off
                MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);


            }
        }



}



/*----------------------------------------------------------------
 * void SysTick_Handler(void)
 *
 * Description: Systick interrupt used to control when RF data is sent
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/

void SysTick_Handler(void)
{
        res = 1;
}

/*----------------------------------------------------------------
 * void PORT5_IRQHandler(void)
 *
 * Description: Port 5 interrupt used to grab status of RF IRQ pin
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
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

/*----------------------------------------------------------------
 * void GPIO_init()
 *
 * Description: Disables all pins to minimize power consumption
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
void GPIO_init()
{
    /* Terminate all GPIO pins to Output LOW to minimize power consumption */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_PA, PIN_ALL16);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_PB, PIN_ALL16);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_PC, PIN_ALL16);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_PD, PIN_ALL16);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_PE, PIN_ALL16);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_PA, PIN_ALL16);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_PB, PIN_ALL16);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_PC, PIN_ALL16);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_PD, PIN_ALL16);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_PE, PIN_ALL16);
}

/*----------------------------------------------------------------
 * void ADC14_IRQHandler()
 *
 * Description: ADC interrupt to fetch data from EMG sensors
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
void ADC14_IRQHandler()
/*
 * purpose set up the ADC14-IRQhandler to track the change in voltages from the EMG data readings

 */
{
    uint32_t flags = ADC14->IFGR0; //reads flags and clears
    if(flags&BIT(10)) // if interrupt flag
    {
        P4_7_Voltage = ADC14->MEM[10]/16383.0*3.3;
        Index_ArrLT = 1;
    }
    if(flags&BIT(11)) // if interrupt flag
    {
        P5_5_Voltage = ADC14->MEM[11]/(16383.0)*3.3;
        Index_ArrLB = 1;
    }
    if(flags&BIT(12)) // if interrupt flag
    {
        P5_4_Voltage = ADC14->MEM[12]/16383.0*3.3;
        Index_ArrRB = 1;
    }

    if(flags&BIT(13)) // if interrupt flag
    {
        P5_2_Voltage = ADC14->MEM[13]/16383.0*3.3;
        Index_ArrRT = 1;
    }

}

/*----------------------------------------------------------------
 * void setupTimerA3
 *
 * Description: Sets up Timer A3 for interrupts and ADC counting
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
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

/*----------------------------------------------------------------
 * void TA3_N_IRQHandler()
 *
 * Description: Timer A3 interrupt to control ADC conversion
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
void TA3_N_IRQHandler()
{
    ADC14->CTL0 |= BIT0; //Start the Analog Conversion
    TIMER_A3->CTL &= ~BIT0;
}

/*----------------------------------------------------------------
 * void ADC_Init()
 *
 * Description: Sets up ADC to read 4 channels
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
void ADC_Init()
{
    P4->SEL0 |=  BIT7; // -- // SEL == 00 -> GPIO
    P4->SEL1 |=  BIT7;//  --SEL == 01 -> TimerA
    P4->DIR  &= ~BIT7;//  --  SEL == 11 -> Analog Input

    P5->SEL0 |=  BIT5;
    P5->SEL1 |=  BIT5;
    P5->DIR  &= ~BIT5;

    P5->SEL0 |=  BIT4;
    P5->SEL1 |=  BIT4;
    P5->DIR  &= ~BIT4;

    P5->SEL0 |=  BIT2;
    P5->SEL1 |=  BIT2;
    P5->DIR  &= ~BIT2;

    ADC14->CTL0 = 0b11000101111000100111011110010000;

    ADC14->CTL1 = 0b010100000000000110000;

    ADC14->MCTL[10] = 6;  //read A6
    ADC14->MCTL[11] = 0;  //read A0
    ADC14->MCTL[12] = 1;  //read A1
    ADC14->MCTL[13] = 3|BIT7;  //read A3  // End of Sequence.

    //MEM[0]
    // 15-0   Contains results of read

    //IE0
    // Interrupt on MCTL[10], MCTL[11],MCTL[12] and MCTL[13] being completed
    ADC14->IER0 = BIT(10)|BIT(11)|BIT(12)|BIT(13);
    //IE1
    // All zeros to not interrupt enable
    ADC14->IER1 = 0;

    ADC14->CTL0 |= BIT1; //Enable the Analog Conversion

    NVIC_EnableIRQ(ADC14_IRQn);
}
