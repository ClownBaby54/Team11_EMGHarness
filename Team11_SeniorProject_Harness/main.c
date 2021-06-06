

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


volatile int second_count;


void CS_init(void);
void GPIO_init(void);
char *itoa(int, char*, int);


int res = 0;

//Array of possible errors
int Err[50];

void main(void)
{
    // data used by the rf
        char rf_data[5] = "HELP";

        rf_data[4] = 0;


        /* Halting the Watchdog  */
        MAP_WDT_A_holdTimer();

        // red led initialization
        MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

        MAP_SysTick_setPeriod(600000);
        MAP_SysTick_enableModule();

        // initiate rf transceiver
        nRF24_init();

        // enable interrupts
        MAP_Interrupt_disableMaster();
        MAP_SysTick_enableInterrupt();
        MAP_Interrupt_enableMaster();

        GPIO_init();
        CS_init();

        __delay_cycles(1000000);

        /* Initialize I2C */
        I2C_initGPIO();
        I2C_init();



        while(1)
        {

            if(res == 1)
            {

                // reset second_count
                res = 0;

                // red led on
                MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);

                // send and write the data
                nRF24_send(5, (char *)rf_data);

                // debugging statements
                printf(rf_data); printf("\n\r");


                // red led off
                MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

                short result16 = 0;
                char str[64];


                /* Read State Of Charge */
                if(!BQ27441_read16(STATE_OF_CHARGE, &result16, 1000))
                    Err[0] = 1;
                else
                {
                    sprintf(str, "State of Charge: %d%%\r\n", (unsigned short)result16);
                }



                __delay_cycles(20000000);
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


/* Initializes Clock System */
void CS_init()
{
    /* Set the core voltage level to VCORE1 */
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);

    /* Set 2 flash wait states for Flash bank 0 and 1*/
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);

    /* Initializes Clock System */
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);
    MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    MAP_CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
}


/* Initializes GPIO */
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
