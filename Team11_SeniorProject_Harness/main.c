#include "msp.h"

#include "driverlib.h"
#include "msp.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "clockConfig.h"
#include "nRF24_driver.h"           // eusci_b2


volatile int second_count;

int res = 0;


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

        /* Initialize UART */
        UART_initGPIO();
        UART_init();



        while(1)
        {

            if(res == 1){

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

         }
        while(1)
            {
                short result16 = 0;
                char str[64];

                /* Read Design Capacity */
                if(!BQ27441_read16(DESIGN_CAPACITY, &result16, 1000))
                    UART_transmitString("Error Reading Design Capacity \r\n");
                else
                {
                    sprintf(str, "Design Capacity: %dmAh\r\n", result16);
                    UART_transmitString(str);
                }

                /* Read Remaining Capacity */
                if(!BQ27441_read16(REMAINING_CAPACITY, &result16, 1000))
                    UART_transmitString("Error Reading Remaining Capacity \r\n");
                else
                {
                    sprintf(str, "Remaining Capacity: %dmAh\r\n", result16);
                    UART_transmitString(str);
                }

                /* Read State Of Charge */
                if(!BQ27441_read16(STATE_OF_CHARGE, &result16, 1000))
                    UART_transmitString("Error Reading State Of Charge \r\n");
                else
                {
                    sprintf(str, "State of Charge: %d%%\r\n", (unsigned short)result16);
                    UART_transmitString(str);
                }

                /* Read Temperature */
                if(!BQ27441_read16(TEMPERATURE, &result16, 1000))
                    UART_transmitString("Error Reading Temperature \r\n");
                else
                {
                    sprintf(str, "Temperature: %dC\r\n", result16/10 - 273);
                    UART_transmitString(str);
                }

                /* Read Voltage */
                if(!BQ27441_read16(VOLTAGE, &result16, 1000))
                    UART_transmitString("Error Reading Voltage \r\n");
                else
                {
                    sprintf(str, "Voltage: %dmV\r\n", result16);
                    UART_transmitString(str);
                }

                /* Read Average Current */
                if(!BQ27441_read16(AVERAGE_CURRENT, &result16, 1000))
                    UART_transmitString("Error Reading Average Current \r\n");
                else
                {
                    sprintf(str, "Average Current: %dmA\r\n", result16);
                    UART_transmitString(str);
                    if (result16 > 0) {
                        UART_transmitString("Status : charging\r\n");
                    } else {
                        UART_transmitString("Status : discharging\r\n");
                    }
                }

                __delay_cycles(20000000);
            }
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
