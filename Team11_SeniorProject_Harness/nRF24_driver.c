/*
 * nRF24_driver.c
 *
 *  Created on: Feb 26, 2018
 *      Author: lourw
 */

#include "nRF24_driver.h"
char ack_rf_data[5];


void nRF24_init(void)
{

    //P5.0 -> IRQ
    //P3.5 -> SCK
    //P3.6 -> MOSI
    //P3.7 -> MISO
    //P4.1 -> CSN
    //P4.0 -> CE


    // declare address of other transceiver
    uint8_t addr[5] = "rad01";
    uint8_t myaddy[5] = "rad02";
    // green led initialization
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);

    // setup port interrupt
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5, GPIO_PIN0);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P5, GPIO_PIN0,
                                 GPIO_HIGH_TO_LOW_TRANSITION);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5, GPIO_PIN0);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P5, GPIO_PIN0);
    MAP_Interrupt_enableInterrupt(INT_PORT5);

    /* Initial values for nRF24L01+ library config variables */
    rf_crc = RF24_EN_CRC | RF24_CRCO; // CRC enabled, 16-bit
    rf_addr_width      = 5;
    rf_speed_power     = RF24_SPEED_1MBPS | RF24_POWER_MIN;
//    rf_channel         = 105;
    rf_channel = 76;

    msprf24_init();  // All RX pipes closed by default
    //msprf24_set_pipe_packetsize(0, 4);
    msprf24_set_pipe_packetsize(0, 0);
  //  msprf24_set_pipe_packetsize(1, 0);  //test
  //  msprf24_open_pipe(0, 0);
    msprf24_open_pipe(0, 1);
    //msprf24_open_pipe(1, 0);            //test
    msprf24_standby();

    w_tx_addr(addr);
    //w_rx_addr(0, addr);  // Pipe 0 receives auto-ack's, autoacks are sent back to the TX addr so the PTX node
                         // needs to listen to the TX addr on pipe#0 to receive them.
    w_rx_addr(0, addr);               //test
}

void nRF24_send(uint8_t len, uint8_t *buf)
{
    w_tx_payload(len, buf);
    msprf24_activate_tx();

//    if(msprf24_rx_pending())
//    {
//        r_rx_payload(5, (char *)ack_rf_data);
//        flush_rx();
//    }

    __delay_cycles(15000);

    if (rf_irq & RF24_IRQ_FLAGGED) {
        rf_irq &= ~RF24_IRQ_FLAGGED;

        msprf24_get_irq_reason();
        if (rf_irq & RF24_IRQ_TX){
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
        }
        if (rf_irq & RF24_IRQ_TXFAILED){
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
        }

        msprf24_irq_clear(rf_irq);
    }

}
