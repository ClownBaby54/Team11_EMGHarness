/*
 * nRF24_driver.h
 *
 *  Created on: Feb 2018
 *      Author: lourw
 */

#ifndef NRF24_DRIVER_H_
#define NRF24_DRIVER_H_

#include "driverlib.h"
#include "msprf24.h"
#include "nrf_userconfig.h"


void nRF24_init(void);
void nRF24_send(uint8_t len, uint8_t *buf);


#endif /* NRF24_DRIVER_H_ */
