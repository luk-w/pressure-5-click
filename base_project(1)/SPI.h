/*
 * SPI.h
 *
 *  Created on: 06.01.2024
 *      Author: Administrator
 */

#ifndef SPI_H_
#define SPI_H_

#include "Ifx_Types.h"
//QSPI
#include <IfxQspi_SpiMaster.h>

#define IFX_INTPRIO_QSPI1_TX  1
#define IFX_INTPRIO_QSPI1_RX  2
#define IFX_INTPRIO_QSPI1_ER  5



void spi_init(void);

void spi_loop(void);

#endif /* SPI_H_ */
