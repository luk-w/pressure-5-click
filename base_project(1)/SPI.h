/*
 * SPI.h
 *
 *  Created on: 06.01.2024
 *      Author: Administrator
 */

#ifndef SPI_H_
#define SPI_H_

#include "Ifx_Types.h"
#include <IfxQspi_SpiMaster.h>

#define IFX_INTPRIO_QSPI1_TX  1
#define IFX_INTPRIO_QSPI1_RX  2
#define IFX_INTPRIO_QSPI1_ER  5

#define SPI_BUFFER_SIZE 128

void spi_init(void);
void spi_read(uint8 reg_addr, uint8 *reg_data, uint32 len);
void spi_write(uint8 reg_addr, uint8 *reg_data, uint32 len);

#endif /* SPI_H_ */
