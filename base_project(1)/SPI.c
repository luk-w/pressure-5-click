/**********************************************************************************************************************
 * \file    SPI.c
 * \brief
 * \version V1.0.0
 * \date    06.01.2024
 * \author  Administrator
 *********************************************************************************************************************/

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/

#include <stdio.h>
#include <string.h>
#include "SPI.h"

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

IfxQspi_SpiMaster spi;
IfxQspi_SpiMaster_Channel spiChannel;

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

IFX_INTERRUPT(qspi0TxISR, 0, IFX_INTPRIO_QSPI1_TX)
{
    IfxQspi_SpiMaster_isrTransmit(&spi);
}

IFX_INTERRUPT(qspi0RxISR, 0, IFX_INTPRIO_QSPI1_RX)
{
    IfxQspi_SpiMaster_isrReceive(&spi);
}

IFX_INTERRUPT(qspi0ErISR, 0, IFX_INTPRIO_QSPI1_ER)
{
    IfxQspi_SpiMaster_isrError(&spi);
}

void spi_init()
{

    IfxQspi_SpiMaster_Config spiMasterConfig;
    IfxQspi_SpiMaster_initModuleConfig(&spiMasterConfig, &MODULE_QSPI1);

    // set the desired mode and maximum baudrate
    spiMasterConfig.base.mode = SpiIf_Mode_master;
    spiMasterConfig.base.maximumBaudrate = 10000000;

    // ISR priorities and interrupt target
    spiMasterConfig.base.txPriority = IFX_INTPRIO_QSPI1_TX;
    spiMasterConfig.base.rxPriority = IFX_INTPRIO_QSPI1_RX;
    spiMasterConfig.base.erPriority = IFX_INTPRIO_QSPI1_ER;
    spiMasterConfig.base.isrProvider = IfxCpu_Irq_getTos(IfxCpu_getCoreIndex());

    // pin configuration
    const IfxQspi_SpiMaster_Pins pins = {
        &IfxQspi1_SCLK_P10_2_OUT, IfxPort_OutputMode_pushPull, // SCLK
        &IfxQspi1_MTSR_P10_3_OUT, IfxPort_OutputMode_pushPull, // MTSR
        &IfxQspi1_MRSTA_P10_1_IN, IfxPort_InputMode_pullDown,  // MRST
        IfxPort_PadDriver_cmosAutomotiveSpeed3                 // pad driver mode
    };

    spiMasterConfig.pins = &pins;

    // initialize module
    // IfxQspi_SpiMaster spi; // defined globally
    IfxQspi_SpiMaster_initModule(&spi, &spiMasterConfig);

    // create channel config
    IfxQspi_SpiMaster_ChannelConfig spiMasterChannelConfig;
    IfxQspi_SpiMaster_initChannelConfig(&spiMasterChannelConfig, &spi);

    // set the baudrate for this channel
    spiMasterChannelConfig.base.baudrate = 5000000;

    // select pin configuration
    const IfxQspi_SpiMaster_Output slsOutput = {&IfxQspi1_SLSO9_P10_5_OUT, IfxPort_OutputMode_pushPull,
                                                IfxPort_PadDriver_cmosAutomotiveSpeed1};
    spiMasterChannelConfig.sls.output = slsOutput;

    // initialize channel
    // IfxQspi_SpiMaster_Channel spiChannel; // defined globally
    IfxQspi_SpiMaster_initChannel(&spiChannel, &spiMasterChannelConfig);
}

void spi_read(uint8 reg_addr, uint8 *reg_data, uint32 len)
{
    uint8 spiTxBuffer[SPI_BUFFER_SIZE];
    uint8 spiRxBuffer[SPI_BUFFER_SIZE];

    spiTxBuffer[0] = reg_addr;

    while (IfxQspi_SpiMaster_getStatus(&spiChannel) == SpiIf_Status_busy)
        ;

    IfxQspi_SpiMaster_exchange(&spiChannel, spiTxBuffer, spiRxBuffer, SPI_BUFFER_SIZE);
}

void spi_write(uint8 reg_addr, uint8 *reg_data, uint32 len)
{
    uint8 spiTxBuffer[SPI_BUFFER_SIZE];

    spiTxBuffer[0] = reg_addr;
    memcpy(&spiTxBuffer[1], reg_data, len);

    while (IfxQspi_SpiMaster_getStatus(&spiChannel) == SpiIf_Status_busy)
        ;

    IfxQspi_SpiMaster_exchange(&spiChannel, spiTxBuffer, NULL_PTR, SPI_BUFFER_SIZE);
}
