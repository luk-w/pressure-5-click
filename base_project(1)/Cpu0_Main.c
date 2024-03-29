/**********************************************************************************************************************
 * \file Cpu0_Main.c
 * \copyright Copyright (C) Infineon Technologies AG 2019
 * 
 *  Created on: 2 Nov 2022
 *  Author: Technikum Wien
 * 
 * 
 *********************************************************************************************************************/
#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include <Asclin/Asc/IfxAsclin_Asc.h> //Needed for UART
#include <Bsp.h>                      //Board support functions (for the waitTime function)
#include <UART.h>
#include "SPI.h"
#include "bmp3_aurix.h"
IfxCpu_syncEvent g_cpuSyncEvent = 0;

#define WAIT_TIME 1000


int core0_main(void) {

    IfxCpu_enableInterrupts();
    
    /* !!WATCHDOG0 AND SAFETY WATCHDOG ARE DISABLED HERE!!
     * Enable the watchdogs and service them periodically if it is required
     */
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());
    
    /* Wait for CPU sync event */
    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 1);
    
    spi_init();
    initUART();
    bmp3_aurix_init();


    while(1) {

        waitTime(IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, WAIT_TIME));
        bmp3_aurix_readout();

    }

}
