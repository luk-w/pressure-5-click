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
    
    initUART();

    while(1) {
        //UART TEST CODE - REMOVE FOR YOUR OWN PROJECT****************************
        uint8 byte = 'A';
        uart_blockingWrite(byte);
        uart_blockingWrite('\n');

        waitTime(IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, WAIT_TIME));

        uint8 msg[11] = "Hello UART\n";
        uart_sendMessage((uint8*)msg, sizeof(msg));
        //END UART TEST CODE *****************************************************
    }
    return (1);
}
