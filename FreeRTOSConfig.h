/*
 * FreeRTOS Kernel V10.3.0
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <lpc21xx.h>
#include "GPIO.h"


extern unsigned int B1_in;
extern unsigned int B2_in;
extern unsigned int L1_in;
extern unsigned int L2_in;
extern unsigned int Tx_in;
extern unsigned int Rx_in;


extern unsigned int B1;
extern unsigned int B2;
extern unsigned int L1;
extern unsigned int L2;
extern unsigned int Tx;
extern unsigned int Rx;
extern unsigned int ideal;

extern unsigned int totalB1;
extern unsigned int totalB2;
extern unsigned int totalL1;
extern unsigned int totalL2;
extern unsigned int totalTx;
extern unsigned int totalRx;

extern unsigned int totalExection;
extern unsigned int cpuLoad;

#define SET_P0_PIN(PINx)		GPIO_write( PORT_0 , PINx , PIN_IS_HIGH)
#define CLR_P0_PIN(PINx)		GPIO_write( PORT_0 , PINx , PIN_IS_LOW)

#define TASK_NAME			(pxCurrentTCB->pcTaskName) 
/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE. 
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

#define configUSE_EDF_SCHEDULER     1
#define configUSE_PREEMPTION		1
#define configUSE_IDLE_HOOK			0
#define configUSE_TICK_HOOK			1
#define configCPU_CLOCK_HZ			( ( unsigned long ) 60000000 )	/* =12.0MHz xtal multiplied by 5 using the PLL. */
#define configTICK_RATE_HZ			( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES		( 4 )
#define configMINIMAL_STACK_SIZE	( ( unsigned short ) 90 )
#define configTOTAL_HEAP_SIZE		( ( size_t ) 13 * 1024 )
#define configMAX_TASK_NAME_LEN		( 8 )
#define configUSE_TRACE_FACILITY	0
#define configUSE_16_BIT_TICKS		0
#define configIDLE_SHOULD_YIELD		1

#define configUSE_EDF_SCHEDULER     1

#define configQUEUE_REGISTRY_SIZE 	0

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES 		0
#define configMAX_CO_ROUTINE_PRIORITIES ( 2 )

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */

#define INCLUDE_vTaskPrioritySet		1
#define INCLUDE_uxTaskPriorityGet		1
#define INCLUDE_vTaskDelete				1
#define INCLUDE_vTaskCleanUpResources	0
#define INCLUDE_vTaskSuspend			1
#define INCLUDE_vTaskDelayUntil			1
#define INCLUDE_vTaskDelay				1

#define traceTASK_SWITCHED_IN()		if (strcmp( TASK_NAME, "B1" ) == 0 ){ B1 = 1; B1_in = T1TC;}  	\
									if (strcmp( TASK_NAME, "B2" ) == 0 ){ B2 = 1; B2_in = T1TC;}  	\
									if (strcmp( TASK_NAME, "Tx" ) == 0 ){ Tx = 1; Tx_in = T1TC;}  	\
									if (strcmp( TASK_NAME, "Rx" ) == 0 ){ Rx = 1; Rx_in = T1TC;}  	\
									if (strcmp( TASK_NAME, "L1" ) == 0 ){ L1 = 1; L1_in = T1TC;}  	\
									if (strcmp( TASK_NAME, "L2" ) == 0 ){ L2 = 1; L2_in = T1TC;}  	\
									if (strcmp( TASK_NAME, "IDLE" ) == 0 ){ ideal = 1;}  			\
									while(0)
											 


#define traceTASK_SWITCHED_OUT()    if (strcmp( TASK_NAME, "B1" ) == 0 ){ B1 = 0; totalExection += T1TC-B1_in; totalB1 += T1TC-B1_in;}  \
								    if (strcmp( TASK_NAME, "B2" ) == 0 ){ B2 = 0; totalExection += T1TC-B2_in; totalB2 += T1TC-B2_in;}  \
								    if (strcmp( TASK_NAME, "Tx" ) == 0 ){ Tx = 0; totalExection += T1TC-Tx_in; totalTx += T1TC-Tx_in;}  \
								    if (strcmp( TASK_NAME, "Rx" ) == 0 ){ Rx = 0; totalExection += T1TC-Rx_in; totalRx += T1TC-Rx_in;}  \
								    if (strcmp( TASK_NAME, "L1" ) == 0 ){ L1 = 0; totalExection += T1TC-L1_in; totalL1 += T1TC-L1_in;}  \
								    if (strcmp( TASK_NAME, "L2" ) == 0 ){ L2 = 0; totalExection += T1TC-L2_in; totalL2 += T1TC-L2_in;}  \
								    if (strcmp( TASK_NAME, "IDLE" ) == 0 ){ ideal = 0;}  						                        \
								    cpuLoad = (totalExection * 100) / T1TC;											        			\
								    while(0)
#endif /* FREERTOS_CONFIG_H */
