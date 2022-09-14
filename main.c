/*
 * FreeRTOS Kernel V10.3.1
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"
#include "queue.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"


/*-----------------------------------------------------------*/
BaseType_t xTaskPeriodicCreate(	TaskFunction_t pxTaskCode,
								const char * const pcName,		/*lint !e971 Unqualified char types are allowed for strings and single characters only. */
								const configSTACK_DEPTH_TYPE usStackDepth,
								void * const pvParameters,
								UBaseType_t uxPriority,
								TaskHandle_t * const pxCreatedTask,
								TickType_t period );
TaskHandle_t xButton1MonitorHandle = NULL;
TaskHandle_t xButton2MonitorHandle = NULL;
TaskHandle_t xPeriodicTransmitterHandle = NULL;
TaskHandle_t xUartReceiverHandle = NULL;
TaskHandle_t xLoad1SimulationHandle = NULL;
TaskHandle_t xLoad2SimulationHandle = NULL;
QueueHandle_t xStructQueue = NULL;
/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )

#define FIXED_PRIORITY		2
/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/

unsigned int B1_in;
unsigned int B2_in;
unsigned int L1_in;
unsigned int L2_in;
unsigned int Tx_in;
unsigned int Rx_in;

unsigned int Tick;

unsigned int B1;
unsigned int B2;
unsigned int L1;
unsigned int L2;
unsigned int Tx;
unsigned int Rx;
unsigned int ideal;

unsigned int totalB1;
unsigned int totalB2;
unsigned int totalL1;
unsigned int totalL2;
unsigned int totalTx;
unsigned int totalRx;

unsigned int totalExection;
unsigned int cpuLoad;


char * button1_rising = "rise1 ";
char * button1_falling = "fall1 ";
char * button2_rising = "rise2 ";
char * button2_falling = "fall2 ";
char * periodic_tx = "100m";



void vApplicationTickHook (void)
{
	Tick = 1;
	Tick = 0;
}


void vButton1Monitor( void * pvParameters )
{
	
	static pinState_t prvState = PIN_IS_HIGH;
	pinState_t CurrentState;
	for( ;; )
	{
		CurrentState = GPIO_read(PORT_1, PIN1);
		if (prvState == PIN_IS_LOW && CurrentState == PIN_IS_HIGH)
		{
			
			xQueueSend( xStructQueue,
                  ( void * ) &button1_rising,
                  ( TickType_t ) 0 );
							 
			prvState = CurrentState;
							
				
		}
		else if (prvState == PIN_IS_HIGH && CurrentState == PIN_IS_LOW)
		{
			xQueueSend( xStructQueue,
                  ( void * ) &button1_falling,
                  ( TickType_t ) 0 );
				prvState = CurrentState;
		}
		vTaskDelay(50);

	}

}

void vButton2Monitor( void * pvParameters )
{
	static pinState_t prvState = PIN_IS_HIGH;
	pinState_t CurrentState;
	for( ;; )
	{
		CurrentState = GPIO_read(PORT_1, PIN2);
		if (prvState == PIN_IS_LOW && CurrentState == PIN_IS_HIGH)
		{
			
			xQueueSend(xStructQueue,
                 ( void * ) &button2_rising,
                 ( TickType_t ) 0 );
							 
			prvState = CurrentState;
							
				
		}
		else if (prvState == PIN_IS_HIGH && CurrentState == PIN_IS_LOW)
		{
			xQueueSend( xStructQueue,
                  ( void * ) &button2_falling,
                  ( TickType_t ) 0 );
				prvState = CurrentState;
		}
		vTaskDelay(50);
		
	}

}

void vPeriodicTransmitter( void * pvParameters )
{
	for( ;; )
	{
		xQueueSend( xStructQueue,
               	    ( void * ) &periodic_tx,
               		( TickType_t ) 0 );
		vTaskDelay(100);
	}

}


void vUartReceiver( void * pvParameters )
{
	char  *receive;
	uint32_t ulNotifiedValue = 0;
	BaseType_t xResult;
	for( ;; )
	{
		if ( xQueueReceive( xStructQueue,
                            &receive,
                            100 ) == pdPASS )
		{
			vSerialPutString(receive, 6);
		}
	}

}

void vLoad1Simulation( void * pvParameters )
{
	int i;
	for( ;; )
	{
		for(i = 0; i < 5 * 7500; i++);
		vTaskDelay(5);
	}

}

void vLoad2Simulation( void * pvParameters )
{
	int i;
	for( ;; )
	{
		for(i = 0; i < 12 * 7500; i++);
		vTaskDelay(88);
	}

}

/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
	

	xTaskPeriodicCreate(
                    	vButton1Monitor,       /* Function that implements the task. */
                    	"B1",          /* Text name for the task. */
                    	100,      /* Stack size in words, not bytes. */
                    	( void * ) 1,    /* Parameter passed into the task. */
                    	FIXED_PRIORITY,/* Priority at which the task is created. */
                    	&xButton1MonitorHandle,
						50 );      /* Used to pass out the created task's handle. */

	xTaskPeriodicCreate(
                    	vButton2Monitor,       /* Function that implements the task. */
                    	"B2",          /* Text name for the task. */
                    	100,      /* Stack size in words, not bytes. */
                    	( void * ) 1,    /* Parameter passed into the task. */
                    	FIXED_PRIORITY,/* Priority at which the task is created. */
                    	&xButton2MonitorHandle,
						50 );      /* Used to pass out the created task's handle. */

	xTaskPeriodicCreate(
                    	vPeriodicTransmitter,       /* Function that implements the task. */
                    	"Tx",          /* Text name for the task. */
                    	100,      /* Stack size in words, not bytes. */
                    	( void * ) 1,    /* Parameter passed into the task. */
                    	FIXED_PRIORITY,/* Priority at which the task is created. */
                    	&xPeriodicTransmitterHandle,
						100 );      /* Used to pass out the created task's handle. */

	xTaskPeriodicCreate(
                    	vUartReceiver,       /* Function that implements the task. */
                    	"Rx",          /* Text name for the task. */
                    	100,      /* Stack size in words, not bytes. */
                    	( void * ) 1,    /* Parameter passed into the task. */
                    	FIXED_PRIORITY,/* Priority at which the task is created. */
                    	&xUartReceiverHandle,
						100 );      /* Used to pass out the created task's handle. */

	xTaskPeriodicCreate(
                    	vLoad1Simulation,       /* Function that implements the task. */
                    	"L1",          /* Text name for the task. */
                    	100,      /* Stack size in words, not bytes. */
                    	( void * ) 1,    /* Parameter passed into the task. */
                    	FIXED_PRIORITY,/* Priority at which the task is created. */
                    	&xLoad1SimulationHandle,
						10 );      /* Used to pass out the created task's handle. */

	xTaskPeriodicCreate(
                    	vLoad2Simulation,       /* Function that implements the task. */
                    	"L2",          /* Text name for the task. */
                    	100,      /* Stack size in words, not bytes. */
                    	( void * ) 1,    /* Parameter passed into the task. */
                    	FIXED_PRIORITY,/* Priority at which the task is created. */
                    	&xLoad2SimulationHandle,
						100 );      /* Used to pass out the created task's handle. */

	
    /* Create Tasks here */
	
	xStructQueue = xQueueCreate(
                     		     1,  /* The number of items the queue can hold. */
		                         sizeof( char *) ); /* Size of each item is big enough to hold the whole structure. */



	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/


