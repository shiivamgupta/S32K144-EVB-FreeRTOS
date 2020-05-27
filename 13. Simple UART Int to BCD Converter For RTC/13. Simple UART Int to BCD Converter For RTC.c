/* I have created one task Task1 and using UART0 baud rate 115200 with interrupt */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include<stdio.h>
#include "UART0.h"

#include "interrupt_manager.h"
#include "clock_manager.h"
#include "clockMan1.h"
#include "pin_mux.h"
#include "semphr.h"
#include "BoardDefines.h"

#include "dmaController1.h"
/* User includes (#include below this line is not maintained by Processor Expert) */
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* Timeout in ms for blocking operations */
#define TIMEOUT     100U

/* Receive buffer size */
#define BUFFER_SIZE 256U

/* Buffer used to receive data from the console */
uint8_t buffer[BUFFER_SIZE];
uint8_t bufferIdx;


#define BTN1			13U
#define BTN_GPIO        PTC
#define BTN1_PIN        13U
#define BTN2_PIN        12U
#define BTN_PORT        PORTC
#define BTN_PORT_IRQn   PORTC_IRQn

#define TASK2_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define	TASK1_PRIORITY		( tskIDLE_PRIORITY + 1 )


#define mainDONT_BLOCK						( 0UL )


static void prvSetupHardware( void );


void BTN_ISR( void *pvParameters );
void Task1( void *pvParameters );

TaskHandle_t xHandle1;
TaskHandle_t xHandle2;


/*-----------------------------------------------------------*/

uint8_t DecToBCD(uint8_t x)
{
	uint8_t result=0;
    result|=(x/10);
    result=(result<<4);
    result|=(x%10);

    return result;
}

/*-----------------------------------------------------------*/
void rtos_start( void )
{
	prvSetupHardware();

	xTaskCreate( Task1, "Task1", configMINIMAL_STACK_SIZE, NULL, TASK1_PRIORITY, &xHandle1 );
	//xTaskCreate( Task2, "Task2", configMINIMAL_STACK_SIZE, NULL, TASK2_PRIORITY, &xHandle2 );
	vTaskStartScheduler();

}

/*-----------------------------------------------------------*/

void Task1( void *pvParameters )
{
	(void)pvParameters;
	uint8_t BCDValue=0;
	BCDValue=DecToBCD(78);
	for( ;; )
	{
		LPUART_DRV_SendData(INST_UART0, (uint8_t *)&BCDValue, 1);
		vTaskDelay(1000);

	}
}


/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{

    CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT,
                   g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
    CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_AGREEMENT);

    boardSetup();

    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);
	//PINS_DRV_SetPins(LED_GPIO, (1 << LED1) | (1 << LED2)|(1 << LED3));
	PINS_DRV_SetPinIntSel(BTN_PORT, BTN1_PIN, PORT_INT_RISING_EDGE);
	PINS_DRV_SetPinIntSel(BTN_PORT, BTN2_PIN, PORT_INT_RISING_EDGE);
	/* Install buttons ISR */
	LPUART_DRV_Init(INST_UART0, &UART0_State, &UART0_InitConfig0);

	INT_SYS_DisableIRQ(LPUART0_RxTx_IRQn);
	INT_SYS_SetPriority(LPUART0_RxTx_IRQn, 2);
	INT_SYS_EnableIRQ(LPUART0_RxTx_IRQn);

	INT_SYS_SetPriority(BTN_PORT_IRQn, 3);

		/* Enable buttons interrupt */
	INT_SYS_EnableIRQ(BTN_PORT_IRQn);
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	taskDISABLE_INTERRUPTS();
	for( ;; );
}

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;
	taskDISABLE_INTERRUPTS();
	for( ;; );
}

/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
    volatile size_t xFreeHeapSpace;

	xFreeHeapSpace = xPortGetFreeHeapSize();

	if( xFreeHeapSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}

}
/*-----------------------------------------------------------*/

/* The Blinky build configuration does not include run time stats gathering,
however, the Full and Blinky build configurations share a FreeRTOSConfig.h
file.  Therefore, dummy run time stats functions need to be defined to keep the
linker happy. */
void vMainConfigureTimerForRunTimeStats( void ) {}
unsigned long ulMainGetRunTimeCounterValue( void ) { return 0UL; }

/* A tick hook is used by the "Full" build configuration.  The Full and blinky
build configurations share a FreeRTOSConfig.h header file, so this simple build
configuration also has to define a tick hook - even though it does not actually
use it for anything. */
void vApplicationTickHook( void ) {}

/*-----------------------------------------------------------*/


