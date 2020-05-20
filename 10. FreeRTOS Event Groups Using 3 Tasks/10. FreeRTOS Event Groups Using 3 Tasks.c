/* I have created two tasks Task1 & Task2 having same priority and working after every 1 ms accurate delay using vTaskDelayUntil*/

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include<stdio.h>

#include "interrupt_manager.h"
#include "clock_manager.h"
#include "clockMan1.h"
#include "pin_mux.h"
#include "semphr.h"
#include "event_groups.h"
#include "BoardDefines.h"
//#include "BoardDefines.h"

#define LED_PORT 	PORTD
#define GPIO_PORT	PTD
#define PCC_CLOCK	PCC_PORTD_CLOCK
#define LED1		15U
#define LED2		16U
#define LED3		0U

#define BTN1			13U
#define BTN_GPIO        PTC
#define BTN1_PIN        13U
#define BTN2_PIN        12U
#define BTN_PORT        PORTC
#define BTN_PORT_IRQn   PORTC_IRQn

#define TASK3_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define TASK2_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define	TASK1_PRIORITY		( tskIDLE_PRIORITY + 1 )

#define TASK1_FREQUENCY_MS 	( 1000 / portTICK_PERIOD_MS )
#define TASK2_FREQUENCY_MS 	( 1000 / portTICK_PERIOD_MS )

#define mainDONT_BLOCK						( 0UL )


static void prvSetupHardware( void );

void Task2( void *pvParameters );
void Task1( void *pvParameters );
void Sw_Watchdog( void *pvParameters );

TaskHandle_t xHandle1;
TaskHandle_t xHandle2;
EventGroupHandle_t task_watchdog;
const uint32_t task1_id= (1<<0);
const uint32_t task2_id= (1<<1);

#define taskall	(task1_id | task2_id)

/*-----------------------------------------------------------*/
void rtos_start( void )
{
	prvSetupHardware();
	task_watchdog= xEventGroupCreate();
	xTaskCreate( Task1, "Task1", configMINIMAL_STACK_SIZE, NULL, TASK1_PRIORITY, &xHandle1 );
	xTaskCreate( Task2, "Task2", configMINIMAL_STACK_SIZE, NULL, TASK2_PRIORITY, &xHandle2 );
	xTaskCreate( Sw_Watchdog, "Task3", configMINIMAL_STACK_SIZE, NULL, TASK3_PRIORITY, &xHandle2 );
	vTaskStartScheduler();

}

/*-----------------------------------------------------------*/

void Task1( void *pvParameters )
{
	(void)pvParameters;
	for( ;; )
	{
		PINS_DRV_TogglePins(PTD, (1 << LED2));
		vTaskDelay(1000);
		xEventGroupSetBits(task_watchdog, task1_id);

	}
}

/*-----------------------------------------------------------*/

void Task2( void *pvParameters )
{
	(void)pvParameters;
		for( ;; )
		{
			PINS_DRV_TogglePins(PTD, (1 << LED1));
			vTaskDelay(1000);
			xEventGroupSetBits(task_watchdog, task2_id);
		}
}

/*-----------------------------------------------------------*/

void Sw_Watchdog( void *pvParameters )
{
	(void)pvParameters;
		for( ;; )
		{
			uint32_t result = xEventGroupWaitBits(task_watchdog, taskall, pdTRUE, pdTRUE, 2000 );
			if((result&taskall)==taskall)
			{
				PINS_DRV_TogglePins(PTD, (1 << LED3));
			}
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
	PINS_DRV_SetPins(PTD, (1 << LED1) | (1 << LED2)|(1 << LED3));
	PINS_DRV_SetPinIntSel(BTN_PORT, BTN1_PIN, PORT_INT_RISING_EDGE);
	PINS_DRV_SetPinIntSel(BTN_PORT, BTN2_PIN, PORT_INT_RISING_EDGE);
	/* Install buttons ISR */
	//INT_SYS_InstallHandler(BTN_PORT_IRQn, &INTE_ISR, NULL);
	//INT_SYS_SetPriority(BTN_PORT_IRQn, 3);
	/* Enable buttons interrupt */
	//INT_SYS_EnableIRQ(BTN_PORT_IRQn);
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


