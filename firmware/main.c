#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

static void hardware_init();
static void blinky();

static void prvQueueReceiveTask( void *pvParameters );
static void prvQueueSendTask( void *pvParameters );

static QueueHandle_t xQueue = NULL;

int main(void) {
	hardware_init();
	blinky();

	return 0;
}

static void hardware_init() {
	stdio_init_all();
	cyw43_arch_init();
}

static void blinky() {
	printf("Staring blinky\n");

	xQueue = xQueueCreate(1, sizeof(uint32_t));
	if (xQueue != NULL) {
		xTaskCreate(prvQueueReceiveTask, "Rx", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+2, NULL);
		xTaskCreate(prvQueueSendTask, "Tx", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL);
		vTaskStartScheduler();

		// shouldn't get here
		while(1);
	}
}

static void prvQueueReceiveTask( void *pvParameters ) {
	unsigned long ulReceivedValue;
	const unsigned long ulExpectedValue = 100UL;

	while(1) {
		xQueueReceive(xQueue, &ulReceivedValue, portMAX_DELAY);
		if (ulReceivedValue == ulExpectedValue) {
			cyw43_arch_gpio_put(0, !cyw43_arch_gpio_get(0));
			ulReceivedValue = 0;
		}
	}
}

static void prvQueueSendTask( void *pvParameters ) {
	TickType_t xNextWakeTime;
	const unsigned long ulValueToSend = 100UL;

	xNextWakeTime = xTaskGetTickCount();

	while(1) {
		vTaskDelayUntil(&xNextWakeTime, 250);
		xQueueSend(xQueue, &ulValueToSend, 0U);
	}
}
