#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "btstack.h"
#include "bt.h"

#define TEST_TASK_PRIORITY				( tskIDLE_PRIORITY + 2UL )

static QueueHandle_t data_queue_handle;

void bt_task(void *params) {
    if (bt_init(data_queue_handle)){
        return;
    }

    btstack_main(0, NULL);
    btstack_run_loop_execute();

    while(1) {
        vTaskDelay(1000);
		printf("hello, world");
    }
}
