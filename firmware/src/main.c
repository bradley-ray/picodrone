#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "btstack.h"
#include "btstack_event.h"
#include "btstack_run_loop.h"

#include "bt.h"

void hardware_init(void);

int main() {
	hardware_init();
	bt_init();

	btstack_main(0, NULL);
	btstack_run_loop_execute();
}

void hardware_init(void) {
	stdio_init_all();
    cyw43_arch_init();
}

