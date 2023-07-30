#include <stdio.h>
#include "pico/stdlib.h"

int main(void) {
	stdio_init_all();

	while(1) {
		printf("hello, world!\n");
		sleep_ms(1000);
	}
}
