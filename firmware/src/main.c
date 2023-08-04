#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"

#include "btstack.h"
#include "btstack_event.h"
#include "btstack_run_loop.h"

#include "bt.h"
#include "mpu.h"

void hardware_init(void);


int main() {
	hardware_init();
	mpu_init(MPU_ACCEL_4G, MPU_GYRO_500_DEG);
	btstack_main(0, NULL);
	btstack_run_loop_execute();

	return 0;
}

void hardware_init(void) {
	stdio_init_all();
    cyw43_arch_init();

	// i2c setup for imu
	i2c_init(i2c0, 100*1000);
	gpio_set_function(0, GPIO_FUNC_I2C);
	gpio_set_function(1, GPIO_FUNC_I2C);
	gpio_pull_up(0);
	gpio_pull_up(1);
}

