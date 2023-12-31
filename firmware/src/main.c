#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

#include "btstack.h"
#include "btstack_event.h"
#include "btstack_run_loop.h"

#include "bt.h"
#include "mpu.h"
#include "control.h"

void hardware_init(void);

// TODO: start with these pid values and tune as needed
pid_gain_t pitch_cfg = {
	.Kp = 1.2,
	.Ki = 0.02,
	.Kd = 15,
};

pid_gain_t roll_cfg = {
	.Kp = 1.2,
	.Ki = 0.02,
	.Kd = 15,
};

pid_gain_t yaw_cfg = {
	.Kp = 1.2,
	.Ki = 0.02,
	.Kd = 0,
};

int main() {
	hardware_init();
	printf("starting\n");
	mpu_init(MPU_ACCEL_4G, MPU_GYRO_500_DEG);
	pid_init(&pitch_cfg, &roll_cfg, &yaw_cfg);
	motors_init();
	printf("hello world\n");
	btstack_main(0, NULL);
	btstack_run_loop_execute();

	return 0;
}

void hardware_init(void) {
	stdio_init_all();
    cyw43_arch_init();

	// i2c setup for imu
	i2c_init(MPU_I2C_PORT, 100*1000);
	gpio_set_function(MPU_SDC_PIN, GPIO_FUNC_I2C);
	gpio_set_function(MPU_SDA_PIN, GPIO_FUNC_I2C);
	gpio_pull_up(MPU_SDC_PIN);
	gpio_pull_up(MPU_SDA_PIN);

	// pwm setup
    gpio_set_function(MOTOR_1_GPIO, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_2_GPIO, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_3_GPIO, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_4_GPIO, GPIO_FUNC_PWM);
}

