#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <stdint.h>
#include "mpu.h"

#define MOTOR_1_GPIO 1
#define MOTOR_2_GPIO 2
#define MOTOR_3_GPIO 3
#define MOTOR_4_GPIO 4

typedef struct {
	float Kp, Ki, Kd;
} pid_gain_t;

void pid_step(float throttle, mpu_angle_t* tgt, mpu_angle_t* current);
void pid_init(pid_gain_t* p_gain, pid_gain_t* r_gain, pid_gain_t* y_gain);
void parse_cmd(char* cmd);

void motors_init(void);

#endif
