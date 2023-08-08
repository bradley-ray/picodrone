#include "hardware/pwm.h"
#include "control.h"
#include "mpu.h"

typedef struct {
	int Kp, Ki, Kd;
} gain_t;

typedef struct {
	int p_err, r_err, y_err;
} pid_err_t;

static gain_t P_gain = {0};
static gain_t R_gain = {0};
static gain_t Y_gain = {0};

static pid_err_t P_errors = {0};
static pid_err_t I_errors = {0};
static pid_err_t D_errors = {0};

static struct {int P, R, Y;} PID_controller = {0};

static struct { uint32_t m1, m2, m3, m4; } motors = {0};

static mpu_angle_t current_angles = {0};

static uint8_t motor_1_slice_num, motor_2_slice_num, motor_3_slice_num, motor_4_slice_num;
static uint8_t motor_1_chan, motor_2_chan, motor_3_chan, motor_4_chan;

static void calc_errors(int error_pitch, int error_roll, int error_yaw) {
	I_errors.p_err += error_pitch;
	I_errors.r_err += error_roll;
	I_errors.y_err += error_yaw;

	D_errors.p_err = error_pitch - P_errors.p_err;
	D_errors.r_err = error_roll - P_errors.r_err;
	D_errors.y_err = error_yaw - P_errors.y_err;

	P_errors.p_err = error_pitch;
	P_errors.r_err = error_roll;
	P_errors.y_err = error_yaw;
}

static void calc_pid(void) {
	PID_controller.P = (P_errors.p_err * P_gain.Kp)
					   + (I_errors.p_err * P_gain.Ki)
					   + (D_errors.p_err * P_gain.Kd);
	PID_controller.R = (P_errors.r_err * R_gain.Kp)
					   + (I_errors.r_err * R_gain.Ki)
					   + (D_errors.r_err * R_gain.Kd);
	PID_controller.Y = (P_errors.y_err * Y_gain.Kp)
					   + (I_errors.y_err * Y_gain.Ki)
					   + (D_errors.y_err * Y_gain.Kd);
}

static void calc_pwm(int throttle) {
	// 	  1-----2
	//       |
	//       |
	//       |
	//    4-----3
	
	motors.m1 = throttle + PID_controller.P + PID_controller.R + PID_controller.Y;
	motors.m2 = throttle + PID_controller.P - PID_controller.R - PID_controller.Y;
	motors.m3 = throttle - PID_controller.P - PID_controller.R + PID_controller.Y;
	motors.m4 = throttle - PID_controller.P + PID_controller.R - PID_controller.Y;
}

static void set_pwm(void) {
}

void pid_init(gain_t* p_gain, gain_t* r_gain, gain_t* y_gain) {
	P_gain.Kp = p_gain->Kp;
	P_gain.Ki = p_gain->Ki;
	P_gain.Kd = p_gain->Kd;

	R_gain.Kp = r_gain->Kp;
	R_gain.Ki = r_gain->Ki;
	R_gain.Kd = r_gain->Kd;

	Y_gain.Kp = y_gain->Kp;
	Y_gain.Ki = y_gain->Ki;
	Y_gain.Kd = y_gain->Kd;
}

void pid_step(int16_t tcmd, int16_t pcmd, int16_t rcmd, int16_t ycmd, mpu_angle_t* tgt) {
	int error_pitch = current_angles.pitch - tgt->pitch;
	int error_roll = current_angles.roll - tgt->roll;
	int error_yaw = current_angles.yaw - tgt->yaw;

	calc_errors(error_pitch, error_roll, error_yaw);
	calc_pid();
	calc_pwm(tcmd);
	set_pwm();
}

void motors_init(void) {
	motor_1_slice_num = pwm_gpio_to_slice_num(MOTOR_1_GPIO);
	motor_1_chan = pwm_gpio_to_channel(MOTOR_1_GPIO);
	pwm_set_clkdiv(motor_1_slice_num, 256.0);
	pwm_set_phase_correct(motor_1_slice_num, true);
	pwm_set_wrap(motor_1_slice_num, UINT16_MAX);
	pwm_set_enabled(motor_1_slice_num,true);

	motor_2_slice_num = pwm_gpio_to_slice_num(MOTOR_2_GPIO);
	motor_2_chan = pwm_gpio_to_channel(MOTOR_1_GPIO);
	pwm_set_clkdiv(motor_2_slice_num, 256.0);
	pwm_set_phase_correct(motor_2_slice_num, true);
	pwm_set_wrap(motor_2_slice_num, UINT16_MAX);
	pwm_set_enabled(motor_2_slice_num,true);

	motor_3_slice_num = pwm_gpio_to_slice_num(MOTOR_3_GPIO);
	motor_3_chan = pwm_gpio_to_channel(MOTOR_1_GPIO);
	pwm_set_clkdiv(motor_3_slice_num, 256.0);
	pwm_set_phase_correct(motor_3_slice_num, true);
	pwm_set_wrap(motor_3_slice_num, UINT16_MAX);
	pwm_set_enabled(motor_3_slice_num,true);

	motor_4_slice_num = pwm_gpio_to_slice_num(MOTOR_4_GPIO);
	motor_4_chan = pwm_gpio_to_channel(MOTOR_1_GPIO);
	pwm_set_clkdiv(motor_4_slice_num, 256.0);
	pwm_set_phase_correct(motor_4_slice_num, true);
	pwm_set_wrap(motor_4_slice_num, UINT16_MAX);
	pwm_set_enabled(motor_4_slice_num,true);
}
