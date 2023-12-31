#include "hardware/pwm.h"
#include "control.h"
#include "mpu.h"

#define MOTOR_VAL_MAX 255
#define PWM_MAX (UINT16_MAX/2)
#define IERR_MAX (UINT16_MAX / 2)

typedef struct {
	float p_err, r_err, y_err;
} pid_err_t;

static pid_gain_t P_gain = {0};
static pid_gain_t R_gain = {0};
static pid_gain_t Y_gain = {0};

static pid_err_t P_errors = {0};
static pid_err_t I_errors = {0};
static pid_err_t D_errors = {0};

static float clamp(float min, float max, float num) {
	if (num < min) {
		return min;
	} else if (num > max) {
		return max;
	} else {
		return num;
	}
}

static mpu_angle_t PID_controller = {0};

typedef struct {
	uint8_t slice, chan;
	uint16_t val;
} motor_t;

static struct { motor_t m1, m2, m3, m4; } motors = {
	.m1 = {0},
	.m2 = {0},
	.m3 = {0},
	.m4 = {0},
};

static void calc_errors(pid_err_t* errors) {
	I_errors.p_err += P_gain.Ki * errors->p_err;
	I_errors.r_err += R_gain.Ki * errors->r_err;
	I_errors.y_err += Y_gain.Ki * errors->y_err;

	I_errors.p_err = clamp(0, IERR_MAX, I_errors.p_err);
	I_errors.r_err = clamp(0, IERR_MAX, I_errors.r_err);
	I_errors.y_err = clamp(0, IERR_MAX, I_errors.y_err);

	D_errors.p_err = errors->p_err - P_errors.p_err;
	D_errors.r_err = errors->r_err - P_errors.r_err;
	D_errors.y_err = errors->y_err - P_errors.y_err;

	P_errors.p_err = errors->p_err;
	P_errors.r_err = errors->r_err;
	P_errors.y_err = errors->y_err;
}

static void calc_pid(void) {
	PID_controller.pitch = (P_errors.p_err * P_gain.Kp)
					   + (I_errors.p_err)
					   + (D_errors.p_err * P_gain.Kd);
	PID_controller.roll = (P_errors.r_err * R_gain.Kp)
					   + (I_errors.r_err)
					   + (D_errors.r_err * R_gain.Kd);
	PID_controller.yaw = (P_errors.y_err * Y_gain.Kp)
					   + (I_errors.y_err)
					   + (D_errors.y_err * Y_gain.Kd);
}

static void calc_pwm(int16_t throttle) {
	// 	  1-----2
	//       |
	//       |
	//       |
	//    4-----3
	//
	//    pitch: +backward (x axis)
	//    roll: +right (y axis)
	//    yaw: +left (z axis)
	
	float val;

	val = throttle + PID_controller.pitch + PID_controller.roll + PID_controller.yaw;
	motors.m1.val = (uint16_t)(PWM_MAX * clamp(0, MOTOR_VAL_MAX, val) / 255);

	val = throttle + PID_controller.pitch - PID_controller.roll - PID_controller.yaw;
	motors.m2.val = (uint16_t)(PWM_MAX * clamp(0, MOTOR_VAL_MAX, val) / 255);

	val = throttle - PID_controller.pitch - PID_controller.roll + PID_controller.yaw;
	motors.m3.val = (uint16_t)(PWM_MAX * clamp(0, MOTOR_VAL_MAX, val) / 255);

	val = throttle - PID_controller.pitch + PID_controller.roll - PID_controller.yaw;
	motors.m4.val = (uint16_t)(PWM_MAX * clamp(0, MOTOR_VAL_MAX, val) / 255);
}

static void set_pwm(void) {
	pwm_set_chan_level(motors.m1.slice, motors.m1.chan, motors.m1.val);
	pwm_set_chan_level(motors.m2.slice, motors.m2.chan, motors.m2.val);
	pwm_set_chan_level(motors.m3.slice, motors.m3.chan, motors.m3.val);
	pwm_set_chan_level(motors.m4.slice, motors.m4.chan, motors.m4.val);
}

void pid_init(pid_gain_t* p_gain, pid_gain_t* r_gain, pid_gain_t* y_gain) {
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

void pid_step(float throttle, mpu_angle_t* tgt, mpu_angle_t* current) {
	pid_err_t errors = {0};
	errors.p_err = current->pitch - tgt->pitch;
	errors.r_err = current->roll - tgt->roll;
	errors.y_err = current->yaw - tgt->yaw;

	calc_errors(&errors);
	calc_pid();
	calc_pwm(throttle);
	set_pwm();
}

void motors_init(void) {
	motors.m1.slice = pwm_gpio_to_slice_num(MOTOR_1_GPIO);
	motors.m1.chan = pwm_gpio_to_channel(MOTOR_1_GPIO);
	pwm_set_clkdiv(motors.m1.slice, 256.0);
	pwm_set_phase_correct(motors.m1.slice, true);
	pwm_set_wrap(motors.m1.slice, PWM_MAX);
	pwm_set_enabled(motors.m1.slice,true);

	motors.m2.slice = pwm_gpio_to_slice_num(MOTOR_2_GPIO);
	motors.m2.chan = pwm_gpio_to_channel(MOTOR_2_GPIO);
	pwm_set_clkdiv(motors.m2.slice, 256.0);
	pwm_set_phase_correct(motors.m2.slice, true);
	pwm_set_wrap(motors.m2.slice, PWM_MAX);
	pwm_set_enabled(motors.m2.slice,true);

	motors.m3.slice = pwm_gpio_to_slice_num(MOTOR_3_GPIO);
	motors.m3.chan = pwm_gpio_to_channel(MOTOR_3_GPIO);
	pwm_set_clkdiv(motors.m3.slice, 256.0);
	pwm_set_phase_correct(motors.m3.slice, true);
	pwm_set_wrap(motors.m3.slice, PWM_MAX);
	pwm_set_enabled(motors.m3.slice,true);

	motors.m4.slice = pwm_gpio_to_slice_num(MOTOR_4_GPIO);
	motors.m4.chan = pwm_gpio_to_channel(MOTOR_4_GPIO);
	pwm_set_clkdiv(motors.m4.slice, 256.0);
	pwm_set_phase_correct(motors.m4.slice, true);
	pwm_set_wrap(motors.m4.slice, PWM_MAX);
	pwm_set_enabled(motors.m4.slice,true);
}
