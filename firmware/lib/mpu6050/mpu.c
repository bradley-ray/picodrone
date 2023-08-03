#include "hardware/i2c.h"

#include "mpu.h"
#include <stdio.h>

#define MPU_ADDR 0x68

static uint8_t calibrated = 0;
static mpu_gyro_t calib_gyro = {0};

static mpu_accel_t accel_vals = {0};
static mpu_gyro_t gyro_vals = {0};

static mpu_angle_t current_angle = {0};

static void val_to_int(uint8_t* buf, int16_t* res) {
	for(uint8_t i = 0; i < 3; ++i) {
		res[i] = buf[i*2] << 8 | buf[(i*2) + 1];
	}
}

static void mpu_reset() {
	uint8_t buf[] = {MPU_PWR_MGMT_1, 0};
	i2c_write_blocking(i2c0, MPU_ADDR, buf, 2, false);
}

static void mpu_accel_init(mpu_accel_range_t accel) {
	uint8_t buf[2];
	buf[0] = MPU_ACCEL_CFG;
	buf[1] = accel << 3;
	i2c_write_blocking(i2c0, MPU_ADDR, buf, 2, false);
}

static void mpu_gyro_init(mpu_gyro_range_t gyro) {
	uint8_t buf[2];
	buf[0] = MPU_GYRO_CFG;
	buf[1] = gyro << 3;
	i2c_write_blocking(i2c0, MPU_ADDR, buf, 2, false);
}

static void mpu_calibrate(void) {
	for(uint16_t i = 0; i < 2000; ++i) {
		mpu_update_gyro();
		calib_gyro.x += gyro_vals.x;
		calib_gyro.y += gyro_vals.y;
		calib_gyro.z += gyro_vals.z;

		sleep_ms(1);
	}

	calib_gyro.x /= 2000;
	calib_gyro.y /= 2000;
	calib_gyro.z /= 2000;

	calibrated = 1;
}

void mpu_init(mpu_accel_range_t accel, mpu_gyro_range_t gyro) {
	uint8_t buf[2];
	mpu_reset();
	mpu_accel_init(accel);
	mpu_gyro_init(gyro);
	mpu_calibrate();

	mpu_update_accel();
}


void mpu_update_accel(void) {
	uint8_t buf[6];
	uint8_t start = MPU_ACCEL_XOUT_H;
	i2c_write_blocking(i2c0, MPU_ADDR, &start, 1, true);
	i2c_read_blocking(i2c0, MPU_ADDR, buf, 6, false);
	val_to_int(buf, (int16_t*)&accel_vals);

	printf("ax: %d, ay: %d, az: %d\n", accel_vals.x, accel_vals.y, accel_vals.z);
}

void mpu_update_gyro(void) {
	uint8_t buf[6];
	uint8_t start = MPU_GYRO_XOUT_H;
	i2c_write_blocking(i2c0, MPU_ADDR, &start, 1, true);
	i2c_read_blocking(i2c0, MPU_ADDR, buf, 6, false);
	val_to_int(buf, (int16_t*)&gyro_vals);

	if (calibrated) {
		gyro_vals.x -= calib_gyro.x;
		gyro_vals.y -= calib_gyro.y;
		gyro_vals.z -= calib_gyro.z;
	}

	printf("gx: %d, gy: %d, gz: %d\n", gyro_vals.x, gyro_vals.y, gyro_vals.z);
}

void mpu_update_angles(void) {
	//current_angle.roll += gyro_vals.x;
	//current_angle.pitch += gyro_vals.y;
	//current_angle.yaw += gyro_vals.z;
}

mpu_angle_t* mpu_get_angles(void) {
	return &current_angle;
}
