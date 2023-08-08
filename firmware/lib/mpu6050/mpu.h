#ifndef __MPU6050_H__
#define __MPU6050_H__

#include <inttypes.h>

typedef enum {
	MPU_GYRO_CFG = 0x1b,
	MPU_ACCEL_CFG = 0x1c,
	MPU_ACCEL_XOUT_H = 0x3b,
	MPU_GYRO_XOUT_H = 0x43,
	MPU_PWR_MGMT_1 = 0x6b,
} mpu_reg_t;

typedef enum {
	MPU_ACCEL_2G,
	MPU_ACCEL_4G,
	MPU_ACCEL_8G,
	MPU_ACCEL_16G,
} mpu_accel_range_t;

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} mpu_accel_t;

typedef enum {
	MPU_GYRO_250_DEG,
	MPU_GYRO_500_DEG,
	MPU_GYRO_1000_DEG,
	MPU_GYRO_2000_DEG,
} mpu_gyro_range_t;

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} mpu_gyro_t;

typedef struct {
	float pitch;
	float roll;
	float yaw;
} mpu_angle_t;

void mpu_init(mpu_accel_range_t accel, mpu_gyro_range_t gyro);
void mpu_update_accel(void);
void mpu_update_gyro(void);
void mpu_update_angles(mpu_angle_t* angle, uint32_t time);

#endif
