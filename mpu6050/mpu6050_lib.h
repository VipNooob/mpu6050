#include "stm32f0xx_hal.h"
#include "stdbool.h"


#ifndef mpu6050_lib
#define mpu6050_lib
typedef struct {
	int16_t raw_A[3];
	int16_t raw_W[3];
	float Ax, Ay, Az;
	// W - angular velocity
	float Wx, Wy, Wz;

} mpu6050;

bool mpu6050_init(I2C_HandleTypeDef* hi2c1);
void mpu6050_getRawData(I2C_HandleTypeDef* hi2c1, mpu6050* data);
void mpu6050_getFloat(mpu6050* data);

// functions that need in order to set a required offset into registers of MPU6050
// GYRO
void mpu6050_setGyroOffX(I2C_HandleTypeDef* hi2c1, long off_Wx, int16_t sensivity_range);
void mpu6050_setGyroOffY(I2C_HandleTypeDef* hi2c1, long off_Wx, int16_t sensivity_range);
void mpu6050_setGyroOffZ(I2C_HandleTypeDef* hi2c1, long off_Wx, int16_t sensivity_range);
// ACCELERATION
void mpu6500_readAccelOffsets(I2C_HandleTypeDef* hi2c1, long* accel_regOffsets);
void mpu6050_setAccelOffX(I2C_HandleTypeDef* hi2c1, long off_Ax, int16_t sensivity_range);
void mpu6050_setAccelOffY(I2C_HandleTypeDef* hi2c1, long off_Ay, int16_t sensivity_range);
void mpu6050_setAccelOffZ(I2C_HandleTypeDef* hi2c1, long off_Az, int16_t sensivity_range);


#endif
