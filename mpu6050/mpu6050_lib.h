#include "stm32f1xx_hal.h"
#include "stdbool.h"

typedef struct {
	float Ax, Ay, Az;
	// W - angular velocity
	float Wx, Wy, Wz;

} mpu6050_Values;

bool mpu6050_init(I2C_HandleTypeDef* hi2c1);
void mpu6050_getData(I2C_HandleTypeDef* hi2c1, mpu6050_Values* s);
void mpu6050_calibrate(I2C_HandleTypeDef* hi2c1);
