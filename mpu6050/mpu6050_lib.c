#include "mpu6050_lib.h"


#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C

#define ACCEL_XOUT_H_REG 0x3B // register to get acceleration values from mpu6050
#define GYRO_XOUT_H_REG 0x43 // register to get angular velocity values from mpu6050
#define MPU6050_ADR (0x68 << 1)


bool mpu6050_init(I2C_HandleTypeDef* hi2c1){
	uint8_t data;

	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c1, MPU6050_ADR, WHO_AM_I_REG, 1, &data, 1, 1000);
	if (status != HAL_OK)
		return false;
	if (data == 0x68){
		// setup power management
		// in order to wake the sensor up and set it up to work in ordinary mode
		data = 0x00;
		status = HAL_I2C_Mem_Write(hi2c1, MPU6050_ADR, PWR_MGMT_1_REG, 1, &data, 1, 1000);
		if (status != HAL_OK)
			return false;
		// setup  Sample Rate Divider
		// This register specifies the divider from the gyroscope output rate used to generate the Sample Rate for the MPU-6050.
		// Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
		// Gyroscope Output Rate = 8kHz
		// Set DATA RATE of 1KHz -> Sample Rate = 8 / (1 + 7)
		data = 0x07;
		status = HAL_I2C_Mem_Write(hi2c1, MPU6050_ADR, SMPRT_DIV_REG, 1, &data, 1, 1000);
		if (status != HAL_OK)
			return false;

		// setup the range of the gyroscope
		// angular velocity ± 2000 °/s
		data = 0x18;
		status = HAL_I2C_Mem_Write(hi2c1, MPU6050_ADR, GYRO_CONFIG_REG, 1, &data, 1, 1000);
		if (status != HAL_OK)
			return false;
		// setup the range of the accelerometer
		// acceleration ± 16g
		data = 0x18;
		status = HAL_I2C_Mem_Write(hi2c1, MPU6050_ADR, ACCEL_CONFIG_REG, 1, &data, 1, 1000);
		if (status != HAL_OK)
			return false;

		return true;
	}
	else{
		return false;
	}

}

void mpu6050_getData(I2C_HandleTypeDef* hi2c1, mpu6050_Values* data){
	// array for serving raw acceleration and gyroscope data
	int16_t raw_data[6];
	// getting acceleration raw data
	HAL_I2C_Mem_Read (hi2c1, MPU6050_ADR, ACCEL_XOUT_H_REG, 1, raw_data, 6, 1000);
	int16_t raw_Ax = (raw_data[0] << 8) | raw_data[1];
	int16_t raw_Ay = (raw_data[2] << 8) | raw_data[3];
	int16_t raw_Az = (raw_data[4] << 8) | raw_data[5];

	// getting gyroscope data
	HAL_I2C_Mem_Read (hi2c1, MPU6050_ADR, GYRO_XOUT_H_REG, 1, raw_data, 6, 1000);
	int16_t raw_Wx = (raw_data[0] << 8) | raw_data[1];
	int16_t raw_Wy = (raw_data[2] << 8) | raw_data[3];
	int16_t raw_Wz = (raw_data[4] << 8) | raw_data[5];

	// getting float acceleration value with LSB per g = 2048 LSB/g
	data->Ax = raw_Ax / 2048.0;
	data->Ay = raw_Ay / 2048.0;
	data->Az = raw_Az / 2048.0;

	// getting float gyroscope value with LSB per °s = 16.4 LSB°/s
	data->Wx = raw_Wx / 16.4;
	data->Wy = raw_Wx / 16.4;
	data->Wz = raw_Wx / 16.4;

}




