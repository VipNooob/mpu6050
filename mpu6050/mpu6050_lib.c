#include "mpu6050_lib.h"


#define EARTH_G (9.80665)
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C

#define ACCEL_XOUT_H_REG 0x3B // register to get acceleration values from mpu6050
#define GYRO_XOUT_H_REG 0x43 // register to get angular velocity values from mpu6050
#define MPU6050_ADR (0x68 << 1)

// OFFSETS_registers
#define GYRO_OFFSET_ADR 0x13 //
#define ACCEL_OFFSET_ADR 0x06


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

void mpu6050_getRawData(I2C_HandleTypeDef* hi2c1, mpu6050* data){
	// array for serving raw acceleration and gyroscope data
	uint8_t raw_data[6];
	// getting acceleration raw data
	HAL_I2C_Mem_Read (hi2c1, MPU6050_ADR, ACCEL_XOUT_H_REG, 1, raw_data, 6, 1000);
	data->raw_A[0] = (raw_data[0] << 8) | raw_data[1];
	data->raw_A[1] = (raw_data[2] << 8) | raw_data[3];
	data->raw_A[2] = (raw_data[4] << 8) | raw_data[5];

	// getting gyroscope data
	HAL_I2C_Mem_Read (hi2c1, MPU6050_ADR, GYRO_XOUT_H_REG, 1, raw_data, 6, 1000);
	data->raw_W[0] = (raw_data[0] << 8) | raw_data[1];
	data->raw_W[1] = (raw_data[2] << 8) | raw_data[3];
	data->raw_W[2] = (raw_data[4] << 8) | raw_data[5];

}


void mpu6050_getFloat(mpu6050* data){

	// getting float acceleration value with LSB per g = 2048 LSB/g
	data->Ax = data->raw_A[0] / 2048.0 * EARTH_G;
	data->Ay = data->raw_A[1] / 2048.0 * EARTH_G;
	data->Az = data->raw_A[2] / 2048.0 * EARTH_G;

	// getting float gyroscope value with LSB per °s = 16.4 LSB°/s
	data->Wx = data->raw_W[0] / 16.4;
	data->Wy = data->raw_W[1] / 16.4;
	data->Wz = data->raw_W[2] / 16.4;

}

void mpu6050_setGyroOffX(I2C_HandleTypeDef* hi2c1, long off_Wx, int16_t sensivity_range){
	// Offset should be inputed in the +-1000
	// But the values can be in different ranges
	// value = value + offset
	// offset should be converted in according to value range

	if (sensivity_range == 250){
		// 2^16 - values range
		// +- 250 DPS -> 1 dps = 2^16 / 500 = 131 LSB
		// inside offset register 1 dps = 32.8 LSB
		off_Wx = off_Wx / 4;
	}
	else if (sensivity_range == 500){
		off_Wx = off_Wx / 2;
	}
	else if (sensivity_range == 1000){
		off_Wx = off_Wx * 1;
	}
	else if (sensivity_range == 2000){
		off_Wx = off_Wx * 2;

	}
	unsigned char gyro_data[2];
	gyro_data[0] = (off_Wx >> 8) & 0xff;
	gyro_data[1] = (off_Wx) & 0xff;
	HAL_I2C_Mem_Write (hi2c1, MPU6050_ADR, GYRO_OFFSET_ADR, 1, gyro_data, 2, 1000);

}



void mpu6050_setGyroOffY(I2C_HandleTypeDef* hi2c1, long off_Wx, int16_t sensivity_range){
	// Offset should be inputed in the +-1000
	// But the values can be in different ranges
	// value = value + offset
	// offset should be converted in according to value range

	if (sensivity_range == 250){
		// 2^16 - values range
		// +- 250 DPS -> 1 dps = 2^16 / 500 = 131 LSB
		// inside offset register 1 dps = 32.8 LSB
		off_Wx = off_Wx / 4;
	}
	else if (sensivity_range == 500){
		off_Wx = off_Wx / 2;
	}
	else if (sensivity_range == 1000){
		off_Wx = off_Wx * 1;
	}
	else if (sensivity_range == 2000){
		off_Wx = off_Wx * 2;

	}
	unsigned char gyro_data[2];
	gyro_data[0] = (off_Wx >> 8) & 0xff;
	gyro_data[1] = (off_Wx) & 0xff;
	HAL_I2C_Mem_Write (hi2c1, MPU6050_ADR, GYRO_OFFSET_ADR + 2, 1, gyro_data, 2, 1000);

}

void mpu6050_setGyroOffZ(I2C_HandleTypeDef* hi2c1, long off_Wx, int16_t sensivity_range){
	// Offset should be inputed in the +-1000
	// But the values can be in different ranges
	// value = value + offset
	// offset should be converted in according to value range

	if (sensivity_range == 250){
		// 2^16 - values range
		// +- 250 DPS -> 1 dps = 2^16 / 500 = 131 LSB
		// inside offset register 1 dps = 32.8 LSB
		off_Wx = off_Wx / 4;
	}
	else if (sensivity_range == 500){
		off_Wx = off_Wx / 2;
	}
	else if (sensivity_range == 1000){
		off_Wx = off_Wx * 1;
	}
	else if (sensivity_range == 2000){
		off_Wx = off_Wx * 2;

	}
	unsigned char gyro_data[2];
	gyro_data[0] = (off_Wx >> 8) & 0xff;
	gyro_data[1] = (off_Wx) & 0xff;
	HAL_I2C_Mem_Write (hi2c1, MPU6050_ADR, GYRO_OFFSET_ADR + 4, 1, gyro_data, 2, 1000);

}


void mpu6500_readAccelOffsets(I2C_HandleTypeDef* hi2c1, long* accel_regOffsets){
	unsigned char data[6];

	HAL_I2C_Mem_Read (hi2c1, MPU6050_ADR, ACCEL_OFFSET_ADR, 1, data, 2, 1000);
	HAL_I2C_Mem_Read (hi2c1, MPU6050_ADR, ACCEL_OFFSET_ADR + 2, 1, data + 2, 2, 1000);
	HAL_I2C_Mem_Read (hi2c1, MPU6050_ADR, ACCEL_OFFSET_ADR + 4, 1, data + 4, 2, 1000);
	accel_regOffsets[0] =  ((long)data[0] << 8) | data[1];
	accel_regOffsets[1] =  ((long)data[2] << 8) | data[3];
	accel_regOffsets[2] =  ((long)data[4] << 8) | data[5];
}


void mpu6050_setAccelOffX(I2C_HandleTypeDef* hi2c1, long off_Ax, int16_t sensivity_range){
	// values = values + offset(8g)
	if (sensivity_range == 2){
		off_Ax /= 8;
	}
	else if (sensivity_range == 4){
		off_Ax /= 4;
	}
	else if(sensivity_range == 8){
		off_Ax /= 2;
	}
	else if (sensivity_range == 16){
		off_Ax *= 1;
		off_Ax /= 1.156;

	}


	unsigned char data[2] = {0, 0};
	long accel_reg_bias = 0;
	long mask = 0x0001;
	unsigned char mask_bit = 0;

	// array to get bias from a register
	unsigned char temp_data[2];
	HAL_I2C_Mem_Read (hi2c1, MPU6050_ADR, ACCEL_OFFSET_ADR, 1, temp_data, 2, 1000);
	accel_reg_bias =  ((long)temp_data[0] << 8) | temp_data[1];


	if(accel_reg_bias & mask)
		mask_bit = 0x01;



	accel_reg_bias -= off_Ax;

	data[0] = (accel_reg_bias >> 8) & 0xff;
	data[1] = (accel_reg_bias) & 0xff;
	data[1] = data[1] | mask_bit;

	HAL_I2C_Mem_Write (hi2c1, MPU6050_ADR, ACCEL_OFFSET_ADR, 1, data, 2, 1000);


}

void mpu6050_setAccelOffY(I2C_HandleTypeDef* hi2c1, long off_Ay, int16_t sensivity_range){
	// values = values + offset(8g)
	if (sensivity_range == 2){
		off_Ay /= 8;
	}
	else if (sensivity_range == 4){
		off_Ay /= 4;
	}
	else if(sensivity_range == 8){
		off_Ay /= 2;
	}
	else if (sensivity_range == 16){
		off_Ay *= 1;
		off_Ay /= 1.156;

	}


	unsigned char data[2] = {0, 0};
	long accel_reg_bias = 0;
	long mask = 0x0001;
	unsigned char mask_bit = 0;

	// array to get bias from a register
	unsigned char temp_data[2];
	HAL_I2C_Mem_Read (hi2c1, MPU6050_ADR, ACCEL_OFFSET_ADR + 2, 1, temp_data, 2, 1000);
	accel_reg_bias =  ((long)temp_data[0] << 8) | temp_data[1];


	if(accel_reg_bias & mask)
		mask_bit = 0x01;



	accel_reg_bias -= off_Ay;

	data[0] = (accel_reg_bias >> 8) & 0xff;
	data[1] = (accel_reg_bias) & 0xff;
	data[1] = data[1] | mask_bit;

	HAL_I2C_Mem_Write (hi2c1, MPU6050_ADR, ACCEL_OFFSET_ADR + 2, 1, data, 2, 1000);


}

void mpu6050_setAccelOffZ(I2C_HandleTypeDef* hi2c1, long off_Az, int16_t sensivity_range){
	// values = values + offset(8g)
	if (sensivity_range == 2){
		off_Az /= 8;
	}
	else if (sensivity_range == 4){
		off_Az /= 4;
	}
	else if(sensivity_range == 8){
		off_Az /= 2;
	}
	else if (sensivity_range == 16){
		off_Az *= 1;
		off_Az /= 1.17;

	}


	unsigned char data[2] = {0, 0};
	long accel_reg_bias = 0;
	long mask = 0x0001;
	unsigned char mask_bit = 0;

	// array to get bias from a register
	unsigned char temp_data[2];
	HAL_I2C_Mem_Read (hi2c1, MPU6050_ADR, ACCEL_OFFSET_ADR + 4, 1, temp_data, 2, 1000);
	accel_reg_bias =  ((long)temp_data[0] << 8) | temp_data[1];


	if(accel_reg_bias & mask)
		mask_bit = 0x01;



	accel_reg_bias -= off_Az;

	data[0] = (accel_reg_bias >> 8) & 0xff;
	data[1] = (accel_reg_bias) & 0xff;
	data[1] = data[1] | mask_bit;

	HAL_I2C_Mem_Write (hi2c1, MPU6050_ADR, ACCEL_OFFSET_ADR + 4, 1, data, 2, 1000);


}







