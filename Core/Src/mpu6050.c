/*
 * mpu6050.c
 *
 *  Created on: Oct 27, 2025
 *      Author: hdani
 */
#include "mpu6050.h"

//CHANGE TO MACROS
uint8_t gyroAddr = 0x68;
uint8_t accel_regH = 0x3B;
uint8_t gyro_regH = 0x43;
uint8_t gyroPwr_reg = 0x6B;

uint8_t gyro_tx = 0;

uint8_t accel_buf[6] = {0};
uint8_t gyro_buf[6] = {0};

int16_t accelX_val = 0;
int16_t accelY_val = 0;
int16_t accelZ_val = 0;

int16_t gyroX_val = 0;
int16_t gyroY_val = 0;
int16_t gyroZ_val = 0;

void mpu6050_init(void){
	//HAL_I2C_Mem_Write(&hi2c1, gyroAddr << 1, gyroPwr_reg, I2C_MEMADD_SIZE_8BIT, &gyro_tx, 1, HAL_MAX_DELAY);
}

void mpu6050_readall(void){
	//HAL_I2C_Mem_Read(&hi2c1, gyroAddr << 1, accel_regH, I2C_MEMADD_SIZE_8BIT, accel_buf, 6, HAL_MAX_DELAY);
	accelX_val = (int16_t)(accel_buf[0] << 8 | accel_buf[1]);
	accelY_val = (int16_t)(accel_buf[2] << 8 | accel_buf[3]);
	accelZ_val = (int16_t)(accel_buf[4] << 8 | accel_buf[5]);

	//HAL_I2C_Mem_Read(&hi2c1, gyroAddr << 1, gyro_regH, I2C_MEMADD_SIZE_8BIT, gyro_buf, 6, HAL_MAX_DELAY);
	gyroX_val = (int16_t)(gyro_buf[0] << 8 | gyro_buf[1]);
	gyroY_val = (int16_t)(gyro_buf[2] << 8 | gyro_buf[3]);
	gyroZ_val = (int16_t)(gyro_buf[4] << 8 | gyro_buf[5]);

}
