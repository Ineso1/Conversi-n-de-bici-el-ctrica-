#ifndef __accel_H
#define __accel_H

#include <stdint.h>
#include "usart.h"

#define ACC_MEASURE_PERIOD					91			// 20 [ms] => 50Hz; 10 => 100Hz (91 oli enne seal)
#define	IMU_NUMBER_OF_BYTES					18			// Number of bytes to read from IMU register


extern uint8_t		imu_readings[IMU_NUMBER_OF_BYTES];


void BNO055_Init(void);
void BNO055_Init_I2C(I2C_HandleTypeDef* hi2c_device);
void readAccelData(int16_t * destination);
//void SendAccelData(USART_TypeDef* USARTx, uint8_t* str);
uint8_t GetAccelData(I2C_HandleTypeDef* hi2c_device, uint8_t* str);
uint8_t GetAccelChipId(I2C_HandleTypeDef* hi2c_device, uint8_t *chip_id);
uint8_t GetAccelTemp(I2C_HandleTypeDef* hi2c_device);
uint8_t BNO055_Get_Calibration(I2C_HandleTypeDef* hi2c_device);
void BNO055_Calc_Calibration(uint8_t calibration, uint8_t *cal_system, uint8_t *cal_gyro, uint8_t *cal_acc, uint8_t *cal_mag);
#endif

