#ifndef __GY86_H
#define __GY86_H

#include "stm32f4xx.h"                  // Device header
#include "MyIIC.h" 

#define MPU6050_Address 0xD0
#define HMC_Address 0x3C


#define	MPU6050_SMPLRT_DIV		0x19
#define	MPU6050_CONFIG			0x1A
#define	MPU6050_GYRO_CONFIG		0x1B
#define	MPU6050_ACCEL_CONFIG	0x1C

#define MPU6050_INT_PIN_CFG		0x37

#define	MPU6050_ACCEL_XOUT_H	0x3B
#define	MPU6050_ACCEL_XOUT_L	0x3C
#define	MPU6050_ACCEL_YOUT_H	0x3D
#define	MPU6050_ACCEL_YOUT_L	0x3E
#define	MPU6050_ACCEL_ZOUT_H	0x3F
#define	MPU6050_ACCEL_ZOUT_L	0x40
#define	MPU6050_TEMP_OUT_H		0x41
#define	MPU6050_TEMP_OUT_L		0x42
#define	MPU6050_GYRO_XOUT_H		0x43
#define	MPU6050_GYRO_XOUT_L		0x44
#define	MPU6050_GYRO_YOUT_H		0x45
#define	MPU6050_GYRO_YOUT_L		0x46
#define	MPU6050_GYRO_ZOUT_H		0x47
#define	MPU6050_GYRO_ZOUT_L		0x48

#define MPU6050_USER_CTRL 		0x6A		

#define	MPU6050_PWR_MGMT_1		0x6B
#define	MPU6050_PWR_MGMT_2		0x6C
#define	MPU6050_WHO_AM_I		0x75


#define HMC_CR_A				0x00
#define HMC_CR_B				0x01
#define HMC_MODE				0x02

#define HMC_XOUT_M				0x03
#define HMC_XOUT_L				0x04
#define HMC_ZOUT_M				0x05
#define HMC_ZOUT_L				0x06
#define HMC_YOUT_M				0x07
#define HMC_YOUT_L				0x08

void GY86_Init(void);

void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);

void HMC_Init(void);
void HMC_GetData(int16_t *GA_X, int16_t *GA_Z, int16_t *GA_Y);

#endif
