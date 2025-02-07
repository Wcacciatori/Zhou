#include "GY86.h" 
#include "Delay.h"


/*
	GY86的初始化，读数据
*/
void GY86_Init(void)
{
	MyIIC_Init();
	MPU6050_Init();
	HMC_Init();
}

/*
 *陀螺仪和加速度计的写寄存器，读寄存器，初始化，读ID，读数据
 */

void MPU_WriteReg(uint8_t RegAddress, uint8_t Data){
	MyIIC_Start();
	MyIIC_SendByte(MPU6050_Address);
	MyIIC_ReceiveAck();
	MyIIC_SendByte(RegAddress);
	MyIIC_ReceiveAck();
	MyIIC_SendByte(Data);
	MyIIC_ReceiveAck();
	MyIIC_Stop();
}

uint8_t MPU_ReadReg(uint8_t RegAddress){
	uint8_t Data;
	
	MyIIC_Start();
	MyIIC_SendByte(MPU6050_Address);
	MyIIC_ReceiveAck();
	MyIIC_SendByte(RegAddress);
	MyIIC_ReceiveAck();

	MyIIC_Start();
	MyIIC_SendByte(MPU6050_Address | 0x01);
	MyIIC_ReceiveAck();
	Data=MyIIC_ReceiveByte();
	MyIIC_SendAck (1);
	MyIIC_Stop();
	return Data ;
}


void MPU6050_Init(void){
	MyIIC_Init();
	//todo：加速度计频率要和任务   就是这里！！频率！！
	//todo：加速度计矫正还没改
	MPU_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
	MPU_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
	MPU_WriteReg(MPU6050_SMPLRT_DIV, 0x07);//采样率除的分频系数
	MPU_WriteReg(MPU6050_CONFIG, 0x06);//低通滤波器 保留明显好转
	MPU_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
	MPU_WriteReg(MPU6050_ACCEL_CONFIG, 0x04);// 6=三周均自检	，高通滤波4=0.63Hz 
	MPU_WriteReg(MPU6050_INT_PIN_CFG , 0x02);
	MPU_WriteReg(MPU6050_USER_CTRL , 0x00);
	
}

uint8_t MPU6050_GetID(void)
{
	return MPU_ReadReg(MPU6050_WHO_AM_I);
}

void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t DataH, DataL;
	
	DataH = MPU_ReadReg(MPU6050_ACCEL_XOUT_H);
	DataL = MPU_ReadReg(MPU6050_ACCEL_XOUT_L);
	*AccX = (DataH << 8) | DataL;
	
	DataH = MPU_ReadReg(MPU6050_ACCEL_YOUT_H);
	DataL = MPU_ReadReg(MPU6050_ACCEL_YOUT_L);
	*AccY = (DataH << 8) | DataL;
	
	DataH = MPU_ReadReg(MPU6050_ACCEL_ZOUT_H);
	DataL = MPU_ReadReg(MPU6050_ACCEL_ZOUT_L);
	*AccZ = (DataH << 8) | DataL;
	
	DataH = MPU_ReadReg(MPU6050_GYRO_XOUT_H);
	DataL = MPU_ReadReg(MPU6050_GYRO_XOUT_L);
	*GyroX = (DataH << 8) | DataL;
	
	DataH = MPU_ReadReg(MPU6050_GYRO_YOUT_H);
	DataL = MPU_ReadReg(MPU6050_GYRO_YOUT_L);
	*GyroY = (DataH << 8) | DataL;
	
	DataH = MPU_ReadReg(MPU6050_GYRO_ZOUT_H);
	DataL = MPU_ReadReg(MPU6050_GYRO_ZOUT_L);
	*GyroZ = (DataH << 8) | DataL;
}



/*
 *磁力计的写寄存器，读寄存器，初始化，读数据
 */

void HMC_WriteReg(uint8_t RegAddress , uint8_t Data)
{
	MyIIC_Start();
	MyIIC_SendByte(HMC_Address);
	MyIIC_ReceiveAck();
	MyIIC_SendByte(RegAddress);
	MyIIC_ReceiveAck();
	MyIIC_SendByte(Data);
	MyIIC_ReceiveAck();
	MyIIC_Stop();
}

uint8_t HMC_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	MyIIC_Start();
	MyIIC_SendByte(HMC_Address);
	MyIIC_ReceiveAck();
	MyIIC_SendByte(RegAddress);
	MyIIC_ReceiveAck();
	MyIIC_Start();
	MyIIC_SendByte(HMC_Address | 0x01);
	MyIIC_ReceiveAck();
	Data = MyIIC_ReceiveByte();
	MyIIC_SendAck(1);
	MyIIC_Stop();
	return Data;
}


void HMC_Init(void)
{
	//成都磁场强度：0.51297高斯。倾角48.31。 磁偏角为-2.40方向朝西
	HMC_WriteReg(HMC_CR_A, 0x78);//采样平均数=8,数据输出速率=7(不使用)
	HMC_WriteReg(HMC_CR_B, 0x20);//增益=1090counts/Gauss
	HMC_WriteReg(HMC_MODE, 0x00);
	
}

void HMC_GetData(int16_t *GA_X, int16_t *GA_Z, int16_t *GA_Y)
{
	uint8_t DataM, DataL;
	
	DataM = HMC_ReadReg(HMC_XOUT_M);
	DataL = HMC_ReadReg(HMC_XOUT_L);
	*GA_X = (DataM << 8) | DataL;
	
	DataM = HMC_ReadReg(HMC_ZOUT_M);
	DataL = HMC_ReadReg(HMC_ZOUT_L);
	*GA_Z = (DataM << 8) | DataL;
	
	DataM = HMC_ReadReg(HMC_YOUT_M);
	DataL = HMC_ReadReg(HMC_YOUT_L);
	*GA_Y = (DataM << 8) | DataL;
	
}
