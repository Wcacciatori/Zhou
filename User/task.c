#include "main.h"
#include "Delay.h"

extern uint32_t Duty[4];
int16_t  AX, AY, AZ, GX, GY, GZ, MX, MZ, MY;

uint16_t pwm_IN[4];


OS_ERR err;
OS_CPU_SR  cpu_sr = 0u;

uint8_t test;

void Task_USART_test(){
	while(1)
	{
	OS_ENTER_CRITICAL();
	Serial_Printf("www\r\n");
	OS_EXIT_CRITICAL();
	OSTimeDly(10);
	}
}



void Task_Motor()
{
	while(1){
	OS_ENTER_CRITICAL();
	PWM_SetDuty3(pwm_IN[2]);
	PWM_SetDuty2(pwm_IN[2]);
  PWM_SetDuty1(pwm_IN[2]);
  PWM_SetDuty4(pwm_IN[2]);
		//Serial_Printf("www");
	OS_EXIT_CRITICAL();
	OSTimeDly(5);
	}
}


void Task_GY86()
{
	while(1){

	OS_ENTER_CRITICAL();
			MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
			HMC_GetData(&MX, &MZ, &MY);
	OS_EXIT_CRITICAL();
			OS_CPU_SysTickInitFreq(84000000);		
	OSTimeDly(5);
	}
}

void Task_BT(){
	while(1){
	OS_ENTER_CRITICAL();	
		Serial_Printf("ch1 = %d\r\r\r\r\r\r\r\r\r\r\r\r\r", pwm_IN[0]);//1 右 左右
		Serial_Printf("ch2 = %d\r\r\r\r\r\r\r\r\r\r\r\r\r", pwm_IN[1]);//2 右 上下
		Serial_Printf("ch3 = %d\r\r\r\r\r\r\r\r\r\r\r\r\r", pwm_IN[2]);//3 左 上下
		Serial_Printf("ch4 = %d\r\r\r\r\r\r\r\r\r\r\r\r\r", pwm_IN[3]);//4 左 左右
			
		Serial_Printf("accX:%d\r\r\r\r\r\r\r\r\r\r\r\r\r\r",AX);			
		Serial_Printf("accY:%d\r\r\r\r\r\r\r\r\r\r\r\r",AY);
		Serial_Printf("accZ:%d\r\r\r\r\r\r\r\r\r\r\r",AZ);

		Serial_Printf("gyroX:%d\r\r\r\r\r\r\r\r\r\r\r",GX);
		Serial_Printf("gyroY:%d\r\r\r\r\r\r\r\r\r\r\r",GY);
		Serial_Printf("gyroZ:%d\r\r\r\r\r\r\r\r\r\r\r",GZ);
				
		Serial_Printf("magX:%d\r\r\r\r\r\r\r\r\r\r\r",MX);
		Serial_Printf("magY:%d\r\r\r\r\r\r\r\r\r\r\r",MY);
		Serial_Printf("magZ:%d\r\r\r\r\r\r\r\r\r\r\r",MZ);
		Serial_Printf("\r\n");
	OS_EXIT_CRITICAL();
	OSTimeDly(10);
}
};








//失败（错误）的样例
void Task_USART_RECEIVE(){
	
	//创建信号量
	//Sem_Task_UR = OSSemCreate(0);
	
	while(1)
	{
		//接收机结数据
		
		//接受信号量
//		OSSemPend(Sem_Task_UR, 0, &err);
//		Serial_Printf("yyy");
			
		//接收数据
		test=Serial_GetRxData();
		Serial_SendByte(test);
		//OSSemSet(Sem_Task_UR, 0, &err);

	}
}
