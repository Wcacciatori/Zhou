#ifndef _TASK_H
#define _TASK_H

//×Ø³Èºì¿Õ

OS_STK USART_RECEIVE[100];//Ê§°ÜÑùÀýËùÓÃÕ»

OS_STK USART_test[100];
OS_STK	GY86[512];
OS_STK	Motor[512];
OS_STK	BT[512];
OS_STK	PoseCalcu[512];

void Task_USART_test(void);
void Task_Motor(void);
void Task_GY86(void);
void Task_BT(void);
void ANODT_Send01(int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z, uint8_t stat);
void ANODT_Send03(double roll, double pitch, double yaw, uint8_t Fusion_stat);
void ANODT_Send04(float q0, float q1, float q2, float q3, uint8_t Fusion_stat);
void Task_PoseCalcu(void);

#endif
