#ifndef _TASK_H
#define _TASK_H

//�سȺ��

OS_STK USART_RECEIVE[100];//ʧ����������ջ

OS_STK USART_test[100];
OS_STK	GY86[512];
OS_STK	PID[512];
OS_STK	BT[512];
OS_STK	PoseCalcu[512];

void Task_USART_test(void);
void Task_PID(void);
void Task_GY86(void);
void Task_BT(void);
void ANODT_Send01(int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z, uint8_t stat);
void ANODT_Send02(int16_t mag_x, int16_t mag_y, int16_t mag_z, int32_t ALT_BAR, int16_t TMP, uint8_t BAR_STA, uint8_t MAG_STA);
void ANODT_Send03(double roll, double pitch, double yaw, uint8_t Fusion_stat);
void ANODT_Send04(float q0, float q1, float q2, float q3, uint8_t Fusion_stat);
void ANODT_Send20(uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4);
void Task_PoseCalcu(void);

#endif
