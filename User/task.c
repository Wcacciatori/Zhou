#include "main.h"
#include "Delay.h"

#define BYTE0(dwTemp)	(*(char *)(&dwTemp))
#define BYTE1(dwTemp)	(*((char *)(&dwTemp)+1))
#define BYTE2(dwTemp)	(*((char *)(&dwTemp)+2))
#define BYTE3(dwTemp)	(*((char *)(&dwTemp)+3))

extern volatile float q0,q1,q2,q3;
extern float q0Acc,q1Acc,q2Acc,q3Acc;//初始值可设为1,0,0,0
extern float q0gyro,q1gyro,q2gyro,q3gyro;//初始值可设为1,0,0,0
extern double yaw,pitch,roll;
extern gyro gyroData;
extern acc accData;
extern uint32_t Duty[4];
int16_t  AX, AY, AZ, GX, GY, GZ, MX, MZ, MY;
float Axf,Ayf,Azf;
uint16_t pwm_IN[4];
uint8_t Sensor_data_buf[19];

void ANODT_Send01(int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z, uint8_t stat);
void ANODT_Send03(double roll, double pitch, double yaw, uint8_t Fusion_stat);
void ANODT_Send04(float q0, float q1, float q2, float q3, uint8_t Fusion_stat);

typedef struct{
	uint8_t head;
	uint8_t addr;
	uint8_t id;
	uint8_t len;
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	uint8_t shock_sta;
	uint8_t sc;//和校验
	uint8_t ac;//附加校验
}Sensor_data;

	
typedef struct cali{
	float Kx;
	float Ky;
	float Kz;
	float Bx;
	float By;
	float Bz;
}cali;

OS_ERR err;
OS_CPU_SR  cpu_sr = 0u;

uint8_t test;

const cali acc_cali={1.018636, 0.998253, 0.976484, 0.040125, -0.013710, -0.073240};

void Task_USART_test(){
	while(1)
	{
	OS_ENTER_CRITICAL();
	Serial_Printf("www\r\n");
	OS_EXIT_CRITICAL();
	OSTimeDly(10);
	}
}

void Task_PoseCalcu(){
	while(1)
	{
	OS_ENTER_CRITICAL();
//		qAccCompute(&q0Acc,&q1Acc,&q2Acc,&q3Acc);
//		updateQuaternion(&q0gyro,&q1gyro,&q2gyro,&q3gyro);
		madgwick(&q0, &q1, &q2, &q3);
		updateAngleTmp(&q0, &q1, &q2, &q3);
		//发送到上位机
			ANODT_Send01(AX,AY,AZ,GX,GY,GZ,0);
			ANODT_Send03(roll*100, pitch*100, yaw*100, 1);
			ANODT_Send04(q0,q1,q2,q3,1);
	OS_EXIT_CRITICAL();
	OSTimeDly(1);//1000HZ
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
			MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);//获取mpu6000原始数据
			//稍加处理
			Axf=(AX/16384.0-acc_cali.Bx)*acc_cali.Kx;
			Ayf=(AY/16384.0-acc_cali.By)*acc_cali.Ky;
			Azf=(AZ/16384.0-acc_cali.Bz)*acc_cali.Kz;
			AX = (int16_t)(Axf*100);
			AY = (int16_t)(Ayf*100);
			AZ = (int16_t)(Azf*100);
			GX = (int16_t)(GX/16.384);
			GY = (int16_t)(GY/16.384);
			GZ = (int16_t)(GZ/16.384);
			accData.x = Axf;
			accData.y = Ayf;
			accData.z = Azf;
			gyroData.x = (GX/16.384);
			gyroData.y = (GY/16.384);
			gyroData.z = (GZ/16.384);//+0.0555可以消除一点偏移
			//获取磁力计数据
			HMC_GetData(&MX, &MZ, &MY);
	OS_EXIT_CRITICAL();
			OS_CPU_SysTickInitFreq(84000000);		
	OSTimeDly(6);//500HZ
	}
}

void Task_BT(){
	while(1){
	OS_ENTER_CRITICAL();	
//		Serial_Printf("ch1 = %d\r\r\r\r\r\r\r\r\r\r\r\r\r", pwm_IN[0]);//1 右 左右
//		Serial_Printf("ch2 = %d\r\r\r\r\r\r\r\r\r\r\r\r\r", pwm_IN[1]);//2 右 上下
//		Serial_Printf("ch3 = %d\r\r\r\r\r\r\r\r\r\r\r\r\r", pwm_IN[2]);//3 左 上下
//		Serial_Printf("ch4 = %d\r\r\r\r\r\r\r\r\r\r\r\r\r", pwm_IN[3]);//4 左 左右
//		Serial_Printf();


	OS_EXIT_CRITICAL();
	OSTimeDly(5);//200hz
}
};

void ANODT_Send01(int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z, uint8_t stat){
	uint8_t cnt=0;
	
	Sensor_data_buf[cnt++] = 0xAA;
	Sensor_data_buf[cnt++] = 0xFF;
	Sensor_data_buf[cnt++] = 0x01;
	Sensor_data_buf[cnt++] = 13;
	Sensor_data_buf[cnt++] = BYTE0(acc_x);
	Sensor_data_buf[cnt++] = BYTE1(acc_x);
	Sensor_data_buf[cnt++] = BYTE0(acc_y);
	Sensor_data_buf[cnt++] = BYTE1(acc_y);
	Sensor_data_buf[cnt++] = BYTE0(acc_z);
	Sensor_data_buf[cnt++] = BYTE1(acc_z);
	Sensor_data_buf[cnt++] = BYTE0(gyro_x);
	Sensor_data_buf[cnt++] = BYTE1(gyro_x);
	Sensor_data_buf[cnt++] = BYTE0(gyro_y);
	Sensor_data_buf[cnt++] = BYTE1(gyro_y);
	Sensor_data_buf[cnt++] = BYTE0(gyro_z);
	Sensor_data_buf[cnt++] = BYTE1(gyro_z);
	Sensor_data_buf[cnt++] = stat;
	
	uint8_t sumcheck=0,addcheck=0;
	
	for(uint8_t i=0; i < (Sensor_data_buf[3]+4); i++)
	{
	sumcheck += Sensor_data_buf[i]; 
	addcheck += sumcheck; 
	}
	
	Sensor_data_buf[cnt++] = sumcheck;
	Sensor_data_buf[cnt++] = addcheck;
	
	Serial_SendArray(Sensor_data_buf,19);
	
}

void ANODT_Send03(double roll, double pitch, double yaw, uint8_t Fusion_stat){
	uint8_t cnt=0;
	int16_t roll_tosend, pitch_tosend, yaw_tosend;
	roll_tosend = roll*100;
	pitch_tosend = pitch*100;
	yaw_tosend = yaw*100;
	Sensor_data_buf[cnt++] = 0xAA;
	Sensor_data_buf[cnt++] = 0xFF;
	Sensor_data_buf[cnt++] = 0x03;
	Sensor_data_buf[cnt++] = 7;
	Sensor_data_buf[cnt++] = BYTE0(roll_tosend);
	Sensor_data_buf[cnt++] = BYTE1(roll_tosend);
	Sensor_data_buf[cnt++] = BYTE0(pitch_tosend);
	Sensor_data_buf[cnt++] = BYTE1(pitch_tosend);
	Sensor_data_buf[cnt++] = BYTE0(yaw_tosend);
	Sensor_data_buf[cnt++] = BYTE1(yaw_tosend);
	Sensor_data_buf[cnt++] = Fusion_stat;
	
	uint8_t sumcheck=0,addcheck=0;
	
	for(uint8_t i=0; i < (Sensor_data_buf[3]+4); i++)
	{
	sumcheck += Sensor_data_buf[i]; 
	addcheck += sumcheck; 
	}
	
	Sensor_data_buf[cnt++] = sumcheck;
	Sensor_data_buf[cnt++] = addcheck;
	
	Serial_SendArray(Sensor_data_buf,13);
	
}

void ANODT_Send04(float q0, float q1, float q2, float q3, uint8_t Fusion_stat){
	uint8_t cnt=0;
	int16_t q0_tosend, q1_tosend, q2_tosend, q3_tosend;
	q0_tosend = q0*10000;
	q1_tosend = q1*10000;
	q2_tosend = q2*10000;
	q3_tosend = q3*10000;
	Sensor_data_buf[cnt++] = 0xAA;
	Sensor_data_buf[cnt++] = 0xFF;
	Sensor_data_buf[cnt++] = 0x04;
	Sensor_data_buf[cnt++] = 9;
	Sensor_data_buf[cnt++] = BYTE0(q0_tosend);
	Sensor_data_buf[cnt++] = BYTE1(q0_tosend);
	Sensor_data_buf[cnt++] = BYTE0(q1_tosend);
	Sensor_data_buf[cnt++] = BYTE1(q1_tosend);
	Sensor_data_buf[cnt++] = BYTE0(q2_tosend);
	Sensor_data_buf[cnt++] = BYTE1(q2_tosend);
	Sensor_data_buf[cnt++] = BYTE0(q3_tosend);
	Sensor_data_buf[cnt++] = BYTE1(q3_tosend);
	Sensor_data_buf[cnt++] = Fusion_stat;
	
	uint8_t sumcheck=0,addcheck=0;
	
	for(uint8_t i=0; i < (Sensor_data_buf[3]+4); i++)
	{
	sumcheck += Sensor_data_buf[i]; 
	addcheck += sumcheck; 
	}
	
	Sensor_data_buf[cnt++] = sumcheck;
	Sensor_data_buf[cnt++] = addcheck;
	
	Serial_SendArray(Sensor_data_buf,15);
	
}


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


//			Serial_Printf("accX:%6f\r",Axf);			
//		  Serial_Printf("accY:%6f\r",Ayf);
//		  Serial_Printf("accZ:%6f\r",Azf);

//			Serial_Printf("accX:%6d\r\r\r\r\r\r\r\r\r",AX);			
//		  Serial_Printf("accY:%6d\r\r\r\r\r\r\r\r\r",AY);
//		  Serial_Printf("accZ:%6d\r\r\r\r\r\r\r\r\r",AZ);

//			Serial_Printf("gyroX:%6d\r\r\r\r\r\r\r\r",GX);
//			Serial_Printf("gyroY:%6d\r\r\r\r\r\r\r\r",GY);
//			Serial_Printf("gyroZ:%6d\r\r\r\r\r\r\r\r",GZ);
//					
//			Serial_Printf("magX:%6d\r\r\r\r\r\r\r\r\r",MX);
//			Serial_Printf("magY:%6d\r\r\r\r\r\r\r\r\r",MY);
//			Serial_Printf("magZ:%6d\r\r\r\r\r\r\r\r\r",MZ);
			//Serial_Printf("\r\n");