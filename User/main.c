#include "main.h"
#include "task.h"


extern int16_t  AX, AY, AZ, GX, GY, GZ, MX, MZ, MY;

extern volatile float q0,q1,q2,q3;
extern float q0Acc,q1Acc,q2Acc,q3Acc;//初始值可设为1,0,0,0
extern float q0gyro,q1gyro,q2gyro,q3gyro;//初始值可设为1,0,0,0
OS_ERR errr;
float tmp2;
void SysTick_Init(void){
	//OS_CPU_SysTickInitFreq(84000000);
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);
	SysTick_Config(RCC_Clocks.SYSCLK_Frequency/OS_TICKS_PER_SEC);
}




int main(void)
{
		SysTick_Init();
		DELAY_Init(84);
		
		Serial_Init();
		Receicer_Init();
		GY86_Init();
		Motor_Init();
		Serial_Printf("ok");
//	while(1){
//		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
//		tmp2 = AX/16384.0;
//		Serial_Printf("ax:%f\r\n", tmp2);
//	}
		//OS_CPU_SysTickInitFreq(84000000);
		OSInit();
		//OSTaskCreate((void *)Task_USART_test, (void *)0, &USART_test[99], 9);
		q0=1;q1=0;q2=0;q3=0;

		OSTaskCreateExt((void *)Task_GY86, (void *)0, &GY86[511], 7, 7, &GY86[0], sizeof(GY86), (void *)0, OS_TASK_OPT_SAVE_FP | OS_TASK_OPT_STK_CHK);
		//OSTaskCreate((void *)Task_Motor, (void *)0, &Motor[99], 8);
		//OSTaskCreateExt((void *)Task_BT, (void *)0, &BT[99], 8, 8, &BT[0], sizeof(BT), (void *)0, OS_TASK_OPT_SAVE_FP);
		OSTaskCreateExt((void *)Task_PoseCalcu, (void *)0, &PoseCalcu[511], 6, 6, &PoseCalcu[0], sizeof(PoseCalcu), (void *)0, OS_TASK_OPT_SAVE_FP | OS_TASK_OPT_STK_CHK);
		OSTaskNameSet(7, (INT8U *)"GY86", &errr);
		//OSTaskNameSet(8, (INT8U *)"Motor", &errr);
		//OSTaskNameSet(9, (INT8U *)"BT", &errr);
		OSTaskNameSet(6, (INT8U *)"PoseCalcu", &errr);

		OS_TRACE_INIT();
		OSStart();


}





	
	
	
	
	
//	OLED_Init();
//	GY86_Init();
//	Serial_Init();
//	PWM_Init();
//	Receicer_Init();

	//重新设定电机
//		PWM_SetDuty1(10);
//		PWM_SetDuty2(10);
//		PWM_SetDuty3(10);
//		PWM_SetDuty4(10);
//		Delay_s(3);
//		
//		PWM_SetDuty1(5);
//		PWM_SetDuty2(5);
//		PWM_SetDuty3(5);
//		PWM_SetDuty4(5);


	


//void LED_Init(){
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB ,ENABLE);
//	
//	GPIO_InitTypeDef GPIO_InitStructure;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 ;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
//	GPIO_Init(GPIOB , &GPIO_InitStructure);
//}
//void LED_ON(){
//	GPIO_SetBits(GPIOB, GPIO_Pin_4);
//}
//void LED_OFF(){
//	GPIO_ResetBits(GPIOB, GPIO_Pin_4);
//}

//Systick_Config  OSTimeDly SystemCoreClock OS_TICKS_PER_SEC	OS_TaskIdle
