#include "main.h"
#include "task.h"



OS_ERR errr;

//RCC_GetClocksFreq
int main(void)
{
//		SysTick_Config(SystemCoreClock/OS_TICKS_PER_SEC);

//		OS_CPU_SysTickInitFreq(84000000);

		SysTick_Config(SystemCoreClock/OS_TICKS_PER_SEC);
		Serial_Init();
		Receicer_Init();
		GY86_Init();
		Motor_Init();
		SysTick_Config(SystemCoreClock/OS_TICKS_PER_SEC);
		OS_CPU_SysTickInitFreq(84000000);

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
	
		OSInit();
		//OSTaskCreate((void *)Task_USART_test, (void *)0, &USART_test[99], 9);
		OSTaskCreate((void *)Task_GY86, (void *)0, &GY86[99], 6);
		OSTaskCreate((void *)Task_Motor, (void *)0, &Motor[99], 7);
		OSTaskCreate((void *)Task_BT, (void *)0, &BT[99], 8);
		OSTaskNameSet(6, (INT8U *)"GY86", &errr);
		OSTaskNameSet(7, (INT8U *)"Motor", &errr);
		OSTaskNameSet(8, (INT8U *)"BT", &errr);
		OS_TRACE_INIT();
		OSStart();


}







	
	
	
	
	
//	OLED_Init();
//	GY86_Init();
//	Serial_Init();
//	PWM_Init();
//	Receicer_Init();




	


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
