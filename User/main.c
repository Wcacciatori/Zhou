#include "main.h"
#include "task.h"


extern int16_t  AX, AY, AZ, GX, GY, GZ, MX, MZ, MY;
extern uint16_t pwm_IN[4];
extern uint16_t pwm_OUT[4];
extern volatile float q0,q1,q2,q3;
extern float q0Acc,q1Acc,q2Acc,q3Acc;//初始值可设为1,0,0,0
extern float q0gyro,q1gyro,q2gyro,q3gyro;//初始值可设为1,0,0,0
OS_ERR errr;
float tmp2;
void SysTick_Init(void){
	//OS_CPU_SysTickInitFreq(84000000);
	
	//配置时钟源用于驱动系统时钟
	RCC_PLLConfig(RCC_PLLSource_HSI, 8, 168, 4, 4);
	//配置AHB APB分频系数
	RCC_HCLKConfig(RCC_SYSCLK_Div1);
	RCC_PCLK1Config(RCC_HCLK_Div2);
	RCC_PCLK2Config(RCC_HCLK_Div1);
	RCC_PLLCmd(ENABLE);
	
	//设置flash
	FLASH->ACR |= FLASH_ACR_PRFTEN;
	FLASH->ACR |= FLASH_ACR_LATENCY_2WS;
	
	//使能系统时钟
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY)!=SET);
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);
	SysTick_Config(RCC_Clocks.SYSCLK_Frequency/OS_TICKS_PER_SEC);//para_In=两个中断之间的滴答数
}




int main(void)
{
		
		DELAY_Init(84);
		SysTick_Init();
		Serial_Init();
		Receicer_Init();
		GY86_Init();
		Motor_Init();
		Serial_Printf("ok");
	
		//重新设定电机
//		PWM_SetDuty1(200);
//		PWM_SetDuty2(200);
//		PWM_SetDuty3(200);
//		PWM_SetDuty4(200);
//		delay_ms(3000);
//		
//		PWM_SetDuty1(100);
//		PWM_SetDuty2(100);
//		PWM_SetDuty3(100);
//		PWM_SetDuty4(100);
//	
	
	
		//OS_CPU_SysTickInitFreq(84000000);
		OSInit();
		//OSTaskCreate((void *)Task_USART_test, (void *)0, &USART_test[99], 9);
		q0=1;q1=0;q2=0;q3=0;
		initializePIDControllers();
		//OSTaskCreateExt((void *)Task_GY86, (void *)0, &GY86[511], 7, 7, &GY86[0], sizeof(GY86), (void *)0, OS_TASK_OPT_SAVE_FP | OS_TASK_OPT_STK_CHK);
		//OSTaskCreate((void *)Task_Motor, (void *)0, &Motor[99], 8);
		OSTaskCreateExt((void *)Task_BT, (void *)0, &BT[511], 8, 8, &BT[0], sizeof(BT), (void *)0, OS_TASK_OPT_SAVE_FP);
		OSTaskCreateExt((void *)Task_PoseCalcu, (void *)0, &PoseCalcu[511], 6, 6, &PoseCalcu[0], sizeof(PoseCalcu), (void *)0, OS_TASK_OPT_SAVE_FP | OS_TASK_OPT_STK_CHK);
		//OSTaskNameSet(7, (INT8U *)"GY86", &errr);
		//OSTaskNameSet(8, (INT8U *)"Motor", &errr);
		OSTaskNameSet(9, (INT8U *)"BT", &errr);
		OSTaskNameSet(6, (INT8U *)"PoseCalcu", &errr);

		OS_TRACE_INIT();
		OSStart();


}




	
	
//	OLED_Init();
//	GY86_Init();
//	Serial_Init();
//	PWM_Init();
//	Receicer_Init();

//	//重新设定电机
//		TIM_SetCompare1(TIM3, 100);
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
