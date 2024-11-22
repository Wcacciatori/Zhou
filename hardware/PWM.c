#include "main.h"                  // Device header


/*
定时中断初始化步骤：
1.开启RCC时钟
2.选择时基单元的时钟源，对于定时中断，一般选择内部时钟源
3.配置时基单元，包括预分频器，自动重装器，计数模式等
4.配置输出中断控制，允许更新中断输出到NVIC
5.配置NVIC，在NVIC中打开定时器中断的通道，并分配一个优先级
6.运行控制
7.使能计数器
8.再写一个中断函数（初始化外）

PWM输出比较初始化步骤：
1.开启RCC时钟，TIM和GPIO
2.GPIO初始化，开启重映射
3.选择时基单元的时钟源
4.配置时基单元，包括预分频器，自动重装器，计数模式等
5.输出比较通道初始化

PWM输入捕获初始化步骤：
1.RCC开启时钟，把TIM和GPIO的时钟都打开
2.GPIO初始化，配置为输入模式，一般选择上拉或者浮空
3.配置时基单元，让CNT在内部时钟的驱动下自增运行
4.配置输入捕获单元，包括滤波器，极性，直连通道or交叉通道，分频器等参数（配置结构体）
5.选择从模式的触发源（TI1FP1）
6.选择触发之后执行的操作（Reset）
7.TIM_Cmd，开启定时器

计算公式：
PWM频率：Freq = CK_PSC/(PSC+1)/(ARR+1)
PWM占空比：Duty = CCR/(ARR+1)
PWM分辨率：Reso = 1/(ARR+1)
*/


void PWM_Init();
void Motor_Init(void){

	PWM_Init();
}


void PWM_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC ,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOC , &GPIO_InitStructure);
			
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);
	
	TIM_InternalClockConfig(TIM3);//默认调用的也是内部时钟，其实写不写都一样
	
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;;;
	TIM_TimeBaseInitStruct.TIM_Period = 2000 - 1;//ARR F=50hz
	TIM_TimeBaseInitStruct.TIM_Prescaler = 160 - 1;//PSC	得周期为20ms  84/160 = 
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);

//通道x：通道x对应的即为PWMx	如：通道三需要复用功能为PWM3的GPIO口
	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_OCStructInit(&TIM_OCInitStruct);
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = 0;//用以设置CCR
	TIM_OC1Init(TIM3, &TIM_OCInitStruct);
	TIM_OC2Init(TIM3, &TIM_OCInitStruct);
	TIM_OC3Init(TIM3, &TIM_OCInitStruct);
	TIM_OC4Init(TIM3, &TIM_OCInitStruct);


	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM3,ENABLE);
	
	TIM_Cmd(TIM3, ENABLE);
}


/*
用以单独更改CCR的值 范围0-
*/
void PWM_SetDuty1(float Compare)
{
	TIM_SetCompare1(TIM3, Compare*20);
}

void PWM_SetDuty2(float Compare)
{
	TIM_SetCompare2(TIM3, Compare*20);
}

void PWM_SetDuty3(float Compare)
{
	TIM_SetCompare3(TIM3, Compare*20);
}

void PWM_SetDuty4(float Compare)
{
	TIM_SetCompare4(TIM3, Compare*20);
}
