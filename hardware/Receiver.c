/*


已弃用

*/


#include "main.h"                  // Device header




/*
接收机的功能为：接受发射器发射来的各个通道的pwm波，之后再获取并输出到各个电机上

此文件应包含函数有：
	1.接收机的初始化
			四个GPIO的初始化
			调用GPIO_PinAFConfig配置复用功能
			配置时基单元
			配置各个通道（输入捕获单元）
			选择从模式和触发源
			开启定时器
	2.接受存储并处理pwm波

*/
void Receicer_Init1(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA ,ENABLE);
	

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 ;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOA , &GPIO_InitStructure);
			
	TIM_InternalClockConfig(TIM5);//默认调用的也是内部时钟，其实写不写都一样

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5);	
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 2000 - 1;//ARR
	TIM_TimeBaseInitStruct.TIM_Prescaler = 160- 1;//PSC	得周期为14ms
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStruct);
	
	TIM_ICInitTypeDef TIM_ICInitStruct;
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStruct.TIM_ICFilter = 0xF;
	TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_PWMIConfig(TIM5, &TIM_ICInitStruct);
	
	TIM_ICInitTypeDef TIM_ICInitStruct1;
	TIM_ICInitStruct1.TIM_Channel = TIM_Channel_3;
	TIM_ICInitStruct1.TIM_ICFilter = 0xF;
	TIM_ICInitStruct1.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStruct1.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStruct1.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM5, &TIM_ICInitStruct1);
	
	TIM_ICInitStruct1.TIM_Channel = TIM_Channel_4;
	TIM_ICInitStruct1.TIM_ICFilter = 0xF;
	TIM_ICInitStruct1.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICInitStruct1.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStruct1.TIM_ICSelection = TIM_ICSelection_TRC ;
	TIM_ICInit(TIM5, &TIM_ICInitStruct1);	
	
	TIM_SelectInputTrigger(TIM5, TIM_TS_TI1FP1);
	TIM_SelectSlaveMode(TIM5, TIM_SlaveMode_Reset);
	
	TIM_Cmd(TIM5, ENABLE);
}

//uint32_t IC_GetFreq(void)
//{
//	return 1000000 / (TIM_GetCapture1(TIM5) + 1);
//}

//uint32_t IC_GetDuty(void)
//{
//	return (TIM_GetCapture2(TIM5) + 1) * 100 / (TIM_GetCapture1(TIM5) + 1);
//}


