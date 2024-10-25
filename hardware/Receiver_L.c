#include "main.h"                  // Device header

extern uint16_t pwm_IN[4];

void	ICTIM2_Init();
void	ICTIM4_Init();

void Receicer_Init(){
	ICTIM2_Init();
	ICTIM4_Init();
}


void ICTIM2_Init(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
  
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2);	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2);	


	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
  
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 65536-1;//arr
	TIM_TimeBaseInitStructure.TIM_Prescaler = 16-1;//psc
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);

 
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0F;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM2,&TIM_ICInitStructure);
	

	TIM_ICInitTypeDef TIM_ICInitStructure1;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter = 0x0F;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM2,&TIM_ICInitStructure1);
	

  TI1_Config(TIM2, TIM_ICPolarity_Rising, TIM_ICSelection_DirectTI, 0x0F);
	TI2_Config(TIM2, TIM_ICPolarity_Rising, TIM_ICSelection_DirectTI, 0x0F);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
	
//	TIM_SelectInputTrigger(TIM2,TIM_TS_TI1FP1); 
//	TIM_SelectSlaveMode(TIM2,TIM_SlaveMode_Reset);
  
	TIM_Cmd(TIM2,ENABLE);

}

void ICTIM4_Init(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
  
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4);	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_TIM4);	


	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
  
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 65536-1;//arr
	TIM_TimeBaseInitStructure.TIM_Prescaler = 16-1;//psc
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);

 
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0F;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM4,&TIM_ICInitStructure);

	TIM_ICInitTypeDef TIM_ICInitStructure1;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM_ICInitStructure.TIM_ICFilter = 0x0F;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM4,&TIM_ICInitStructure1);

  TI1_Config(TIM4, TIM_ICPolarity_Rising, TIM_ICSelection_DirectTI, 0x0F);
	TI3_Config(TIM4, TIM_ICPolarity_Rising, TIM_ICSelection_DirectTI, 0x0F);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_CC3, ENABLE);
	
//	TIM_SelectInputTrigger(TIM2,TIM_TS_TI1FP1); 
//	TIM_SelectSlaveMode(TIM2,TIM_SlaveMode_Reset);
  
	TIM_Cmd(TIM4,ENABLE);

}



//OLD 其实应该也可以，只是当时原理图搞错，一直收的是通道5的信号
/*
使能了2个中断源，CCR1捕获到上升沿，CCR2捕获到下降沿会产生中断
中断服务程序需要将ccr2锁存的数据比上ccr1锁存的数据得到占空比，然后再在下一个pwm波来之前将cnt清零
进入中断时，要判断检测到的是上升沿还是下降沿
	若检测到上升沿，则得到当前ccr1的值，并计算占空比，将cnt清零
	若检测到下降沿，则得到当前ccr2的值
*/
//void TIM2_IRQHandler(void)
//{
//	OS_CPU_SR  cpu_sr = 0u;
//	
//	
//	OS_ENTER_CRITICAL();
//	OSIntEnter();
//	if(TIM_GetITStatus(TIM2, TIM_IT_CC1) == SET)
//	{
//		DataCCR1 = TIM_GetCapture1(TIM2);
//		Data = (DataCCR2+1)*100/(DataCCR1+1);
//		Duty[0] = Data*5/7;
//		TIM_SetCounter(TIM2, 0);
//		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
//	}
//	else if(TIM_GetITStatus(TIM2, TIM_IT_CC2) == SET)
//	{
//		DataCCR2 = TIM_GetCapture2(TIM2);
//		//Serial_Printf("test");
//		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
//	}
//	else if(TIM_GetITStatus(TIM2, TIM_IT_CC3) == SET)
//	{
//		DataCCR3 = TIM_GetCapture3(TIM2);
//		Serial_Printf("test");
//		Data1 = (DataCCR4+1)*100/(DataCCR3+1);
//		Duty[1] = Data1*5/7;
//		TIM_SetCounter(TIM2, 0);
//		TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);		
//	}
//	else if(TIM_GetITStatus(TIM2, TIM_IT_CC4) == SET)
//	{
//		DataCCR4 = TIM_GetCapture4(TIM2);
//		TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
//	}
//	OSIntExit();
//	OS_EXIT_CRITICAL();
//}

//NEW
uint8_t TIM2_CAPTURE_STA[4];
uint16_t TIM2_CAPTURE_VAL[4][2];
	
void TIM2_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr = 0u;
	
	uint8_t i;
	OS_ENTER_CRITICAL();
	OSIntEnter();
	if(TIM_GetITStatus(TIM2, TIM_IT_CC1) == SET)
	{
						i = 0;
			if (TIM2_CAPTURE_STA[i]) { //已捕获到低电平，说明此时捕获到下降沿
                TIM2_CAPTURE_STA[i] = 0; //更改捕获状态
                TIM2_CAPTURE_VAL[i][1] = TIM_GetCapture1(TIM2); //读取下降沿对应计数器值
                pwm_IN[2] = TIM2_CAPTURE_VAL[i][1]*100 / TIM2_CAPTURE_VAL[i][0] ; //计算占空比
                pwm_IN[2] = (pwm_IN[2] + 1) *5/7;
								TIM2->CCER &= ~(0x0001 << (1 + 4 * i)); //CCxP=00 通道上升沿捕获
			} else { //捕获到上升沿
                TIM2_CAPTURE_STA[i] = 1; //更改捕获状态
                TIM2_CAPTURE_VAL[i][0] = TIM_GetCapture1(TIM2); //读取上升沿对应计数器值
								TIM_SetCounter(TIM2, 0);
                TIM2->CCER |= (0x0001 << (1 + 4 * i)); //CCxP=01 通道下降沿捕获
            }
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
    
	}
	if(TIM_GetITStatus(TIM2, TIM_IT_CC2) == SET)
	{
					i = 1;
					if (TIM2_CAPTURE_STA[i]) { //已捕获到低电平，说明此时捕获到下降沿
                TIM2_CAPTURE_STA[i] = 0; //更改捕获状态
                TIM2_CAPTURE_VAL[i][1] = TIM_GetCapture2(TIM2); //读取下降沿对应计数器值
                pwm_IN[0] = TIM2_CAPTURE_VAL[i][1]*100 / TIM2_CAPTURE_VAL[i][0] ; //计算占空比
                pwm_IN[0] = (pwm_IN[0]) *5/4 -5;
								TIM2->CCER &= ~(0x0001 << (1 + 4 * 1)); //CCxP=00 通道上升沿捕获
			} else { //捕获到上升沿
                TIM2_CAPTURE_STA[i] = 1; //更改捕获状态
                TIM2_CAPTURE_VAL[i][0] = TIM_GetCapture2(TIM2); //读取上升沿对应计数器值
								TIM_SetCounter(TIM2, 0);
                TIM2->CCER |= (0x0001 << (1 + 4 * 1)); //CCxP=01 通道下降沿捕获
            }
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
	}
	OSIntExit();
	OS_EXIT_CRITICAL();
}

uint8_t TIM4_CAPTURE_STA[4];
uint16_t TIM4_CAPTURE_VAL[4][2];

void TIM4_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr = 0u;
	
	uint8_t i;
	OS_ENTER_CRITICAL();
	OSIntEnter();
	if(TIM_GetITStatus(TIM4, TIM_IT_CC1) == SET)
	{
						i = 0;
			if (TIM4_CAPTURE_STA[i]) { //已捕获到低电平，说明此时捕获到下降沿
                TIM4_CAPTURE_STA[i] = 0; //更改捕获状态
                TIM4_CAPTURE_VAL[i][1] = TIM_GetCapture1(TIM4); //读取下降沿对应计数器值
                pwm_IN[3] = TIM4_CAPTURE_VAL[i][1]*100 / TIM4_CAPTURE_VAL[i][0] ; //计算占空比
								pwm_IN[3] = (pwm_IN[3])*5/4 -5;
                TIM4->CCER &= ~(0x0001 << (1 + 4 * 0)); //CCxP=00 通道上升沿捕获
			} else { //捕获到上升沿
                TIM4_CAPTURE_STA[i] = 1; //更改捕获状态
                TIM4_CAPTURE_VAL[i][0] = TIM_GetCapture1(TIM4); //读取上升沿对应计数器值
								TIM_SetCounter(TIM4, 0);
                TIM4->CCER |= (0x0001 << (1 + 4 * 0)); //CCxP=01 通道下降沿捕获
            }
        TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
    
	}
	if(TIM_GetITStatus(TIM4, TIM_IT_CC3) == SET)
	{
					i = 2;
					if (TIM4_CAPTURE_STA[i]) { //已捕获到低电平，说明此时捕获到下降沿
                TIM4_CAPTURE_STA[i] = 0; //更改捕获状态
                TIM4_CAPTURE_VAL[i][1] = TIM_GetCapture3(TIM4); //读取下降沿对应计数器值
                pwm_IN[1] = TIM4_CAPTURE_VAL[i][1]*100 / TIM4_CAPTURE_VAL[i][0] ; //计算占空比
                pwm_IN[1] = (pwm_IN[1])*5/4 -5;
								TIM4->CCER &= ~(0x0001 << (1 + 4 * 2)); //CCxP=00 通道上升沿捕获
			} else { //捕获到上升沿
                TIM4_CAPTURE_STA[i] = 1; //更改捕获状态
                TIM4_CAPTURE_VAL[i][0] = TIM_GetCapture3(TIM4); //读取上升沿对应计数器值
								TIM_SetCounter(TIM4, 0);
                TIM4->CCER |= (0x0001 << (1 + 4 * 2)); //CCxP=01 通道下降沿捕获
            }
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);
	}
	OSIntExit();
	OS_EXIT_CRITICAL();
}



























/*现在的问题：捕获不到PWM信号（电平跳变信号），CCR始终为0*/
