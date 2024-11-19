#include "Serial.h"
#include "main.h"

uint16_t Serial_RxData;
uint8_t Serial_RxFlag;

extern	OS_EVENT *Sem_Task_UR;
extern	OS_ERR err;

#define TransUSART USART1

//void Serial_Init()//串口初始化--zhou!备用
//{
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

////PA11-接受  PA12	-发送
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_USART6);
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_USART6);

//	
//	GPIO_InitTypeDef GPIO_InitStructure;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
//	GPIO_Init(GPIOA , &GPIO_InitStructure);
//	
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
//	GPIO_Init(GPIOA , &GPIO_InitStructure);
//	
//	USART_InitTypeDef USART_InitStructure;
//	USART_InitStructure.USART_BaudRate = 9600;
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
//	USART_InitStructure.USART_Parity = USART_Parity_No;
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//	USART_Init(USART6, &USART_InitStructure);
//	
//	USART_ITConfig (USART6, USART_IT_RXNE, ENABLE);
//	
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//	
//	NVIC_InitTypeDef NVIC_InitStructure;
//	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//	NVIC_Init(&NVIC_InitStructure);
//	
//	USART_Cmd(USART6, ENABLE);
//}

void Serial_Init()//串口初始化--BT
{
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

//PA10-接受  PA9-发送
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);

	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOA , &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOA , &GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &USART_InitStructure);
	
	USART_ITConfig (USART1, USART_IT_RXNE, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART1, ENABLE);
}
	
void Serial_SendByte(uint8_t Byte)//发送单个字节
{
	//self
	USART_SendData(TransUSART, Byte);
	while(USART_GetFlagStatus(TransUSART, USART_FLAG_TXE) == RESET);
	
//	//zhou!
//	USART_SendData(USART6, Byte);
//	while(USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)//发送数组
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Array[i]);
	}
}

void Serial_SendString(char *String)//发送字符串
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		Serial_SendByte(String[i]);
	}
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)//返回x的y次方
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

void Serial_SendNumber(int32_t Number, uint8_t Length)//发送十进制数字
{
	uint8_t i;
	//uint32_t Number1;
	if (Number >= 0)
	{
		Serial_SendString("+");
		Number = Number;
	}
	else
	{
		Serial_SendString("-");
		Number = -Number;
	}
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
	}
}

int fputc(int ch, FILE *f)//重定向所需函数
{
	Serial_SendByte(ch);
	return ch;
}

void Serial_Printf(char *format, ...)//printf的重定向
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	Serial_SendString(String);
}

uint8_t Serial_GetRxFlag(void)//获取接受中断信号
{
	if (Serial_RxFlag == 1)
	{
		Serial_RxFlag = 0;
		return 1;
	}
	return 0;
}

uint8_t Serial_GetRxData(void)//获取数据
{
	return Serial_RxData;
}

void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)//判断是否接收到数据
	{
		Serial_RxData = USART_ReceiveData(USART1);//读取接收到的数据
		//Serial_GetRxFlag();//设置接受标志位，表示是否接到数据。
		//OSSemPost(Sem_Task_UR);
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);//清除接收中断标志位
	}

}

void USART6_IRQHandler(void)
{
	if (USART_GetITStatus(USART6, USART_IT_RXNE) == SET)//判断是否接收到数据
	{
		Serial_RxData = USART_ReceiveData(USART6);//读取接收到的数据
		//Serial_GetRxFlag();//设置接受标志位，表示是否接到数据。

		USART_ClearITPendingBit(USART6, USART_IT_RXNE);//清除接收中断标志位
	}

}

//int SumCheck(){
//	U8 sumcheck = 0;
//	U8 addcheck = 0;
//	for(uint8_t i=0; i < (data_buf[3]+4); i++)
//	{
//	sumcheck += data_buf[i]; //从帧头开始，对每一字节进行求和，直到DATA区结束
//	addcheck += sumcheck; //每一字节的求和操作，进行一次sumcheck的累加
//	}
//	//如果计算出的sumcheck和addcheck和接收到的check数据相等，代表校验通过，反之数据有误
//	if(sumcheck == data_buf[data_buf[3]+4] && addcheck == data_buf[data_buf[3]+5])
//	return 1; //校验通过
//	else
//	return 0; //校验失败

//}

void compute_check(uint8_t *sumcheck, uint8_t *addcheck, uint8_t data_buf[19]){
	for(uint8_t i=0; i < (data_buf[3]+4); i++)
	{
	*sumcheck += data_buf[i]; //从帧头开始，对每一字节进行求和，直到DATA区结束
	*addcheck += *sumcheck; //每一字节的求和操作，进行一次sumcheck的累加
	}
}
