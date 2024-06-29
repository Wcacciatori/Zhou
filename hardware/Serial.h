#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stdio.h"
#include "stdarg.h"
#include "misc.h"
#include "Delay.h"


void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
uint8_t Serial_GetRxData(void);//获取数据
void USART1_IRQHandler(void);
void USART6_IRQHandler(void);
void Serial_Printf(char *format, ...);
void Serial_SendArray(int16_t *Array, uint16_t Length);//发送数组
void Serial_SendString(char *String);//发送字符串
void Serial_SendNumber(int32_t Number, uint8_t Length);//发送十进制数字
uint8_t Serial_GetRxFlag(void);//获取接受中断信号


#endif
