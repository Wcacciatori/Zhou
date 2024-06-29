#ifndef __PWM_H
#define __PWM_H

void Motor_Init(void);
void PWM_SetDuty1(float Compare);
void PWM_SetDuty2(float Compare);
void PWM_SetDuty3(float Compare);
void PWM_SetDuty4(float Compare);


#endif
/*
NOTE：


三个最重要的寄存器：
	预分频器PSC：连接基准时钟的输入
	计数器CNT：对分频后的计数时钟进行计数，计数时钟每来一个上升沿，计数器的值就++
	自动重装寄存器ARR：计数到达设定目标时，产生中断信号并清零计数器
一个也很重要的寄存器：
	捕获比较寄存器CCR：输出比较通过比较CNT与CCR值的关系来对电平进行置0置1或者翻转的操作

ps：通用和高级定时器四个输出比较的通道有各自的CCR，但是共用一个CNT




*/
