#ifndef _TASK_H
#define _TASK_H

OS_STK USART_RECEIVE[100];//Ê§°ÜÑùÀýËùÓÃÕ»

OS_STK USART_test[100];
OS_STK	GY86[100];
OS_STK	Motor[100];
OS_STK	BT[100];

void Task_USART_test(void);
void Task_Motor(void);
void Task_GY86(void);
void Task_BT(void);



#endif
