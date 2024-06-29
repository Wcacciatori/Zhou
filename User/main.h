/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/main.h 
  * @author  MCD Application Team
  * @version V1.8.1
  * @date    27-January-2022
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h" // Device header
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"
#include "stdio.h"
#include "stdarg.h"
#include "misc.h"
#include "GY86.h"
#include "OLED.h"
#include "Serial.h"
#include "Delay.h"
#include "Power.h"
#include "PWM.h"
#include "Receiver.h"
#include "Receiver_L.h"
#include "OS.h"
#include "core_cm4.h"

typedef struct{
	int16_t  AX, AY, AZ, GX, GY, GZ, GA_X, GA_Z, GA_Y;
}gy86_data;



//typedef struct{
//	uint32_t Duty_acc;
//	uint32_t Duty_yaw;
//	uint32_t Duty_pit;
//	uint32_t Duty_rol;
//}motor_data;
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void TimingDelay_Decrement(void)	;

#endif /* __MAIN_H */

