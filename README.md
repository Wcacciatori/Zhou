## 项目简介

该项目为电子科技大学信息与软件工程学院综合课程设计课题：基于STM32F4和ucOS-II的四旋翼飞行器 。

主要内容包括：

- 最小系统板转接板的原理图设计和pcb绘制。

- 传感器驱动通信。包括集成了加速度计 陀螺仪 磁力计的GY86和串口通信模块。
- ucOS-II操作系统的移植。
- 传感器数据校准滤波融合，完成实时姿态解算。
- 接收遥控器的pwm信号，结合解算出来的姿态数据进行pid反馈控制，最后输出到四个电机。

大致结构如下图：

![1731846285081](C:\Users\JIANYE~1\AppData\Local\Temp\1731846285081.png)

## 实物图

![1731846859726](C:\Users\JIANYE~1\AppData\Local\Temp\1731846859726.png)

## 工程结构

├── Cfg
├── DebugConfig
├── Listings
├── Objects
├── RTE
├── Source	--ucOS的源文件
├── Startup
├── Sys_View_Config
├── Sys_View_Src
├── User
│   ├── main.c	--主函数
│   ├── main.h
│   ├── stm32f4xx_conf.h
│   ├── stm32f4xx_it.c
│   ├── stm32f4xx_it.h
│   ├── task.c	--运行的任务
│   └── task.h
├── hardware	--硬件模块的驱动通信
│   ├── GY86.c
│   ├── GY86.h
│   ├── Motor.c
│   ├── Motor.h
│   ├── MyIIC.c
│   ├── MyIIC.h
│   ├── OLED.c
│   ├── OLED.h
│   ├── OLED_Font.h
│   ├── PWM.c
│   ├── PWM.h
│   ├── Receiver.c
│   ├── Receiver.h
│   ├── Receiver_L.c
│   ├── Receiver_L.h
│   ├── Serial.c
│   └── Serial.h
├── library	--ST的标准库函数
├── software	
│   ├── PID.c
│   ├── PID.h
│   ├── PoseCalculate.c	--姿态解算
│   ├── PoseCalculate.h
│   └── gaussnewton.c	--求解最优化参数
└── system
    ├── Delay.c
    ├── Delay.h
    ├── Power.c
    └── Power.h



