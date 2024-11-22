#include "main.h"

/*
步骤：
	1.解析接收机数据，把四个通道的数据转化为期望的角度和油门大小
	2.设计每个角的PID控制器
	3.参数设计？？
	4.定义PID控制结构体
	
	
	11.20 roll pitch 接收机信号转为期望角度done
				todo：yaw pid控制器需要重新考虑，好像不用串级，单级就行了,DONE
				TODO:检测pwmout按理来说应该输出越来越多才对

*/
extern gyro gyroData;
extern float yaw,pitch,roll;
float w_yaw_Exp,pitch_Exp,roll_Exp;
extern uint16_t pwm_IN[4];
PIDController *YawController;
PIDController *PitchController;
PIDController *RollController;
uint16_t pwm_OUT[4];
ExpAngle exp_angle;
uint16_t Speed;//限幅后的油门
//赋初值

void initializePIDControllers() {
    // PID控制器初始化_out
    YawController->P_out = 1.0f;
    YawController->I_out = 0.1f;
    YawController->D_out = 0.01f;
    YawController->deltU_out = 0.0f;
    YawController->Err_out = 0.0f;
    YawController->lastErr_out = 0.00f;
		YawController->L_lastErr_out = 0.00f;
		YawController->expW = 0.0f;
		YawController->U = 0.0f;

    PitchController->P_out = 1.0f;
    PitchController->I_out = 0.1f;
    PitchController->D_out = 0.01f;
    PitchController->deltU_out = 0.0f;
    PitchController->Err_out = 0.0f;
    PitchController->lastErr_out = 0.00f;
		PitchController->L_lastErr_out = 0.00f;
		PitchController->expW = 0.0f;
		PitchController->U = 0.0f;
	
    RollController->P_out = 1.0f;
    RollController->I_out = 0.1f;
    RollController->D_out = 0.01f;
    RollController->deltU_out = 0.0f;
    RollController->Err_out = 0.0f;
    RollController->lastErr_out = 0.00f;
		RollController->L_lastErr_out = 0.00f;
		RollController->expW = 0.0f;
		RollController->U = 0.0f;
		
		// PID控制器初始化_in
    YawController->P_out = 1.0f;
    YawController->I_out = 0.1f;
    YawController->D_out = 0.01f;
    YawController->deltU_out = 0.0f;
    YawController->Err_out = 0.0f;
    YawController->lastErr_out = 0.00f;
		YawController->L_lastErr_out = 0.00f;

    PitchController->P_out = 1.0f;
    PitchController->I_out = 0.1f;
    PitchController->D_out = 0.01f;
    PitchController->deltU_out = 0.0f;
    PitchController->Err_out = 0.0f;
    PitchController->lastErr_out = 0.00f;
		PitchController->L_lastErr_out = 0.00f;

    RollController->P_out = 1.0f;
    RollController->I_out = 0.1f;
    RollController->D_out = 0.01f;
    RollController->deltU_out = 0.0f;
    RollController->Err_out = 0.0f;
    RollController->lastErr_out = 0.00f;
		RollController->L_lastErr_out = 0.00f;
}


//控制器串级
float RollPID_Contral(float exp_Roll, float now_Roll, float now_wRoll, PIDController *RollController){
	float A,B,C;//外环参数
	float D,E,F;//内环参数
	
	A = RollController->P_out+RollController->I_out+RollController->D_out;
	B = -RollController->P_out-2*RollController->D_out;
	C = RollController->D_out;
	
	RollController->L_lastErr_out = RollController->lastErr_out;
	RollController->lastErr_out = RollController->Err_out;
	RollController->Err_out = exp_Roll - now_wRoll;
	RollController->deltU_out = A*RollController->Err_out+B*RollController->lastErr_out+C*RollController->L_lastErr_out;
	
	RollController->expW += RollController->deltU_out;
	
	D = RollController->P_in+RollController->I_in+RollController->D_in;
	E = -RollController->P_in-2*RollController->D_in;
	F = RollController->D_in;
	
	RollController->L_lastErr_in = RollController->lastErr_in;
	RollController->lastErr_in = RollController->Err_in;
	RollController->Err_in = RollController->expW - now_wRoll;
	RollController->deltU_in = D*RollController->Err_in + E*RollController->lastErr_in + F*RollController->L_lastErr_in;
	
}

float PitchPID_Contral(float exp_Pitch, float now_Pitch, float now_wPitch, PIDController *PitchController){
	float A,B,C;//外环参数
	float D,E,F;//内环参数
	
	A = PitchController->P_out+PitchController->I_out+PitchController->D_out;
	B = -PitchController->P_out-2*PitchController->D_out;
	C = PitchController->D_out;
	
	PitchController->L_lastErr_out = PitchController->lastErr_out;
	PitchController->lastErr_out = PitchController->Err_out;
	PitchController->Err_out = exp_Pitch - now_wPitch;
	PitchController->deltU_out = A*PitchController->Err_out+B*PitchController->lastErr_out+C*PitchController->L_lastErr_out;
	
	PitchController->expW += PitchController->deltU_out;
	
	D = PitchController->P_in+PitchController->I_in+PitchController->D_in;
	E = -PitchController->P_in-2*PitchController->D_in;
	F = PitchController->D_in;
	
	PitchController->L_lastErr_in = PitchController->lastErr_in;
	PitchController->lastErr_in = PitchController->Err_in;
	PitchController->Err_in = PitchController->expW - now_wPitch;
	PitchController->deltU_in = D*PitchController->Err_in + E*PitchController->lastErr_in + F*PitchController->L_lastErr_in;
	
}

//单级就行
float YawPID_Contral(float exp_wYaw, float now_Yaw, float now_wYaw, PIDController *YawController){
//	float A,B,C;//外环参数
	
	float D,E,F;//内环参数
	
//	A = YawController->P_out+YawController->I_out+YawController->D_out;
//	B = -YawController->P_out-2*YawController->D_out;
//	C = YawController->D_out;
//	
//	YawController->L_lastErr_out = YawController->lastErr_out;
//	YawController->lastErr_out = YawController->Err_out;
//	YawController->Err_out = exp_Yaw - now_wYaw;
//	YawController->deltU_out = A*YawController->Err_out+B*YawController->lastErr_out+C*YawController->L_lastErr_out;
//	
	YawController->expW = exp_wYaw;
	
	D = YawController->P_in+YawController->I_in+YawController->D_in;
	E = -YawController->P_in-2*YawController->D_in;
	F = YawController->D_in;
	
	YawController->L_lastErr_in = YawController->lastErr_in;
	YawController->lastErr_in = YawController->Err_in;
	YawController->Err_in = YawController->expW - now_wYaw;
	YawController->deltU_in = D*YawController->Err_in+E*YawController->lastErr_in+F*YawController->L_lastErr_in;
	
}

/********************************************************
  * @brief      : ManualTurnMap_value
  * @details    : 从遥控器接收到的数据映射为角度
   * @param [in] : input：   遥控器接收到的数据   取值范围： ？？
									
  * @param [out]: 映射后的值  取值范围：-2000-2000

  * History
  * @date       : 2024/8/23
  *	@author     : wjy
  *
  ********************************************************/
int16_t AccMap_value(uint16_t input) {
    if (input >= 480 && input <= 1100) {
        // 145 ~ 195 到 -1000 ~ 0
        return ((input - 142) * 1000 / (195 - 142)) - 1000;
    } else if (input > 0xCD && input <= 0xFF) {
        // 202 ~ 252 到 0 ~ 1000
        return ((input - 202) * 1000 / (252 - 202));
    } else {
        return 0; 
    }
}

//注意：范围两端处都应该放大一些，防止遥控器给到信号而没有映射到进而自动转成0，会引发混乱

//Yaw角的映射: pwm->angleW, yaw映射到的范围还需要考虑? yaw直接搞成角速度就行了
float YawMap_value(uint16_t input) {
    if (input >= 1050 && input <= 1250) {
        // 1050 ~ 1250 到 -15 ~ 0
        return ((input - 1050) * (15) / (1250 - 1050)) - 15;
    } else if (input > 1320 && input <= 1550) {
        // 1320 ~ 1550 到 0 ~ 15
        return ((input - 1320) * 15 / (1550 - 1320));
    } else {
        return 0; 
    }
}
//Roll角的映射: pwm->angle
float RollMap_value(uint16_t input) {
    if (input >= 1180 && input <= 1380) {
        // 1380 ~ 1180 到 -1500 ~ 0
        return ((input - 1380) * (1500) / (1180 - 1380)) - 1500;
    } else if (input >= 965 && input <= 1160) {
        // 1160 ~ 965 到 0 ~ 1500
        return ((input - 1160) * 1500 / (965 - 1160));
    } else {
        return 0; 
    }
}

//Pitch角的映射: pwm->angle
float PitchMap_value(uint16_t input) {
    if (input >= 950 && input <= 1140) {
        // 950 ~ 1140 到 -15 ~ 0
        return ((input - 1140) * (15) / (950 - 1140)) - 15;
    } else if (input > 1320 && input <= 1550) {
        // 1170 ~ 1360 到 0 ~ 15
        return ((input - 1320) * 15 / (1550 - 1320));
    } else {
        return 0; 
    }
}

/*
IN:接收机4个pwm
out：三个控制器的ExpAngle

*/
void ExpAngleByReceiver(uint16_t pwm_IN[4]){
	/*
		油门-右上下
		偏航-右左右
		俯仰-左上下
		横滚-左左右
		
	*/
	pitch_Exp = PitchMap_value(pwm_IN[1]);
	roll_Exp = RollMap_value(pwm_IN[0])/100;
	w_yaw_Exp = YawMap_value(pwm_IN[3]);
	
}
/*
in：三个控制器
out：motor[4]四个电机输出

*/

float PID_Contral(){
	
	//期望值还没计算,需要通过receiver来计算,done
	ExpAngleByReceiver(pwm_IN);
	Serial_Printf("roll_Exp:%f\r\n ", roll_Exp);
//	Serial_Printf("w_yaw_Exp:%f\r\n", w_yaw_Exp);
//	Serial_Printf("pitch_Exp:%f\r\n ", pitch_Exp);
	delay_ms(1000);
	RollPID_Contral(roll_Exp, roll, gyroData.x, RollController);
	PitchPID_Contral(pitch_Exp, pitch, gyroData.y, PitchController);
	YawPID_Contral(w_yaw_Exp, yaw, gyroData.z, YawController);
		
	RollController->U  += RollController->deltU_in;
	PitchController->U += PitchController->deltU_in;
	YawController->U   += YawController->deltU_in;
	
	pwm_OUT[0] =  RollController->U - PitchController->U + YawController->U;
	pwm_OUT[1] =  RollController->U + PitchController->U - YawController->U;
	pwm_OUT[2] =  RollController->U - PitchController->U - YawController->U;
	pwm_OUT[3] =  RollController->U + PitchController->U + YawController->U;
	
	//pwm_IN[2] +
	//pwm_IN[2] +
	//pwm_IN[2] -
	//pwm_IN[2] -
	

}
