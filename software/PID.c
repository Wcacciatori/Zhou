#include "main.h"

/*
步骤：
	1.解析接收机数据，把四个通道的数据转化为期望的角度和油门大小
	2.设计每个角的PID控制器
	3.参数设计？？
	4.定义PID控制结构体

*/
extern gyro gyroData;


PIDController *YawController;
PIDController *PitchController;
PIDController *RollController;

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


////控制器还没有写成串级
//float RollPID_Contral(float exp_Roll, float now_Roll, float now_wRoll, PIDController *RollController){
//	float A,B,C;//
//	
//	A = RollController->P+RollController->I+RollController->D;
//	B = -RollController->P-2*RollController->D;
//	C = RollController->D;
//	
//	RollController->L_lastErr = RollController->lastErr;
//	RollController->lastErr = RollController->Err;
//	RollController->Err = exp_Roll - 
//	RollController->deltU = A*RollController->Err+B*RollController->lastErr+C*RollController->L_lastErr;
//	
//}

//float PitchPID_Contral(float exp_Pitch, float now_Pitch, float now_wPitch, PIDController *PitchController){
//	float A,B,C;//
//	
//	A = RollController->P+RollController->I+RollController->D;
//	B = -RollController->P-2*RollController->D;
//	C = RollController->D;
//	
//	RollController->L_lastErr = RollController->lastErr;
//	RollController->lastErr = RollController->deltU;
//	RollController->deltU = A*RollController->Err+B*RollController->lastErr+C*RollController->L_lastErr;
//	
//}

//float YawPID_Contral(float exp_Yaw, float now_Yaw, float now_wYaw, PIDController *YawController){
//	float A,B,C;//
//	
//	A = YawController->P+YawController->I+YawController->D;
//	B = -YawController->P-2*YawController->D;
//	C = YawController->D;
//	
//	YawController->L_lastErr = YawController->lastErr;
//	YawController->lastErr = YawController->deltU;
//	YawController->deltU = A*YawController->Err+B*YawController->lastErr+C*YawController->L_lastErr;
//	
//}
