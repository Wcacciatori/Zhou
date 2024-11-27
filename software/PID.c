#include "main.h"

/*
���裺
	1.�������ջ����ݣ����ĸ�ͨ��������ת��Ϊ�����ĽǶȺ����Ŵ�С
	2.���ÿ���ǵ�PID������
	3.������ƣ���
	4.����PID���ƽṹ��
	
	
	11.20 roll pitch ���ջ��ź�תΪ�����Ƕ�done
				todo��yaw pid��������Ҫ���¿��ǣ������ô���������������,DONE
				TODO:���pwmout������˵Ӧ�����Խ��Խ��Ŷ�

*/
extern gyro gyroData;
extern float yaw,pitch,roll;
float w_yaw_Exp,pitch_Exp,roll_Exp;
extern uint16_t pwm_IN[4];
PIDController YawController;
PIDController PitchController;
PIDController RollController;
uint16_t pwm_OUT[4];
ExpAngle exp_angle;
uint16_t Speed;//�޷��������
//����ֵ

void initializePIDControllers() {
    // PID��������ʼ��_out
    YawController.P_out = 1.0f;
    YawController.I_out = 0.1f;
    YawController.D_out = 0.01f;
    YawController.deltU_out = 0.0f;
    YawController.Err_out = 0.0f;
    YawController.lastErr_out = 0.00f;
		YawController.L_lastErr_out = 0.00f;
		YawController.expW = 0.0f;
		YawController.U = 0.0f;

    PitchController.P_out = 1.0f;
    PitchController.I_out = 0.1f;
    PitchController.D_out = 0.01f;
    PitchController.deltU_out = 0.0f;
    PitchController.Err_out = 0.0f;
    PitchController.lastErr_out = 0.00f;
		PitchController.L_lastErr_out = 0.00f;
		PitchController.expW = 0.0f;
		PitchController.U = 0.0f;
	
    RollController.P_out = 8.0f;
    RollController.I_out = 0.1f;
    RollController.D_out = 0.01f;
    RollController.deltU_out = 0.0f;
    RollController.Err_out = 0.0f;
    RollController.lastErr_out = 0.00f;
		RollController.L_lastErr_out = 0.00f;
		RollController.expW = 0.0f;
		RollController.U = 0.0f;
		
		// PID��������ʼ��_in
    YawController.P_in = 1.0f;
    YawController.I_in = 0.1f;
    YawController.D_in = 0.01f;
    YawController.deltU_in = 0.0f;
    YawController.Err_in = 0.0f;
    YawController.lastErr_in = 0.00f;
		YawController.L_lastErr_in = 0.00f;

    PitchController.P_in = 1.0f;
    PitchController.I_in = 0.1f;
    PitchController.D_in = 0.01f;
    PitchController.deltU_in = 0.0f;
    PitchController.Err_in = 0.0f;
    PitchController.lastErr_in = 0.00f;
		PitchController.L_lastErr_in = 0.00f;

    RollController.P_in = 1.0f;
    RollController.I_in = 0.1f;
    RollController.D_in = 0.01f;
    RollController.deltU_in = 0.0f;
    RollController.Err_in = 0.0f;
    RollController.lastErr_in = 0.00f;
		RollController.L_lastErr_in = 0.00f;
}

float limit(float data, float max, float min){

	if(data > min && data < max){
		return data;
	}else if(data > max){
		return max;
	}else{
		return min;
	}
}


//����������
float RollPID_Contral(float exp_Roll, float now_Roll, float now_wRoll, PIDController *RollController){
	float A,B,C;//�⻷����
	float D,E,F;//�ڻ�����
	
	A = RollController->P_out+RollController->I_out+RollController->D_out;
	B = -RollController->P_out-2*RollController->D_out;
	C = RollController->D_out;
	
	RollController->L_lastErr_out = RollController->lastErr_out;
	RollController->lastErr_out = RollController->Err_out;
	RollController->Err_out = exp_Roll - now_wRoll;
	RollController->deltU_out = A*RollController->Err_out+B*RollController->lastErr_out+C*RollController->L_lastErr_out;
	
	RollController->expW = limit(RollController->expW += RollController->deltU_out, 500, -500);
	
	D = RollController->P_in+RollController->I_in+RollController->D_in;
	E = -RollController->P_in-2*RollController->D_in;
	F = RollController->D_in;
	
	RollController->L_lastErr_in = RollController->lastErr_in;
	RollController->lastErr_in = RollController->Err_in;
	RollController->Err_in = RollController->expW - now_wRoll;
	RollController->deltU_in = D*RollController->Err_in + E*RollController->lastErr_in + F*RollController->L_lastErr_in;
	
}

float PitchPID_Contral(float exp_Pitch, float now_Pitch, float now_wPitch, PIDController *PitchController){
	float A,B,C;//�⻷����
	float D,E,F;//�ڻ�����
	
	A = PitchController->P_out+PitchController->I_out+PitchController->D_out;
	B = -PitchController->P_out-2*PitchController->D_out;
	C = PitchController->D_out;
	
	PitchController->L_lastErr_out = PitchController->lastErr_out;
	PitchController->lastErr_out = PitchController->Err_out;
	PitchController->Err_out = exp_Pitch - now_wPitch;
	PitchController->deltU_out = A*PitchController->Err_out+B*PitchController->lastErr_out+C*PitchController->L_lastErr_out;
	
	PitchController->expW = limit(PitchController->expW += PitchController->deltU_out, 500, -500);
	
	D = PitchController->P_in+PitchController->I_in+PitchController->D_in;
	E = -PitchController->P_in-2*PitchController->D_in;
	F = PitchController->D_in;
	
	PitchController->L_lastErr_in = PitchController->lastErr_in;
	PitchController->lastErr_in = PitchController->Err_in;
	PitchController->Err_in = PitchController->expW - now_wPitch;
	PitchController->deltU_in = D*PitchController->Err_in + E*PitchController->lastErr_in + F*PitchController->L_lastErr_in;
	
}

//��������
float YawPID_Contral(float exp_wYaw, float now_Yaw, float now_wYaw, PIDController *YawController){

	float D,E,F;//�ڻ�����
	
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
  * @details    : ��ң�������յ�������ӳ��Ϊ�Ƕ�
   * @param [in] : input��   ң�������յ�������   ȡֵ��Χ�� ����
									
  * @param [out]: ӳ����ֵ  ȡֵ��Χ��-2000-2000

  * History
  * @date       : 2024/8/23
  *	@author     : wjy
  *
  ********************************************************/
int16_t AccMap_value(uint16_t input) {
    if (input >= 400 && input <= 1100) {
        // 950 ~ 485 �� 0 ~ 2000
        return -((input - 485) * 2000 / (950 - 485)) + 2000;

    } else {
        return 0; 
    }
}

//ע�⣺��Χ���˴���Ӧ�÷Ŵ�һЩ����ֹң���������źŶ�û��ӳ�䵽�����Զ�ת��0������������

//Yaw�ǵ�ӳ��: pwm->angleW, yawӳ�䵽�ķ�Χ����Ҫ����? yawֱ�Ӹ�ɽ��ٶȾ�����
float YawMap_value(uint16_t input) {
    if (input >= 900 && input <= 1130) {
        // 940 ~ 1130 �� -1500 ~ 0
        return ((input - 940) * (1500) / (1130 - 940)) - 1500;
    } else if (input > 1150 && input <= 1445) {
        // 1150 ~ 1345 �� 0 ~ 1500
        return ((input - 1150) * 1500 / (1345 - 1150));
    } else {
        return 0; 
    }
}
//Roll�ǵ�ӳ��: pwm->angle
float RollMap_value(uint16_t input) {
    if (input >= 1180 && input <= 1480) {
        // 1380 ~ 1180 �� -1500 ~ 0
        return ((input - 1380) * (1500) / (1180 - 1380)) - 1500;
    } else if (input >= 905 && input <= 1160) {
        // 1160 ~ 965 �� 0 ~ 1500
        return ((input - 1160) * 1500 / (965 - 1160));
    } else {
        return 0; 
    }
}

//Pitch�ǵ�ӳ��: pwm->angle
float PitchMap_value(uint16_t input) {
    if (input >= 850 && input <= 1140) {
        // 950 ~ 1140 �� -1500 ~ 0
        return ((input - 950) * (1500) / (1140 - 950)) - 1500;
    } else if (input >= 1170 && input <= 1460) {
        // 1170 ~ 1360 �� 0 ~ 1500
        return ((input - 1170) * 1500 / (1360 - 1170));
    } else {
        return 0; 
    }
}

/*
IN:���ջ�4��pwm
out��������������ExpAngle

*/
void ExpAngleByReceiver(uint16_t pwm_IN[4]){
	/*
		����-������
		ƫ��-������
		����-������
		���-������	
	*/
	pitch_Exp = PitchMap_value(pwm_IN[1])/100;
	roll_Exp = RollMap_value(pwm_IN[0])/100;
	w_yaw_Exp = YawMap_value(pwm_IN[3])/100;

}
/*
in������������
out��motor[4]�ĸ�������

*/

float PID_Contral(){
	
	int acc;
	
	//����ֵ��û����,��Ҫͨ��receiver������,done
	ExpAngleByReceiver(pwm_IN);

	acc = AccMap_value(pwm_IN[2]);
	RollPID_Contral(roll_Exp, roll, gyroData.x, &RollController);
	PitchPID_Contral(pitch_Exp, pitch, gyroData.y, &PitchController);
	YawPID_Contral(w_yaw_Exp, yaw, gyroData.z, &YawController);
		
	RollController.U 	= limit(RollController.U 	+= RollController.deltU_in, 500, -500);
	PitchController.U = limit(PitchController.U += PitchController.deltU_in, 500, -500);
	YawController.U   = limit(YawController.U 	+= YawController.deltU_in, 500, -500);
	
	
	//���⣺Uû���������� ���������õĹ����ѽ����ԭ�����ǿ�������ʼ��
//		Serial_Printf("roll_U%f\r\n ", RollController.U);
//		Serial_Printf("pitch_U:%f\r\n", PitchController.U);
//		Serial_Printf("w_yaw_U:%f\r\n ", YawController.U);
//		Serial_Printf("\r\n");
//		delay_ms(100);

	if(acc>500){
		pwm_OUT[0] = acc +  RollController.U;
		pwm_OUT[1] = acc +  RollController.U;
		pwm_OUT[2] = acc -  RollController.U;
		pwm_OUT[3] = acc -  RollController.U;
	}else{
		pwm_OUT[0] = acc; 
		pwm_OUT[1] = acc; 
		pwm_OUT[2] = acc; 
		pwm_OUT[3] = acc; 
		
	}

}
