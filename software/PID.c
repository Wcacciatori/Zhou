#include "main.h"
#include <math.h> 
/*
���裺
	1.�������ջ����ݣ����ĸ�ͨ��������ת��Ϊ�����ĽǶȺ����Ŵ�С
	2.���ÿ���ǵ�PID������
	3.������ƣ���
	4.����PID���ƽṹ��
	
	
	11.20 roll pitch ���ջ��ź�תΪ�����Ƕ�done
				todo��yaw pid��������Ҫ���¿��ǣ������ô���������������,DONE
				TODO:���pwmout������˵Ӧ�����Խ��Խ��Ŷ�
	11.28 ���⣺ң�����ر�ʱ��Ȼ��ת��������������쳣��ҡ�����м�ʱ�����ת��ҡ��������ʱת������
*/

#define dT 0.01

#define pid_version 1 //1���°汾��0�Ǿɰ汾

extern gyro gyroData;
extern volatile float yaw,pitch,roll;
float w_yaw_Exp,pitch_Exp,roll_Exp;
extern uint16_t pwm_IN[4];

uint16_t pwm_OUT[4]={0};

//ExpAngle exp_angle;
//uint16_t Speed;//�޷��������


//����ֵ
#if  pid_version == 0
PIDController YawController;
PIDController PitchController;
PIDController RollController;
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
	
    RollController.P_out = 5.0f;
    RollController.I_out = 0.0f;
    RollController.D_out = 0.0f;
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
    RollController.I_in = 0.0f;
    RollController.D_in = 0.00f;
    RollController.deltU_in = 0.0f;
    RollController.Err_in = 0.0f;
    RollController.lastErr_in = 0.00f;
		RollController.L_lastErr_in = 0.00f;
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
	RollController->Err_out = exp_Roll - now_Roll;
	RollController->deltU_out = A*RollController->Err_out+B*RollController->lastErr_out+C*RollController->L_lastErr_out;
	
	RollController->expW = limit(RollController->expW += RollController->deltU_out, 1000, -1000);
	
	D = RollController->P_in+RollController->I_in+RollController->D_in;
	E = -RollController->P_in-2*RollController->D_in;
	F = RollController->D_in;
	
	RollController->L_lastErr_in = RollController->lastErr_in;
	RollController->lastErr_in = RollController->Err_in;
	RollController->Err_in = RollController->expW - now_wRoll;
	RollController->deltU_in = D*RollController->Err_in + E*RollController->lastErr_in + F*RollController->L_lastErr_in;
	
}

////����������
//float RollPID_Contral(float exp_Roll, float now_Roll, float now_wRoll, PIDController *RollController){
//	float A,B,C;//�⻷����
//	float D,E,F;//�ڻ�����
//	float integral_in,integral_out;
//	float differential_in, differential_out;
//	
//	//�����⻷���
//	RollController->Err_out = exp_Roll - now_Roll;
//	
//	//�����⻷������
//	integral_out += RollController->Err_out*dT;
//	
//	//�����⻷΢����
//	differential_out = (RollController->Err_out - RollController->lastErr_out)/dT;
//	
//	//�����⻷���
//	RollController->deltU_out = RollController->P_out*RollController->Err_out + RollController->I_out*integral_out + RollController->D_out*differential_out;
//	
//	//�����ڻ����
//	RollController->Err_in = RollController->deltU_out - now_wRoll;
//	
//	//�����ڻ�������
//	//integral_in +=
//}

float PitchPID_Contral(float exp_Pitch, float now_Pitch, float now_wPitch, PIDController *PitchController){
	float A,B,C;//�⻷����
	float D,E,F;//�ڻ�����
	
	A = PitchController->P_out+PitchController->I_out+PitchController->D_out;
	B = -PitchController->P_out-2*PitchController->D_out;
	C = PitchController->D_out;
	
	PitchController->L_lastErr_out = PitchController->lastErr_out;
	PitchController->lastErr_out = PitchController->Err_out;
	PitchController->Err_out = exp_Pitch - now_Pitch;
	PitchController->deltU_out = A*PitchController->Err_out+B*PitchController->lastErr_out+C*PitchController->L_lastErr_out;
	
	PitchController->expW = limit(PitchController->expW += PitchController->deltU_out, 1000, -1000);
	
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

#else
//�²���
PIDController_New YawController;
PIDController_New PitchController_in;
PIDController_New RollController_in;

PIDController_New PitchController_out;
PIDController_New RollController_out;
void initializePIDControllers(){
	YawController.Kp=0;
	YawController.Ki=0;
	YawController.Kd=0;
	YawController.integral=0;
	YawController.differential=0;
	YawController.exp_value=0;
	YawController.lastErr=0;
	YawController.U=0;
	
	PitchController_out.Kp=0;
	PitchController_out.Ki=0;
	PitchController_out.Kd=0;
	PitchController_out.integral=0;
	PitchController_out.differential=0;
	PitchController_out.exp_value=0;
	PitchController_out.lastErr=0;
	PitchController_out.U=0;

	RollController_out.Kp=0.73;
	RollController_out.Ki=0.02;
	RollController_out.Kd=0;
	RollController_out.integral=0;
	RollController_out.differential=0;
	RollController_out.exp_value=0;
	RollController_out.lastErr=0;
	RollController_out.U=0;
	
	PitchController_in.Kp=0;
	PitchController_in.Ki=0;
	PitchController_in.Kd=0;
	PitchController_in.integral=0;
	PitchController_in.differential=0;
	PitchController_in.exp_value=0;
	PitchController_in.lastErr=0;
	PitchController_in.U=0;

	RollController_in.Kp=1;
	RollController_in.Ki=0;
	RollController_in.Kd=0;
	RollController_in.integral=0;
	RollController_in.differential=0;
	RollController_in.exp_value=0;
	RollController_in.lastErr=0;
	RollController_in.U=0;
}

void PID_Contral_(float exp_value, float test_value, PIDController_New *PIDController){
	float err;
	//�������
	err = exp_value - test_value;
	
	//�������
	PIDController->integral += err*dT;
	
	//����΢��
	PIDController->differential = err - PIDController->lastErr;
	
	//������
	PIDController->U = PIDController->Kp*err + PIDController->Ki*PIDController->integral + PIDController->Kd*PIDController->differential;

	//�洢err
	PIDController->lastErr = err;
}

#endif
float limit(float data, float max, float min){

	if(data > min && data < max){
		return data;
	}else if(data > max){
		return max;
	}else{
		return min;
	}
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
    if (input >= 590 && input <= 1280) {
        // 615 ~ 1235 �� 135 ~ 260
				return ((input - 615) * (260-126) / (1235-615)) + 126;
    } else {
        return 0; 
    }
}

//ע�⣺��Χ���˴���Ӧ�÷Ŵ�һЩ����ֹң���������źŶ�û��ӳ�䵽�����Զ�ת��0������������

//Yaw�ǵ�ӳ��: pwm->angleW, yawӳ�䵽�ķ�Χ����Ҫ����? yawֱ�Ӹ�ɽ��ٶȾ�����
float YawMap_value(uint16_t input) {
    if (input >= 600 && input <= 915) {
        // 600 ~ 915 �� -1500 ~ 0
        return ((input - 600) * (1500) / (915 - 600)) - 1500;
    } else if (input > 925 && input <= 1245) {
        // 925 ~ 1245 �� 0 ~ 1500
        return ((input - 925) * 1500 / (1245 - 925));
    } else {
        return 0; 
    }
}
//Roll�ǵ�ӳ��: pwm->angle
float RollMap_value(uint16_t input) {
    if (input >= 930 && input <= 1300) {
        // 1245 ~ 930 �� -1500 ~ 0
        return ((input - 930) * (1500) / (930 - 1245));
    } else if (input >= 600 && input <= 925) {
        // 925 ~ 615 �� 0 ~ 1500
        return ((input - 925) * 1500 / (615 - 925));
    } else {
        return 0; 
    }
}

//Pitch�ǵ�ӳ��: pwm->angle
float PitchMap_value(uint16_t input) {
    if (input >= 610 && input <= 918) {
        // 610 ~ 918 �� -1500 ~ 0
        return ((input - 610) * (1500) / (918 - 610)) - 1500;
    } else if (input >= 928 && input <= 1250) {
        // 928 ~ 1250 �� 0 ~ 1500
        return ((input - 928) * 1500 / (1250 - 928));
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
	pitch_Exp = PitchMap_value(pwm_IN[1])/100;//��һ����Ϊ�˶�Ӧ�����*1000������߾���
	roll_Exp 	= RollMap_value(pwm_IN[0])/100;
	w_yaw_Exp = YawMap_value(pwm_IN[3])/100;

}
/*
in������������
out��motor[4]�ĸ�������

*/

float PID_Contral(){
	
	int acc;//����
	
	//����ֵ��û����,��Ҫͨ��receiver������,done
	ExpAngleByReceiver(pwm_IN);

	acc = AccMap_value(pwm_IN[2]);
	
#if pid_version == 0
	RollPID_Contral(roll_Exp, roll, gyroData.x, &RollController);
	PitchPID_Contral(pitch_Exp, pitch, gyroData.y, &PitchController);
	YawPID_Contral(w_yaw_Exp, yaw, gyroData.z, &YawController);
		
	RollController.U 	= limit(RollController.U 	+= RollController.deltU_in, 500, -500);
	PitchController.U = limit(PitchController.U += PitchController.deltU_in, 500, -500);
	YawController.U   = limit(YawController.U 	+= YawController.deltU_in, 500, -500);
#else
	//roll�����⻷
	PID_Contral_(roll_Exp,roll,&RollController_out);
	PID_Contral_(RollController_out.U,gyroData.x, &RollController_in);	
	
	//pitch�����⻷
	PID_Contral_(pitch_Exp,pitch,&PitchController_out);
	PID_Contral_(PitchController_out.U,gyroData.y, &PitchController_in);	
	
	//yaw��
	PID_Contral_(w_yaw_Exp,gyroData.z,&YawController);
	
	
#endif
	
	//���⣺Uû���������� ���������õĹ����ѽ����ԭ�����ǿ�������ʼ��
//		Serial_Printf("roll_U%f\r\n ", RollController.U);
//		Serial_Printf("pitch_U:%f\r\n", PitchController.U);
//		Serial_Printf("w_yaw_U:%f\r\n ", YawController.U);
//		Serial_Printf("\r\n");
//		delay_ms(100);

	if(pwm_IN[2]>600)//����600˵��ң������
		if(acc>160){
			pwm_OUT[0] = acc +  RollController_in.U;
			pwm_OUT[1] = acc -  RollController_in.U;
			pwm_OUT[2] = acc +  RollController_in.U;
			pwm_OUT[3] = acc -  RollController_in.U;
		}else{ //��֤�ɻ��������
			pwm_OUT[0] = acc; 
			pwm_OUT[1] = acc; 
			pwm_OUT[2] = acc; 
			pwm_OUT[3] = acc; 
		}
	else {//С��600ң�����ر� �����������
		pwm_OUT[0] =0;
		pwm_OUT[1] =0;
		pwm_OUT[2] =0;
		pwm_OUT[3] =0;
	}

}
//����todo�ģ� �����޶ȣ���  ��ɱ���������������  ң�����ر���Ȼת������