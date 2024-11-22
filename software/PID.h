#ifndef __PID_H
#define __PID_H
typedef struct PID{
	float P_out;
	float I_out;
	float D_out;
	float deltU_out;
	float Err_out;
	float lastErr_out;
	float L_lastErr_out;
	
	float expW;
	
	float P_in;
	float I_in;
	float D_in;
	float deltU_in;
	float Err_in;
	float lastErr_in;
	float L_lastErr_in;
	
	float U;
}PIDController;

typedef struct ExpAngle{
	float roll;
	float pitch;
	float yaw;

}ExpAngle;

void initializePIDControllers(void);
float PID_Contral(void);
#endif
