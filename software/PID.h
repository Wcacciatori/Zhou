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
	
	float P_in;
	float I_in;
	float D_in;
	float deltU_in;
	float Err_in;
	float lastErr_in;
	float L_lastErr_in;
}PIDController;

void initializePIDControllers(void);

#endif
