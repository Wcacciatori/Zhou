#ifndef __POSECALCULATE_H
#define __POSECALCULATE_H
typedef struct w{
		float x;
		float y;
		float z;

}gyro;

typedef struct z{
		float x;
		float y;
		float z;

}mag;

typedef struct y{
		float x;
		float y;
		float z;

}acc;
void madgwick(float *q0,float *q1,float *q2,float *q3);
void updateAngleTmp(float *q0,float *q1,float *q2,float *q3);
void qAccCompute(float *q0Acc,float *q1Acc,float *q2Acc,float *q3Acc);
void updateQuaternion(float *q0gyro,float *q1gyro,float *q2gyro,float *q3gyro);
float invSqrt(float x);
#endif
