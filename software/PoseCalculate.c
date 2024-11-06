#include "main.h"
/*
步骤
	0.定义四元数，欧拉角，收集角速度，收集加速度，定义deltT，定义旋转矩阵（好像不用）
	1.龙格库塔求微分方程得到四元数
	2.？根据加速度计的值与四元数姿态矩阵中的重力分量误差进行矫正？（融合算法）
	3.四元数归一化？：为了保证微分方程正确解（其中将n*n看作-1）
	else
*/
#define beta 1.0f
#define deltT 0.01f

volatile float q0,q1,q2,q3;//初始值可设为1,0,0,0
float q0Acc,q1Acc,q2Acc,q3Acc;//初始值可设为1,0,0,0
float q0gyro,q1gyro,q2gyro,q3gyro;//初始值可设为1,0,0,0
double yaw,pitch,roll;

gyro gyroData;//test
acc accData;//test

float invSqrt(float x);
void qAccCompute(float *q0Acc,float *q1Acc,float *q2Acc,float *q3Acc);
void updateQuaternion(float *q0gyro,float *q1gyro,float *q2gyro,float *q3gyro);
//每次更新前需要做归一乎？或者更新后第一时间做归一也可以！



void updateAngleTmp(float *q0,float *q1,float *q2,float *q3){
	
	double q0q1,q0q2,q0q3,q1q2,q1q3,q2q3,q1q1,q2q2,q3q3;
	q0q1 = (*q0)*(*q1);
	q0q2 = (*q0)*(*q2);
	q0q3 = (*q0)*(*q3);
	q1q2 = (*q1)*(*q2);
	q1q3 = (*q1)*(*q3);
	q2q3 = (*q2)*(*q3);
	q1q1 = (*q1)*(*q1);
	q2q2 = (*q2)*(*q2);
	q3q3 = (*q3)*(*q3);
	
	yaw = atan(2*(q0q3+q1q2)/(1-2*(q3q3+q2q2)));
	pitch = asin(2*(q0q2-q1q3));
	roll = atan(2*(q0q1+q2q3)/(1-2*(q1q1+q2q2)));

}


//
void madgwick(float *q0,float *q1,float *q2,float *q3){
	
	//梯度下降
	int i=0,j=0,k=0;
	float recipNorm;//平方和开根号分之1
	float grad[4];
	float s0, s1, s2, s3;
	float qDot[4];
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	
	qDot[0] = (-gyroData.x*(*q1)-gyroData.y*(*q2)-gyroData.z*(*q3))*0.5f;
	qDot[1] = ( gyroData.x*(*q0)-gyroData.y*(*q3)+gyroData.z*(*q2))*0.5f;
	qDot[2] = ( gyroData.x*(*q3)+gyroData.y*(*q0)-gyroData.z*(*q1))*0.5f;
	qDot[3] = (-gyroData.x*(*q2)+gyroData.y*(*q1)+gyroData.z*(*q0))*0.5f;
	
	if(!((accData.x == 0.0f) && (accData.y == 0.0f) && (accData.z == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(accData.x * accData.x + accData.y * accData.y + accData.z * accData.z);
		accData.x *= recipNorm;
		accData.y *= recipNorm;
		accData.z *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * (*q0);
		_2q1 = 2.0f * (*q1);
		_2q2 = 2.0f * (*q2);
		_2q3 = 2.0f * (*q3);
		_4q0 = 4.0f * (*q0);
		_4q1 = 4.0f * (*q1);
		_4q2 = 4.0f * (*q2);
		_8q1 = 8.0f * (*q1);
		_8q2 = 8.0f * (*q2);
		q0q0 = (*q0) * (*q0);
		q1q1 = (*q1) * (*q1);
		q2q2 = (*q2) * (*q2);
		q3q3 = (*q3) * (*q3);

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * accData.x + _4q0 * q1q1 - _2q1 * accData.y;
		s1 = _4q1 * q3q3 - _2q3 * accData.x + 4.0f * q0q0 * (*q1) - _2q0 * accData.y - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * accData.z;
		s2 = 4.0f * q0q0 * (*q2) + _2q0 * accData.x + _4q2 * q3q3 - _2q3 * accData.y - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * accData.z;
		s3 = 4.0f * q1q1 * (*q3) - _2q1 * accData.x + 4.0f * q2q2 * (*q3) - _2q2 * accData.y;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot[0] -= beta * s0;
		qDot[1] -= beta * s1;
		qDot[2] -= beta * s2;
		qDot[3] -= beta * s3;
	}      


 
	              
	*q0 += qDot[0]*deltT;
	*q1 += qDot[1]*deltT;
	*q2 += qDot[2]*deltT;
	*q3 += qDot[3]*deltT;
//	
//	//归一化

	recipNorm = invSqrt((*q0) * (*q0) + (*q1) * (*q1) + (*q2) * (*q2) + (*q3) * (*q3));
	*q0 *= recipNorm;
	*q1 *= recipNorm;
	*q2 *= recipNorm;
	*q3 *= recipNorm;
//		for (int i = 0; i < 4; i++) {
//			for (int j = 0; j < 1; j++) {	
//					Serial_Printf("%f - %f ",qDot[i], grad[i]);	
//			}	
//			Serial_Printf("\n");	
//		}	
}

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


//void qAccCompute(float *q0Acc,float *q1Acc,float *q2Acc,float *q3Acc){
//	//梯度下降
//	//
//	int i=0,j=0,k=0;
//	
//	float P[4];
//	float e[3];
//	float grad[4];
//	
//	P[0] = *q0Acc;
//	P[1] = *q1Acc;
//	P[2] = *q2Acc;
//	P[3] = *q3Acc;
//	
//	e[0] = 2*P[1]*P[3]+2*P[0]*P[2]-accData.x;
//	e[1] = 2*P[2]*P[3]-2*P[0]*P[1]-accData.y;
//	e[2] = 1-2*P[1]*P[1]-2*P[2]*P[2]-accData.z;
//	
//	float J[3][4],J_T[4][3];
//	
//	J[0][0] = 2*P[2];
//	J[0][1] = 0;
//	J[0][2] = 2*P[0];
//	J[0][3] = 2*P[1];
//	J[1][0] = -2*P[1];
//	J[1][1] = -2*P[0];
//	J[1][2] = 2*P[3];
//	J[1][3] = 2*P[2];
//	J[2][0] = 0;
//	J[2][1] = -4*P[1];
//	J[2][2] = -4*P[2];
//	J[2][3] = 0;
//	
//	for (i = 0; i < 3; i++) {
//     for (j = 0; j < 4; j++) {
//         J_T[j][i] = J[i][j]; 
//     }
//  }
//	
//	//计算梯度
//	for (i = 0; i < 4; i++) {
//				grad[i]=0;
//					for (j = 0; j < 3; j++) {
//									grad[i] += J_T[i][j] * e[j];
//					}
//	}
//	//梯度单位化
//	float quadSum;//平方和
//	quadSum = (float)sqrt(grad[0] * grad[0] + grad[1] * grad[1] + grad[2] * grad[2] + grad[3] * grad[3]);
//	grad[0] /= quadSum;
//	grad[1] /= quadSum;
//	grad[2] /= quadSum;
//	grad[3] /= quadSum;
//	Serial_Printf("sum:%f\r\n",quadSum);

////	   for (int i = 0; i < 4; i++) {
////        for (int j = 0; j < 1; j++) {
////            Serial_Printf("%f ", grad[i]);
////        }
////        Serial_Printf("\n");
////    }
////	
//	
////	*q0Acc = P[0]-0.1*grad[0];
////	*q1Acc = P[1]-0.1*grad[1];
////	*q2Acc = P[2]-0.1*grad[2];
////	*q3Acc = P[3]-0.1*grad[3];

//	*/

//}

//void updateQuaternion(float *q0gyro,float *q1gyro,float *q2gyro,float *q3gyro){
//	*q0gyro += (-gyroData.x*(*q1gyro)-gyroData.y*(*q2gyro)-gyroData.z*(*q3gyro))*deltT/2*0.01;
//	*q1gyro += (gyroData.x*(*q0gyro)-gyroData.y*(*q3gyro)+gyroData.z*(*q2gyro))*deltT/2*0.01;
//	*q2gyro += (gyroData.x*(*q3gyro)+gyroData.y*(*q0gyro)-gyroData.z*(*q1gyro))*deltT/2*0.01;
//	*q3gyro += (-gyroData.x*(*q2gyro)+gyroData.y*(*q1gyro)+gyroData.z*(*q0gyro))*deltT/2*0.01;
//	
//	//归一化
//	float quadSum;//平方和
//	quadSum = sqrt((*q0gyro) * (*q0gyro) + (*q1gyro) * (*q1gyro) + (*q2gyro) * (*q2gyro) + (*q3gyro) * (*q3gyro));
//	*q0gyro /= quadSum;
//	*q1gyro /= quadSum;
//	*q2gyro /= quadSum;
//	*q3gyro /= quadSum;
//	
//}

//	if(!((accData.x == 0.0f) && (accData.y == 0.0f) && (accData.z == 0.0f))){
//			P[0] = *q0;
//			P[1] = *q1;
//			P[2] = *q2;
//			P[3] = *q3;
//			
//			e[0] =   2*P[1]*P[3]+2*P[0]*P[2]-accData.x;
//			e[1] =   2*P[2]*P[3]-2*P[0]*P[1]-accData.y;
//			e[2] = 1-2*P[1]*P[1]-2*P[2]*P[2]-accData.z;
//			
//			float J[3][4],J_T[4][3];
//			
//			J[0][0] = 2*P[2];
//			J[0][1] = 2*P[3];
//			J[0][2] = 2*P[0];
//			J[0][3] = 2*P[1];
//			
//			J[1][0] = -2*P[1];
//			J[1][1] = -2*P[0];
//			J[1][2] = 2*P[3];
//			J[1][3] = 2*P[2];
//			
//			J[2][0] = 0;
//			J[2][1] = -4*P[1];
//			J[2][2] = -4*P[2];
//			J[2][3] = 0;
//			
//			for (i = 0; i < 3; i++) {
//				 for (j = 0; j < 4; j++) {
//						 J_T[j][i] = J[i][j]; 
//				 }
//			}
//			
//			//计算梯度
//			for (i = 0; i < 4; i++) {
//						grad[i]=0;
//							for (j = 0; j < 3; j++) {
//											grad[i] += J_T[i][j] * e[j];
//							}
//			}
//			//梯度单位化

//			quadSum = (float)sqrt(grad[0] * grad[0] + grad[1] * grad[1] + grad[2] * grad[2] + grad[3] * grad[3]);
//			grad[0] /= quadSum;
//			grad[1] /= quadSum;
//			grad[2] /= quadSum;	
//			grad[3] /= quadSum;
//	}