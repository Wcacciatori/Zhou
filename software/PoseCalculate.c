#include "main.h"
/*
步骤
	0.定义四元数，欧拉角，收集角速度，收集加速度，定义deltT
	1.龙格库塔求微分方程得到四元数
	2.根据加速度计的值与四元数姿态矩阵中的重力分量误差进行矫正（融合算法）:用来算加速度计对姿态四元数的影响（梯度）
	3.四元数归一化？：为了保证微分方程正确解（其中将n*n看作-1）
	else
*/
//#define beta 0.8f     //越小越接近陀螺仪，即越稳定但是漂移同时越严重
//#define deltT 0.05f  //越小越不晃，但是角度变化量跟不上，越大反应越快
#define PI 3.14159265358979323846

#define sampleFreq	150.0f		// sample frequency in Hz
#define betaDef		0.8f		// 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float beta = betaDef;								// 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

float yaw,pitch,roll;

float rollOffset = -0.013094f, pitchOffset = -7.447712f;

gyro gyroData;//test
volatile acc accData;//test
mag magData;//test

float invSqrt(float x);
void qAccCompute(float *q0Acc,float *q1Acc,float *q2Acc,float *q3Acc);
void updateQuaternion(float *q0gyro,float *q1gyro,float *q2gyro,float *q3gyro);
//每次更新前需要做归一乎？或者更新后第一时间做归一也可以！

//四元数转欧拉角
void AttitudeSolver_GetEulerAngles(float *roll, float *pitch, float *yaw) {
    *roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.2958f;  // ?????
    *pitch = asinf(2.0f * (q0 * q2 - q3 * q1)) * 57.2958f;
    *yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 57.2958f;
}


//
//void madgwick(float *q0,float *q1,float *q2,float *q3){
//	
//	//梯度下降
//	int i=0,j=0,k=0;
//	float recipNorm;//平方和开根号分之1
//	float grad[4];
//	float s0, s1, s2, s3;
//	float qDot[4];
//	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
//	
//	qDot[0] = (-gyroData.x*(*q1)-gyroData.y*(*q2)-gyroData.z*(*q3))*0.5f;
//	qDot[1] = ( gyroData.x*(*q0)-gyroData.y*(*q3)+gyroData.z*(*q2))*0.5f;
//	qDot[2] = ( gyroData.x*(*q3)+gyroData.y*(*q0)-gyroData.z*(*q1))*0.5f;
//	qDot[3] = (-gyroData.x*(*q2)+gyroData.y*(*q1)+gyroData.z*(*q0))*0.5f;
//	
//	if(!((accData.x == 0.0f) && (accData.y == 0.0f) && (accData.z == 0.0f))) {

//		// Normalise accelerometer measurement
//		recipNorm = invSqrt(accData.x * accData.x + accData.y * accData.y + accData.z * accData.z);
//		accData.x *= recipNorm;
//		accData.y *= recipNorm;
//		accData.z *= recipNorm;   

//		// Auxiliary variables to avoid repeated arithmetic
//		_2q0 = 2.0f * (*q0);
//		_2q1 = 2.0f * (*q1);
//		_2q2 = 2.0f * (*q2);
//		_2q3 = 2.0f * (*q3);
//		_4q0 = 4.0f * (*q0);
//		_4q1 = 4.0f * (*q1);
//		_4q2 = 4.0f * (*q2);
//		_8q1 = 8.0f * (*q1);
//		_8q2 = 8.0f * (*q2);
//		q0q0 = (*q0) * (*q0);
//		q1q1 = (*q1) * (*q1);
//		q2q2 = (*q2) * (*q2);
//		q3q3 = (*q3) * (*q3);

//		// Gradient decent algorithm corrective step
//		s0 = _4q0 * q2q2 + _2q2 * accData.x + _4q0 * q1q1 - _2q1 * accData.y;
//		s1 = _4q1 * q3q3 - _2q3 * accData.x + 4.0f * q0q0 * (*q1) - _2q0 * accData.y - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * accData.z;
//		s2 = 4.0f * q0q0 * (*q2) + _2q0 * accData.x + _4q2 * q3q3 - _2q3 * accData.y - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * accData.z;
//		s3 = 4.0f * q1q1 * (*q3) - _2q1 * accData.x + 4.0f * q2q2 * (*q3) - _2q2 * accData.y;
//		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
//		s0 *= recipNorm;
//		s1 *= recipNorm;
//		s2 *= recipNorm;
//		s3 *= recipNorm;

//		// Apply feedback step
//		qDot[0] -= beta * s0;
//		qDot[1] -= beta * s1;
//		qDot[2] -= beta * s2;
//		qDot[3] -= beta * s3;
//	}      


// 
//	              
//	*q0 += qDot[0]*deltT;
//	*q1 += qDot[1]*deltT;
//	*q2 += qDot[2]*deltT;
//	*q3 += qDot[3]*deltT;
////	
////	//归一化

//	recipNorm = invSqrt((*q0) * (*q0) + (*q1) * (*q1) + (*q2) * (*q2) + (*q3) * (*q3));
//	*q0 *= recipNorm;
//	*q1 *= recipNorm;
//	*q2 *= recipNorm;
//	*q3 *= recipNorm;
////		for (int i = 0; i < 4; i++) {
////			for (int j = 0; j < 1; j++) {	
////					Serial_Printf("%f - %f ",qDot[i], grad[i]);	
////			}	
////			Serial_Printf("\n");	
////		}	
//	double q0q1,q0q2,q0q3,q1q2,q1q3,q2q3;
//	q0q1 = (*q0)*(*q1);
//	q0q2 = (*q0)*(*q2);
//	q0q3 = (*q0)*(*q3);
//	q1q2 = (*q1)*(*q2);
//	q1q3 = (*q1)*(*q3);
//	q2q3 = (*q2)*(*q3);
//	q0q0 = (*q0)*(*q0);
//	q1q1 = (*q1)*(*q1);
//	q2q2 = (*q2)*(*q2);
//	q3q3 = (*q3)*(*q3);
//	
//	pitch = atan2(2*(q0q1+q2q3), 1-2*q2q2-2*q1q1);	
//	roll = asin(2*(q0q2-q1q3));
//	yaw = atan2(2*(q0q3+q1q2), (1-2*(q3q3+q2q2)));
////p其实是yaw，roll其实是pi
////		yaw *= (180.0/PI);
////		pitch *= 0;
////		roll *= 0;	
//	yaw *= (180.0/PI);
//	pitch *= (180.0/PI);
//	roll *= (180.0/PI);
//}

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		// usart_send("use IMU algorithm\n");
		MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
	// const float left_nums = 100000;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;
		// Apply feedback step
        // if (print_rate %  print_per_time == 0){
		// 	usart_send("gyro:      qDot1: %7d,qDot2: %7d,qDot3: %7d,qDot4: %7d\n",(int)(qDot1*left_nums),(int)(qDot2*left_nums),(int)(qDot3*left_nums),(int)(qDot4*left_nums));
		// 	usart_send("s0123:     qDot1: %7d,qDot2: %7d,qDot3: %7d,qDot4: %7d\n",(int)(- beta * s0 *left_nums),(int)(- beta * s1 *left_nums),(int)(- beta * s2 *left_nums),(int)(- beta * s3 *left_nums));
		// }
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
        // if (print_rate %  print_per_time == 0){
        //     print_rate = 0;
		// 	usart_send("copensate: qDot1: %7d,qDot2: %7d,qDot3: %7d,qDot4: %7d\n",(int)(qDot1*left_nums),(int)(qDot2*left_nums),(int)(qDot3*left_nums),(int)(qDot4*left_nums));
		// }
		// print_rate++;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
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