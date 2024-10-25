#include "main.h"

/*
高斯牛顿迭代校准传感器数据

步骤：
0.获取传感器数据
1.建立校准模型：V=KA+B
	V：加速度计实际输出的值 3*1
	K：比例误差矩阵diag{Kx,Ky,Kz}
	A：加速度计理想输出的值 3*1
	B: 零漂误差矩阵
2.模型更新为：A=M（V-B）
	M=(K)^-1
3.定义误差e(k)=Vx(k)^2+Vy(k)^2+Vz(k)^2-g^2
4.定义函数S=每次测量误差的累加和
	
*/
#define M 6
#define g 9.8
#define ThreshodValue 0.000001

double V[M][3];//实际输出
double K[3][3];//比例误差
double A[M][3];//理想输出  平方和=g^2
double B[3];//零漂误差
//p=kb
double P[6];//参数矩阵
double e[M];//误差矩阵

void computeResiduals(double V[M][3],double e[M]){
	for(int i=0;i<M;i++){
		e[i]=V[i][0]*V[i][0]+V[i][1]*V[i][1]+V[i][2]*V[i][2]-g*g;
	}
}
//P[Kx,Ky,Kz,Bx,By,Bz]
//参数1：理想输出矩阵6*3 
void computeJacobian(double A[M][3],double J[M][M],double P[M]){	
	for(int i=0;i<M;i++){
		J[0][i]=2*A[i][0]*A[i][0]*P[0]+2*P[3]*A[i][0];
		J[1][i]=2*A[i][1]*A[i][1]*P[1]+2*P[4]*A[i][1];
		J[2][i]=2*A[i][2]*A[i][2]*P[2]+2*P[5]*A[i][2];
		J[3][i]=2*P[3]+2*A[i][0]*P[0];
		J[4][i]=2*P[4]+2*A[i][1]*P[1];
		J[5][i]=2*P[5]+2*A[i][2]*P[2];
	}
}

void gaussNewtonCalibration(){
		double J[M][M];
		double J_T[M][M];
		double J_T_Mul_J[M][M];
		double J_T_Mul_J_N[M][M];
		double J_T_Mul_e[M];
		double unname[M];//unname=(J_T*J)^-1*J_T*e
		int i=0,j=0,k=0;
		//V和A靠手动输入
		computeResiduals(V,e);
		//手动输入一组初始P值
		//开始迭代
		while(e[0]>ThreshodValue||e[1]>ThreshodValue||e[2]>ThreshodValue||e[3]>ThreshodValue||e[4]>ThreshodValue||e[5]>ThreshodValue){
			//计算雅可比矩阵
			for(i=0;i<6;i++){
				computeJacobian(A,J,P);
			}
			//计算转置
			for (i = 0; i < M; i++) {
        for (j = 0; j < M; j++) {
            J_T[j][i] = J[i][j]; 
        }
			}
			//J的转置*J
			for (i = 0; i < M; i++) {
					for (j = 0; j < M; j++) {
							for (k = 0; k < M; k++) {
									J_T_Mul_J[i][j] += J_T[i][k] * J[k][j];  
							}
					}
			}
			//求J_T_Mul_J的逆  todo!!!
			
			//J的转置*e
			for (i = 0; i < M; i++) {
					for (j = 0; j < M; j++) {
							J_T_Mul_e[i] += J_T[i][j] * e[j];
					}
			}
			//计算unname
			for (i = 0; i < M; i++) {
					for (j = 0; j < M; j++) {
							unname[i] += J_T_Mul_J_N[i][j] * J_T_Mul_e[j];
					}
			}
			//计算迭代后的P矩阵
			for(i=0;i<M;i++){
					P[i]=P[i]-unname[i];
			}

}























