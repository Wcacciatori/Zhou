#include "main.h"

/*
��˹ţ�ٵ���У׼����������

���裺
0.��ȡ����������
1.����У׼ģ�ͣ�V=KA+B
	V�����ٶȼ�ʵ�������ֵ 3*1
	K������������diag{Kx,Ky,Kz}
	A�����ٶȼ����������ֵ 3*1
	B: ��Ư������
2.ģ�͸���Ϊ��A=M��V-B��
	M=(K)^-1
3.�������e(k)=Vx(k)^2+Vy(k)^2+Vz(k)^2-g^2
4.���庯��S=ÿ�β��������ۼӺ�
	
*/
#define M 6
#define g 9.8
#define ThreshodValue 0.000001

double V[M][3];//ʵ�����
double K[3][3];//�������
double A[M][3];//�������  ƽ����=g^2
double B[3];//��Ư���
//p=kb
double P[6];//��������
double e[M];//������

void computeResiduals(double V[M][3],double e[M]){
	for(int i=0;i<M;i++){
		e[i]=V[i][0]*V[i][0]+V[i][1]*V[i][1]+V[i][2]*V[i][2]-g*g;
	}
}
//P[Kx,Ky,Kz,Bx,By,Bz]
//����1�������������6*3 
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
		//V��A���ֶ�����
		computeResiduals(V,e);
		//�ֶ�����һ���ʼPֵ
		//��ʼ����
		while(e[0]>ThreshodValue||e[1]>ThreshodValue||e[2]>ThreshodValue||e[3]>ThreshodValue||e[4]>ThreshodValue||e[5]>ThreshodValue){
			//�����ſɱȾ���
			for(i=0;i<6;i++){
				computeJacobian(A,J,P);
			}
			//����ת��
			for (i = 0; i < M; i++) {
        for (j = 0; j < M; j++) {
            J_T[j][i] = J[i][j]; 
        }
			}
			//J��ת��*J
			for (i = 0; i < M; i++) {
					for (j = 0; j < M; j++) {
							for (k = 0; k < M; k++) {
									J_T_Mul_J[i][j] += J_T[i][k] * J[k][j];  
							}
					}
			}
			//��J_T_Mul_J����  todo!!!
			
			//J��ת��*e
			for (i = 0; i < M; i++) {
					for (j = 0; j < M; j++) {
							J_T_Mul_e[i] += J_T[i][j] * e[j];
					}
			}
			//����unname
			for (i = 0; i < M; i++) {
					for (j = 0; j < M; j++) {
							unname[i] += J_T_Mul_J_N[i][j] * J_T_Mul_e[j];
					}
			}
			//����������P����
			for(i=0;i<M;i++){
					P[i]=P[i]-unname[i];
			}

}























