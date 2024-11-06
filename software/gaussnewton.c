#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
/*
高斯牛顿迭代校准传感器数据

步骤
0.获取传感器数据
1.建立校准模型：A=(V-B)*K
	V:加速度计实际输出值 3*1
	K：比例误差矩阵diag{Kx,Ky,Kz}
	A：加速度计理想输出值 3*1
	B: 零漂误差矩阵
    问题：结果发散
    分析：步长太大，即unname过大，每次反馈后更大，需要在某个地方限制步长
    问题原因：雅可比矩阵计算错误，搞成转置了,如果不限制步长也会导致发散
*/
#define M 6
#define g 1
#define ThreshodValue 0.000001

double V[M][3];//实际输出
double K[3][3];//比例误差矩阵
double A[M][3];//理想输出， 平方和=g^2
double B[3];//零漂误差
//p=kb
double P[6];//参数矩阵
double e[M];//误差矩阵


void printMatrix(double matrix[M][M], int n) {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            printf("%lf ", matrix[i][j]);
        }
        printf("\n");
    }
}

void computeResiduals(double V[M][3],double e[M],double P[6]){
	for(int i=0;i<M;i++){
		e[i]=(V[i][0]-P[3])*(V[i][0]-P[3])*P[0]*P[0]+(V[i][1]-P[4])*(V[i][1]-P[4])*P[1]*P[1]+(V[i][2]-P[5])*(V[i][2]-P[5])*P[2]*P[2]-1.0f;
	}
}
//P[Kx,Ky,Kz,Bx,By,Bz]
//
double computeJacobian(double A[M][3],double J[M][M],double P[M]){	
	for(int i=0;i<M;i++){
            J[i][0]= 2*P[0]*(V[i][0]-P[3])*(V[i][0]-P[3]);
            J[i][1]= 2*P[1]*(V[i][1]-P[4])*(V[i][1]-P[4]);
            J[i][2]= 2*P[2]*(V[i][2]-P[5])*(V[i][2]-P[5]);
            J[i][3]=-2*P[0]*P[0]*(V[i][0]-P[3]);
            J[i][4]=-2*P[1]*P[1]*(V[i][1]-P[4]);
            J[i][5]=-2*P[2]*P[2]*(V[i][2]-P[5]);
	}
}

//矩阵求逆
int inverseMatrix(double matrix[M][M], double inverse[M][M], int n) {
    // 创建扩展矩阵
    double augmented[M][2 * M];
    
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            augmented[i][j] = matrix[i][j];
        }
        for (int j = 0; j < n; j++) {
            augmented[i][j + n] = (i == j) ? 1 : 0;  // 单位矩阵
        }
    }

    // 高斯消元
    for (int i = 0; i < n; i++) {
        // 找到主元
        double MEl = fabs(augmented[i][i]);
        int MRow = i;
        for (int k = i + 1; k < n; k++) {
            if (fabs(augmented[k][i]) > MEl) {
                MEl = fabs(augmented[k][i]);
                MRow = k;
            }
        }
        // 交换最大行与当前行
        for (int k = i; k < 2 * n; k++) {
            double tmp = augmented[MRow][k];
            augmented[MRow][k] = augmented[i][k];
            augmented[i][k] = tmp;
        }
        // 将主元变为1
        double divisor = augmented[i][i];
        if (divisor == 0) {
            return 0;  // 矩阵不可逆
        }
        for (int k = 0; k < 2 * n; k++) {
            augmented[i][k] /= divisor;
        }

        // 消去其他行的当前列
        for (int k = 0; k < n; k++) {
            if (k != i) {
                double factor = augmented[k][i];
                for (int j = 0; j < 2 * n; j++) {
                    augmented[k][j] -= factor * augmented[i][j];
                }
            }
        }
    }

    // 提取逆矩阵
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            inverse[i][j] = augmented[i][j];//调整下降方向，已删
        }
    }

    return 1;  // 矩阵逆运算成功
}

void gaussNewtonCalibration(){
    double J[M][M];
    double J_T[M][M];
    double J_T_Mul_J[M][M];
    double J_T_Mul_J_N[M][M];
    double J_T_Mul_e[M];
    double unname[M];//unname=(J_T*J)^-1*J_T*e
    int i=0,j=0,k=0;
    double delt=0;
    //V和A靠手动输入

    //开始迭代
    while(1){
        delt=0;
        for (int i = 0; i < M; i++) {
            for (int j = 0; j < M; j++) {
                J_T_Mul_J[i][j] = 0.0;  // 将每个元素设置为 0
                J[i][j] = 0.0;
            }
        }
        for (j = 0; j < M; j++) {
                J_T_Mul_e[j] = 0.0;  // 将每个元素设置为 0
                unname[j]=0.0;
        }
        
        
        //计算误差函数
        computeResiduals(V,e,P);

        //计算雅可比矩阵
        for(i=0;i<6;i++){
            J[i][0]= 2*P[0]*(V[i][0]-P[3])*(V[i][0]-P[3]);
            J[i][1]= 2*P[1]*(V[i][1]-P[4])*(V[i][1]-P[4]);
            J[i][2]= 2*P[2]*(V[i][2]-P[5])*(V[i][2]-P[5]);
            J[i][3]=-2*P[0]*P[0]*(V[i][0]-P[3]);
            J[i][4]=-2*P[1]*P[1]*(V[i][1]-P[4]);
            J[i][5]=-2*P[2]*P[2]*(V[i][2]-P[5]);
        }
        //printMatrix(J,M);
        //计算转置  
        for (i = 0; i < M; i++) {
            for (j = 0; j < M; j++) {
                J_T[j][i] = J[i][j]; 
            }
        }
        //J的转置*J
        for (i = 0; i < M; i++) {
                for (j = 0; j < M; j++) {
                    J_T_Mul_J[i][j]=0;
                        for (k = 0; k < M; k++) {
                                J_T_Mul_J[i][j] += J_T[i][k] * J[k][j];  
                        }
                }
        }
        //求J_T_Mul_J的逆
        if(inverseMatrix(J_T_Mul_J,J_T_Mul_J_N,M)==0){
            printf("矩阵不可逆\n");
            printMatrix(J,M);

        }
        //J的转置*e
        for (i = 0; i < M; i++) {
             J_T_Mul_e[i]=0;
                for (j = 0; j < M; j++) {
                        J_T_Mul_e[i] += J_T[i][j] * e[j];
                }
        }
        //计算unname
        for (i = 0; i < M; i++) {
            unname[i]=0;
                for (j = 0; j < M; j++) {
                        unname[i] += J_T_Mul_J_N[i][j] * J_T_Mul_e[j];
                }
        }
        //计算迭代后的P矩阵
        for(i=0;i<M;i++){
                P[i]=P[i]-unname[i]*0.01;//需要限制步长
        }
        //todo：结束条件？：步长足够小，定义为unname*unname_T,done
        //问题：每次迭代中，是否需要更新e？用！！
        for(i=0;i<M;i++){
               delt+=unname[i]*unname[i];
        }
        printf("delt:%lf\n",delt);

        // for(int r=0;r<6;r++){
        //     printf("P%d:%f\n",r,P[r]);
        // }
        if(delt<ThreshodValue){
            //todo
            K[0][0]=P[0];
            K[1][1]=P[1];
            K[2][2]=P[2];
            B[0]=P[3];
            B[1]=P[4];
            B[2]=P[5];
            break;
        }
    }
}

/*单独作为一个程序跑，放在在项目里就把main注释了*/

// int main(){
//     // 写入6组实际值和理想值V和A
//     // 平放

//     V[0][0]=0.049926;
//     V[0][1]=-0.025634;
//     V[0][2]=0.9506836;
//     A[0][0]=0;
//     A[0][1]=0;
//     A[0][2]=g;

//     //倒立
//     V[1][0]=0.045043;
//     V[1][1]=-0.001953;
//     V[1][2]=-1.097168;
//     A[1][0]=0;
//     A[1][1]=0;
//     A[1][2]=-g;

//     //左倾
//     V[2][0]=0.046142;
//     V[2][1]=-1.015380;
//     V[2][2]=-0.073242;
//     A[2][0]=0;
//     A[2][1]=-g;
//     A[2][2]=0;

//     //右倾
//     V[3][0]=0.032104;
//     V[3][1]=0.9879150;
//     V[3][2]=-0.064697;
//     A[3][0]=0;
//     A[3][1]=g;
//     A[3][2]=0;

//     //前扑
//     V[4][0]=-0.940429;
//     V[4][1]=-0.023681;
//     V[4][2]=-0.026489;
//     A[4][0]=-g;
//     A[4][1]=0;
//     A[4][2]=0;

//     //后仰
//     V[5][0]=1.020385;
//     V[5][1]=0.038085;
//     V[5][2]=-0.084594;
//     A[5][0]=g;
//     A[5][1]=0;
//     A[5][2]=0;

//     //手动输入一组P值
//     P[0]=1;
//     P[1]=0.98;
//     P[2]=0.956;
//     P[3]=0.04;
//     P[4]=-0.012;
//     P[5]=-0.075;

//     gaussNewtonCalibration();
//     for(int r=0;r<6;r++){
//         printf("P%d:%f\n",r,P[r]);
//     }
// }




/*
/测试逆矩阵计算是否正确
    int i=0,j=0,k=0;
    double w[M][M]={{33037.172982628712, 43102.833310846676, 204.72825462182328, 142.98826325229786, 84.952597136702153, 350.38341080751195}, {43102.833310846676, 58445.711179767852, 
    292.40108622796771, 229.98077248435985, 100.26332502704999, 484.56392403022642}, {204.72825462182328, 292.40108622796771, 42935.034911677387, 39851.418552534335, 
    91.636636303383384, 155.87184559574564}, {142.98826325229786, 229.98077248435985, 39851.418552534335, 38494.390095580951, 37.724163774911396, 131.09477053105479}, {
    84.952597136702153, 100.26332502704999, 91.636636303383384, 37.724163774911396, 31641.402362819717, 36450.293086793594}, {350.38341080751195, 484.56392403022642, 
    155.87184559574564, 131.09477053105479, 36450.293086793594, 43784.862980004007}};
    double wn[M][M]={{0.0008005835803717203, -0.00059046699751382403, -7.7211109130537126e-06, 8.5279297028657995e-06, -1.0148718319331603e-05, 8.5786974824053685e-06}, {-0.00059046699751382403, 
    0.00045262966834266584, 5.0569626626298591e-06, -5.7238134207272592e-06, 1.1501755795153239e-05, -9.8599923381297421e-06}, {-7.7211109130537093e-06, 5.0569626626298557e-06, 
    0.00059613406303500008, -0.00061718059409816329, -1.6506174061865521e-05, 1.347265701855116e-05}, {8.5279297028657944e-06, -5.7238134207272541e-06, -0.00061718059409816308, 
    0.00066495177876952846, 1.8490410330813907e-05, -1.5191688286107387e-05}, {-1.0148718319331629e-05, 1.1501755795153259e-05, -1.6506174061865498e-05, 1.849041033081388e-05, 
    0.00077259113453482619, -0.00064321411362142143}, {8.5786974824053888e-06, -9.8599923381297574e-06, 1.3472657018551138e-05, -1.5191688286107365e-05, -0.00064321411362142143, 
    0.00055834379284222014}};
    double www[6][6];

        for (i = 0; i < M; i++) {
                for (j = 0; j < M; j++) {
                        for (k = 0; k < M; k++) {
                                www[i][j] += w[i][k] * wn[k][j];  
                        }
                }
        }
    printMatrix(www,6);



 P 一组不错的结果？：完美的结果！

P0:1.018627
P1:0.998246
P2:0.976477
P3:0.040124
P4:-0.013710
P5:-0.073242
*/
















