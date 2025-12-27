 /******************************************************************************************************************************************************************************************************\
 ***
 *** Description       : IMPLEMENTATION OF THE ADDITIVE NOISE UKF: Damped pendulum.
 *** Codefile          : ukfCfg1.c
 *** Documentation     : https://github.com/ivo-georgiev/ukfLib/wiki/Damped-pendulum:-CFG1
 ***
 ***
 *** MIT License
 ***
 *** Copyright (c) 2017 ivo-georgiev
 ***
 *** Permission is hereby granted, free of charge, to any person obtaining a copy
 *** of this software and associated documentation files (the "Software"), to deal
 *** in the Software without restriction, including without limitation the rights
 *** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *** copies of the Software, and to permit persons to whom the Software is
 *** furnished to do so, subject to the following conditions:
 ***
 *** The above copyright notice and this permission notice shall be included in all
 *** copies or substantial portions of the Software.
 ***
 *** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *** OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *** SOFTWARE.
\******************************************************************************************************************************************************************************************************/
#include "ukfCfg1.h"

static void Fx1(Matrix_t *pu_p, Matrix_t *pX_p, Matrix_t *pX_m, int sigmaIdx, double dT);
static void Fx2(Matrix_t* pu_p, Matrix_t* pX_p, Matrix_t* pX_m, int sigmaIdx, double dT);
static void Fx3(Matrix_t* pu_p, Matrix_t* pX_p, Matrix_t* pX_m, int sigmaIdx, double dT);
static void Fx4(Matrix_t* pu_p, Matrix_t* pX_p, Matrix_t* pX_m, int sigmaIdx, double dT);
static void Fx5(Matrix_t* pu_p, Matrix_t* pX_p, Matrix_t* pX_m, int sigmaIdx, double dT);

static void Hy1(Matrix_t* pu, Matrix_t* pX_m, Matrix_t* pY_m, int sigmaIdx);
static void Hy2(Matrix_t* pu, Matrix_t* pX_m, Matrix_t* pY_m, int sigmaIdx);
static void Hy3(Matrix_t* pu, Matrix_t* pX_m, Matrix_t* pY_m, int sigmaIdx);

static PredictFcn_t PredictFcn[5] = { &Fx1, &Fx2, &Fx3, &Fx4, &Fx5 };
static ObservFcn_t ObservFcn[3] = { &Hy1, &Hy2, &Hy3 };

//-----------------------
//UKF Processing matrix
//-----------------------
static double Sc_vector[1][3] = {{0.1,2,0}};
static double Wm_weight_vector[1][11] = {{0,0,0,0,0,0,0,0,0,0,0}};
static double Wc_weight_vector[1][11] = {{0,0,0,0,0,0,0,0,0,0,0}};
//static double u_system_input[2][1] = {{0},{0}};
//static double u_prev_system_input[2][1] = {{0},{0}};
static double y_meas[3][1] = {{0}, {0}, {0}};
static double y_predicted_mean[3][1] = {{0}, {0}, {0}};
static double x_system_states[5][1] = {{0},{0},{0},{0},{0}};
static double x_system_states_ic[5][1] = {{0},{0},{0},{0},{0}};
static double x_system_states_limits[5][3] = { {0, 0, NULL}, {0, 0, NULL}, {0, 0, NULL}, {0, 0, NULL}, {0, 0, NULL} };
static _Bool x_system_states_limits_enable[5][1] = { {0},{0},{0},{0},NULL };
static double x_system_states_correction[5][1] = {{0},{0},{0},{0},{0}};
static double X_sigma_points[5][11] =
{/*  s1  s2  s3  s4  s5}       */
    {0,  0,  0,  0,  0, 0, 0, 0, 0, 0, 0}, /* x1 */
    {0,  0,  0,  0,  0, 0, 0, 0, 0, 0, 0}, /* x2 */
    {0,  0,  0,  0,  0, 0, 0, 0, 0, 0, 0}, /* x3 */
    {0,  0,  0,  0,  0, 0, 0, 0, 0, 0, 0}, /* x4 */
    {0,  0,  0,  0,  0, 0, 0, 0, 0, 0, 0}, /* x5 */
};
//Sigma points Y(k|k-1) = y_m
static double Y_sigma_points[3][11]=
{/*  s1  s2  s3  s4  s5  */
    {0,  0,  0,  0,  0, 0, 0, 0, 0, 0, 0}, /* y1 */
    {0,  0,  0,  0,  0, 0, 0, 0, 0, 0, 0}, /* y2 */
    {0,  0,  0,  0,  0, 0, 0, 0, 0, 0, 0}, /* y3 */
};

//State covariance  P(k|k-1) = P_m, P(k)= P
static double Pxx_error_covariance[5][5]=
{/*  x1, x2, x3, x4, x5       */
    {0,  0, 0, 0, 0}, /* x1 */
    {0,  0, 0, 0, 0}, /* x2 */
    {0,  0, 0, 0, 0}, /* x3 */
    {0,  0, 0, 0, 0}, /* x4 */
    {0,  0, 0, 0, 0}, /* x5 */
};

//State covariance initial values
static double Pxx0_init_error_covariance[5][5]=
{/*  x1,    x2,    x3,    x4,    x5         */
    {1,    0,     0,     0,     0}, /* x1 */
    {0,    1,     0,     0,     0}, /* x2 */
    {0,    0,     1,     0,     0}, /* x3 */
    {0,    0,     0,     1,     0}, /* x4 */
    {0,    0,     0,     0,     1}, /* x5 */
};

//Process noise covariance Q : initial noise assumptions
static double Qxx_process_noise_cov[5][5]=
{/*  x1,  x2,  x3,  x4,  x5        */
    { 0.1,      0,      0,      0,      0}, /* x1 */
    {   0,    0.1,      0,      0,      0}, /* x2 */
    {   0,      0,   0.01,      0,      0}, /* x3 */
    {   0,      0,      0,  0.001,      0}, /* x4 */
    {   0,      0,      0,      0,   0.01}, /* x5 */
};

//Output noise covariance: initial noise assumptions
static double Ryy0_init_out_covariance[3][3]=
{/*  y1,      */
    {0.01, 0,      0    },  /* y1 */
    {0,    0.001, 0    },  /* y2 */
    {0,    0,      0.5},/* y3 */
};

//Output covariance Pyy = R (initial assumption)
static double Pyy_out_covariance[3][3]=
{/*  y1        */
    {1, 0, 0},  /* y1 */
    {0, 1, 0},  /* y2 */
    {0, 0, 1},  /* y3 */
};

static double Pyy_out_covariance_copy[3][3]=
{/*  y1         */
    {1, 0, 0},  /* y1 */
    {0, 1, 0},  /* y2 */
    {0, 0, 1},  /* y3 */
};

//cross-covariance of state and output
static double Pxy_cross_covariance[5][3] =
{/*  y1        */
    {0, 0, 0},  /* x1 */
    {0, 0, 0},  /* x2 */
    {0, 0, 0},  /* x3 */
    {0, 0, 0},  /* x4 */
    {0, 0, 0},  /* x5 */
};

//Kalman gain matrix
static double K_kalman_gain[5][3] = 
{/*  y1        */
    {0, 0, 0},  /* x1 */
    {0, 0, 0},  /* x2 */
    {0, 0, 0},  /* x3 */
    {0, 0, 0},  /* x4 */
    {0, 0, 0},  /* x5 */
};

static double Pxx_covariance_correction[5][5]=
{/*  x1, x2, x3, x4, x5       */
    {0,  0,  0,  0,  0}, /* x1 */
    {0,  0,  0,  0,  0}, /* x2 */
    {0,  0,  0,  0,  0}, /* x3 */
    {0,  0,  0,  0,  0}, /* x4 */
    {0,  0,  0,  0,  0}, /* x5 */
};

static double I_identity_matrix[3][3]={{1,0,0},{0,1,0},{0,0,1}};

UkfMatrix_t UkfMatrixCfg1 =
{
    {COLXROW(Sc_vector),NROWS(Sc_vector),NCOL(Sc_vector),&Sc_vector[0][0]},
    {COLXROW(Wm_weight_vector),NROWS(Wm_weight_vector),NCOL(Wm_weight_vector),&Wm_weight_vector[0][0]},
    {COLXROW(Wc_weight_vector),NROWS(Wc_weight_vector),NCOL(Wc_weight_vector),&Wc_weight_vector[0][0]},
    {COLXROW(x_system_states),NROWS(x_system_states),NCOL(x_system_states),&x_system_states[0][0]},
    {COLXROW(x_system_states_ic),NROWS(x_system_states_ic),NCOL(x_system_states_ic),&x_system_states_ic[0][0]},
    {COLXROW(x_system_states_limits),NROWS(x_system_states_limits),NCOL(x_system_states_limits),&x_system_states_limits[0][0]},
    {COLXROW(x_system_states_limits_enable),NROWS(x_system_states_limits_enable),NCOL(x_system_states_limits_enable),&x_system_states_limits_enable[0][0]},
    {COLXROW(x_system_states_correction),NROWS(x_system_states_correction),NCOL(x_system_states_correction),&x_system_states_correction[0][0]},
    {0,0,0,NULL},//{COLXROW(u_system_input),NROWS(u_system_input),NCOL(u_system_input),&u_system_input[0][0]},
    {0,0,0,NULL},//{COLXROW(u_prev_system_input),NROWS(u_prev_system_input),NCOL(u_prev_system_input),&u_prev_system_input[0][0]},
    {COLXROW(X_sigma_points),NROWS(X_sigma_points),NCOL(X_sigma_points),&X_sigma_points[0][0]},
    {COLXROW(Y_sigma_points),NROWS(Y_sigma_points),NCOL(Y_sigma_points),&Y_sigma_points[0][0]},
    {COLXROW(y_predicted_mean),NROWS(y_predicted_mean),NCOL(y_predicted_mean),&y_predicted_mean[0][0]},
    {COLXROW(y_meas),NROWS(y_meas),NCOL(y_meas),&y_meas[0][0]},
    {COLXROW(Pyy_out_covariance),NROWS(Pyy_out_covariance),NCOL(Pyy_out_covariance),&Pyy_out_covariance[0][0]},
    {COLXROW(Pyy_out_covariance_copy),NROWS(Pyy_out_covariance_copy),NCOL(Pyy_out_covariance_copy),&Pyy_out_covariance_copy[0][0]},
    {COLXROW(Ryy0_init_out_covariance),NROWS(Ryy0_init_out_covariance),NCOL(Ryy0_init_out_covariance),&Ryy0_init_out_covariance[0][0]},
    {COLXROW(Pxy_cross_covariance),NROWS(Pxy_cross_covariance),NCOL(Pxy_cross_covariance),&Pxy_cross_covariance[0][0]},
    {COLXROW(Pxx_error_covariance),NROWS(Pxx_error_covariance),NCOL(Pxx_error_covariance),&Pxx_error_covariance[0][0]},
    {COLXROW(Pxx0_init_error_covariance),NROWS(Pxx0_init_error_covariance),NCOL(Pxx0_init_error_covariance),&Pxx0_init_error_covariance[0][0]},
    {COLXROW(Qxx_process_noise_cov),NROWS(Qxx_process_noise_cov),NCOL(Qxx_process_noise_cov),&Qxx_process_noise_cov[0][0]},
    {COLXROW(K_kalman_gain),NROWS(K_kalman_gain),NCOL(K_kalman_gain),&K_kalman_gain[0][0]},
    {COLXROW(I_identity_matrix),NROWS(I_identity_matrix),NCOL(I_identity_matrix),&I_identity_matrix[0][0]},
    {COLXROW(Pxx_covariance_correction),NROWS(Pxx_covariance_correction),NCOL(Pxx_covariance_correction),&Pxx_covariance_correction[0][0]},
    &PredictFcn[0],
    &ObservFcn[0],
    0.005
};


// Fx1: 预测状态0: x位置
void Fx1(Matrix_t *pu_p, Matrix_t *pX_p, Matrix_t *pX_m, int sigmaIdx, double dT) {
    const int nCol = pX_m->ncol; // 现在应该是11
    float px = pX_p->val[nCol * 0 + sigmaIdx];
    float py = pX_p->val[nCol * 1 + sigmaIdx];
    float v  = pX_p->val[nCol * 2 + sigmaIdx];
    float psi = pX_p->val[nCol * 3 + sigmaIdx];
    float omega = pX_p->val[nCol * 4 + sigmaIdx];

    if (fabs(omega) < 1e-4) { // 近似直线运动
        pX_m->val[nCol * 0 + sigmaIdx] = px + v * cos(psi) * dT;
    } else { // 圆弧运动
        pX_m->val[nCol * 0 + sigmaIdx] = px + (v/omega) * (sin(psi + omega*dT) - sin(psi));
    }
    // 注意：这里假设pu_p（控制输入）未使用，原示例也是如此
}

// Fx2: 预测状态1: y位置 (原理同Fx1，将cos换成sin，符号有变化)
void Fx2(Matrix_t *pu_p, Matrix_t *pX_p, Matrix_t *pX_m, int sigmaIdx, double dT) {
    const int nCol = pX_m->ncol;
    float px = pX_p->val[nCol * 0 + sigmaIdx];
    float py = pX_p->val[nCol * 1 + sigmaIdx];
    float v  = pX_p->val[nCol * 2 + sigmaIdx];
    float psi = pX_p->val[nCol * 3 + sigmaIdx];
    float omega = pX_p->val[nCol * 4 + sigmaIdx];

    if (fabs(omega) < 1e-4) {
        pX_m->val[nCol * 1 + sigmaIdx] = py + v * sin(psi) * dT;
    } else {
        pX_m->val[nCol * 1 + sigmaIdx] = py + (v/omega) * (-cos(psi + omega*dT) + cos(psi));
    }
}

// Fx3: 预测状态2: 速度 v (CTRV模型中，预测阶段速度不变)
void Fx3(Matrix_t *pu_p, Matrix_t *pX_p, Matrix_t *pX_m, int sigmaIdx, double dT) {
    const int nCol = pX_m->ncol;
    pX_m->val[nCol * 2 + sigmaIdx] = pX_p->val[nCol * 2 + sigmaIdx]; // v不变
    pu_p = pu_p; dT = dT; // 消除未使用参数的警告
}

// Fx4: 预测状态3: 航向角 ψ
void Fx4(Matrix_t *pu_p, Matrix_t *pX_p, Matrix_t *pX_m, int sigmaIdx, double dT) {
    const int nCol = pX_m->ncol;
    float psi = pX_p->val[nCol * 3 + sigmaIdx];
    float omega = pX_p->val[nCol * 4 + sigmaIdx];
    pX_m->val[nCol * 3 + sigmaIdx] = psi + omega * dT; // 航向角积分
    pu_p = pu_p;
}

// Fx5: 预测状态4: 角速度 ω (CTRV模型中，预测阶段角速度不变)
void Fx5(Matrix_t *pu_p, Matrix_t *pX_p, Matrix_t *pX_m, int sigmaIdx, double dT) {
    const int nCol = pX_m->ncol;
    pX_m->val[nCol * 4 + sigmaIdx] = pX_p->val[nCol * 4 + sigmaIdx]; // ω不变
    pu_p = pu_p; dT = dT;
}

// Hy1: 预测测量0: 速度观测
void Hy1(Matrix_t *pu, Matrix_t *pX_m, Matrix_t *pY_m, int sigmaIdx) {
    const int nCol = pY_m->ncol; // 观测的Sigma点数组列数
    const int state_nCol = pX_m->ncol; // 状态Sigma点数组列数
    // 观测值 = 对应的状态值 (v)
    pY_m->val[nCol * 0 + sigmaIdx] = pX_m->val[state_nCol * 2 + sigmaIdx];
    pu = pu;
}

// Hy2: 预测测量1: 航向角观测
void Hy2(Matrix_t *pu, Matrix_t *pX_m, Matrix_t *pY_m, int sigmaIdx) {
    const int nCol = pY_m->ncol;
    const int state_nCol = pX_m->ncol;
    // 观测值 = 对应的状态值 (ψ)
    pY_m->val[nCol * 1 + sigmaIdx] = pX_m->val[state_nCol * 3 + sigmaIdx];
    pu = pu;
}

// Hy3: 预测测量2: 角速度观测
void Hy3(Matrix_t *pu, Matrix_t *pX_m, Matrix_t *pY_m, int sigmaIdx) {
    const int nCol = pY_m->ncol;
    const int state_nCol = pX_m->ncol;
    // 观测值 = 对应的状态值 (ω)
    pY_m->val[nCol * 2 + sigmaIdx] = pX_m->val[state_nCol * 4 + sigmaIdx];
    pu = pu;
}

