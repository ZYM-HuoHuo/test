#ifndef _KF_H
#define _KF_H
#ifdef __cplusplus
extern "C" {
#endif
#include "matrix.h"

typedef struct{
	float Q_cur, Q_bias, R_measure;
	float P[2][2];
	float K[2];
	float S, y, bias;
	float rate, result;
}kalman_filter_t;

/* 1 Dimension */
typedef struct {
    float x;  /* state */
    float A;  /* x(n)=A*x(n-1)+u(n),u(n)~N(0,q) */
    float H;  /* z(n)=H*x(n)+w(n),w(n)~N(0,r)   */
    float q;  /* process(predict) noise convariance */
    float r;  /* measure noise convariance */
    float p;  /* estimated error convariance */
    float gain;
} one_vec_kf_t;

/**
 * X(n)=A*X(n-1)+B*U(n-1)+w(n-1) , w~P(0,Q)
 * Z(n)=H*X(n)+v(n)              , v~P(0,R)
 */
typedef struct {
  dual_vec_t X; // 系统状态
  quad_vec_t A; // 系统矩阵
  quad_vec_t B; // 输入矩阵
  quad_vec_t H; // 观测矩阵
  quad_vec_t Q; // 过程噪声协方差,越小代表越相信公式准确程度
  quad_vec_t R; // 测量噪声协方差,越小代表越相信测量准确程度
  quad_vec_t P; // 预测噪声协方差,由调用函数更新,设置初值即可
  quad_vec_t K; // 卡尔曼增益
  float dt;
} dual_vec_kf_t;

typedef struct {
 triple_vec_t X; // 系统状态
 triple_mat_t A; // 系统矩阵
 triple_mat_t B; // 输入矩阵
 triple_mat_t H; // 观测矩阵
 triple_mat_t Q; // 过程噪声协方差,越小代表越相信公式准确程度
 triple_mat_t R; // 测量噪声协方差,越小代表越相信测量准确程度
 triple_mat_t P; // 预测噪声协方差,由调用函数更新,设置初值即可
 triple_mat_t K; // 卡尔曼增益
 float dt;
} triple_mat_kf_t;

void kalman_init(kalman_filter_t *filter, float q_cur, float q_rate, float r);
float kalman_update(kalman_filter_t *filter, float cur, float rate, float dt);
void kalman_set(kalman_filter_t *filter, float cur);

float kalman1_filter(one_vec_kf_t *state, float z_measure);
float kalman1_filter_est(one_vec_kf_t *state, float z_measure, float est);
void kalman1_init(one_vec_kf_t *state, float init_x, float q, float r);

dual_vec_t zzq_dual_kalman_update(dual_vec_kf_t *kf, dual_vec_t U, dual_vec_t Z);
triple_vec_t zzq_triple_kalman_update(triple_mat_kf_t *kf, triple_vec_t U, triple_vec_t Z);
dual_vec_t my_dual_kalman_update(dual_vec_kf_t *kf, dual_vec_t U, dual_vec_t Z);
triple_vec_t my_triple_kalman_update(triple_mat_kf_t *kf, triple_vec_t U, triple_vec_t Z);
#ifdef __cplusplus
}
#endif
#endif

