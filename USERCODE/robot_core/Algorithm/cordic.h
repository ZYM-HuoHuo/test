#ifndef CORDIC_H_
#define CORDIC_H_
#ifdef __cplusplus
extern "C" {
#endif
void cordic_atan_sqrtf(float yf, float xf, float* ret_atan, float* ret_sqrt);
void cordic_sin_cosf(float ang, float* ret_sin, float* ret_cos);
float cordic_logf(float e);
#ifdef __cplusplus
}
#endif
#endif
