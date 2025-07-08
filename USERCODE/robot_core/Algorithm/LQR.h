#ifndef LQR_H_
#define LQR_H_

#include "arm_math.h"
#ifdef __cplusplus
extern "C" {
#endif
arm_matrix_instance_f32 *violence_get_2x2_inv_matrix(arm_matrix_instance_f32* s, arm_matrix_instance_f32* d);
arm_matrix_instance_f32 *violence_get_4x4_inv_matrix(arm_matrix_instance_f32* s, arm_matrix_instance_f32* d);
arm_matrix_instance_f32 *violence_get_6x6_inv_matrix(arm_matrix_instance_f32* s, arm_matrix_instance_f32* d);

float *Stool_LQR_Online(float L0);
float *LegW_LQR_Online(float L0);
 
void arm_matrix_test(void);
#ifdef __cplusplus
}
#endif
#endif
