#ifndef STATE_SPACE_H_

#endif // !STATE_SPACE_H_
#ifdef __cplusplus
extern "C" {
#endif
// 一下是板凳模型
extern float Stool_A_init[4*4], Stool_B_init[4*1], Stool_C_init[4*4], Stool_D_init[4*1];
extern float LegW_A_init[6*6],LegW_B_init[6*2],LegW_C_init[6*6],LegW_D_init[6*2];

// 腿长拟合
void fitting_Stool_A(float A_fit[4*4], float L0);
void fitting_Stool_B(float B_fit[4*1], float L0);

void fitting_LegW_A(float A_fit[6*6],float L0);
void fitting_LegW_B(float B_fit[6*2],float L0);
#ifdef __cplusplus
}
#endif
