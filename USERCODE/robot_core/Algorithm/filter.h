#ifndef _FILTER_H
#define _FILTER_H
#ifdef __cplusplus
extern "C" {
#endif
//IIR filter channel number (2 or 3 refer to the order)

#define MAX_FILTER_CH_2 4
#define MAX_FILTER_CH_3 11
#define MAX_FILTER_CH_5 8
#define MAX_ONE_EURO_FILTER_CH 3

#define GIMBAL_PITCH_FILTER2_RPM_CH 0
#define GIMBAL_YAW_FILTER2_RPM_CH 1
#define GIMBAL_PITCH_FIR_CH 0
#define GIMBAL_YAW_FIR_CH 1
#define SHOOT_LEFT_FILTER2_RPM_CH 2
#define SHOOT_RIGHT_FILTER2_RPM_CH 3
#define GIMBAL_YAW_VISION_FIR_CH 4
#define CHASSIS_FOLLOW_FILTER2_RPM_CH 5

float mean_filter_2(float x_i);
float iir_filter_2(float x, unsigned int ch);
float iir_filter_3(float x, unsigned int ch);

float fir_filter_5(float x, unsigned int ch);
float one_euro_filter(float x, float dt, unsigned int ch);

typedef struct
{
	float orig_val;			//orignal value,ԭʼֵ
	float fltr_val;			//filter value,�˲�ֵ
	float ts;				//采样频率
	float fc;				//截止频率
	float a;				//低通权重
}LPF_t;
#define HPF_t LPF_t

void LPF_init(LPF_t* filter,float ts,float fc);
float LPF_update(LPF_t* filter,float new_value);
void HPF_init(HPF_t* filter,float ts,float fc);
float HPF_update(HPF_t* filter,float new_value);
#ifdef __cplusplus
extern "C" }
#endif
#endif

