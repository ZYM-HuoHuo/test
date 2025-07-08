/**
 * @file util.h
 *
 * @brief 实用数学工具

 *
 * @copyright SCNU-PIONEER (c) 2022-2023
 *
 */
#ifndef _UTIL_H
#define _UTIL_H
#include "drv_conf.h"
#include HAL_INCLUDE
#include <math.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

#define relative_err(UNdenominnator, denominator) (fabs(UNdenominnator - denominator) / denominator)

#define NL_DENDBAND(error, deadband) \
  NL_DENDBAND_RATIO(error, deadband) * (error)
#define NL_DENDBAND_RATIO(error, deadband) \
  fabsf((error) / (deadband)) >= 1.f ? 1.f : pow2f((error) / (deadband))

#define sign(_n) (_n == 0 ? 0.0f : (_n > 0 ? 1.0f : -1.0f))
#define pow2f(_n) (_n * _n)
#define pow3f(_n) (pow2f(_n) * (_n))
#define pow4f(_n) (pow3f(_n) * (_n))
#define pow5f(_n) (pow4f(_n) * (_n))

#define rad2deg(_n) (180.f * (_n) / PI)
#define deg2rad(_n) (PI * (_n) / 180.f)

#define rpm2mps(_rpm) (2 * PI * (_rpm) / 60.f * 0.001 * D_friction / 2)
#define mps2rpm(_mps) ((_mps) / (D_friction / 2 * 0.001) / (2 * PI) * 60.f)
#define rpm2radps(_rpm) ((_rpm) * PI / 30.f) // rpm转rad/s
#define radps2rpm(_rpm) ((_rpm) * 30.f / PI) // rad/s转rpm
// 浮点限幅
static inline float CLAMP(float input, float max){
  if (input > max)
    return (max);
  else if (input < -max)
    return -(max);
  else
    return input;
}
// 整型限幅
static inline int16_t CLAMP_INT(int16_t input, int16_t max){
  if (input > max)
    return (max);
  else if (input < -max)
    return -(max);
  else
    return input;
}
//绝对值限制
static inline float abs_limit(float num, float Limit){
    if (num > Limit)
        num = Limit;
    else if (num < -Limit)
        num = -Limit;
    return num;
}
//浮点死区
static inline float float_deadband(float Value, float minValue, float maxValue){
    if (Value < maxValue && Value > minValue)
        Value = 0.0f;
    return Value;
}
//int16死区
static inline int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue){
    if (Value < maxValue && Value > minValue)
        Value = 0;
    return Value;
}
// 数值平滑跟踪 跟踪步长为acc
static inline void slope_following(float *ref,float *cur,float acc){
	if(*ref > *cur){
		*cur = *cur + acc;
		if(*cur >= *ref)
		*cur = *ref;
	}
	else if(*ref < *cur){
		*cur = *cur - acc;
		if(*cur <= *ref)
		*cur = *ref;
	}
}
/**
 *	@brief	将周期数据限制在周期两端范围内
 */
#define range_map(scale, min, max)                                                 \
  ((min) > (max)) ? (scale) : (min) + fmodf(((scale) - (min)), ((max) - (min)))
/**
 *	@brief	将数据限制在指定范围内
 */
#define LIMIT_MIN_MAX(x, M1, M2)                                                   \
  (x) = (M1 < M2) ? (((x) <= (M1)) ? (M1) : (((x) >= (M2)) ? (M2) : (x)))          \
                  : (((x) <= (M2)) ? (M2) : (((x) >= (M1)) ? (M1) : (x)))
/**
 *	@brief	将数据的量程映射到指定范围
 *  @note   in range为数据原来的范围 out range为欲映射的新范围
 */
#define map(x, in_min, in_max, out_min, out_max)                                   \
  (((x) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))
  

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

// static float invSqrt(float x) {
//	float halfx = 0.5f * x;
//	float y = x;
//	long i = *(long*)&y;
//	i = 0x5F375A86 - (i>>1);
//	y = *(float*)&i;
//	y = y * (1.5f - (halfx * y * y));
//	return y;
// }
static float fast_inv_sqrt(float number){
  const float x2 = number * 0.5F;
  const float threehalfs = 1.5F;
  union
  {
    float f;
    unsigned int i;
  } conv = {.f = number};
  conv.i = 0x5F375A86 - (conv.i >> 1);
  conv.f *= (threehalfs - (x2 * conv.f * conv.f));
  conv.f *= (threehalfs - (x2 * conv.f * conv.f));
  conv.f *= (threehalfs - (x2 * conv.f * conv.f));
  return conv.f;
}
//快速开方
static float Sqrt(float x){
    float y;
    float delta;
    float maxError;
    if (x <= 0){
        return 0;
    }
    // initial guess
    y = x / 2;
    // refine
    maxError = x * 0.001f;
    do{
        delta = (y * y) - x;
        y -= delta / (2 * y);
    } while (delta > maxError || delta < -maxError);
    return y;
}

//二次函数计算速度曲线
#define T_ACC_CNT 100
static float s_curve(float v_max, float cnt){
  float cntcnt;
  if (cnt < 0.0f){
    cnt = -cnt;
    v_max = -v_max;
  }
  cntcnt = cnt / ((float)T_ACC_CNT);
  if (cnt < T_ACC_CNT / 2.0f){ // cnt<50
    return 2.0f * v_max * (cntcnt * cntcnt);
  }
  else if (cnt < T_ACC_CNT){ // 50<cnt<100
    cntcnt = cntcnt - 1.0f;
    return v_max * (1.0f - 2.0f * (cntcnt * cntcnt));
  }
  else
    return v_max;
}

#define ACC_FILTER_MAX_CH 3
static float get_acc(float current, int ch){
  static float last_val[ACC_FILTER_MAX_CH][3];
  float result;
  if (ch >= ACC_FILTER_MAX_CH)
    return 0.0f;
  // refer to the Backward differentiation formula
  result = (11 * current - 18 * last_val[ch][2] + 9 * last_val[ch][1] -
            2 * last_val[ch][0]) /
           6.0f;
  last_val[ch][0] = last_val[ch][1];
  last_val[ch][1] = last_val[ch][2];
  last_val[ch][2] = current;
  return result;
}
static float deadband_range(float current, float ref, float deadband){
  if (fabs(current - ref) <= deadband){
    current = current;
  }
  else
    current = ref;
  return current;
}
static float compare_amount_min(float amount_a, float amount_b){
  if (amount_a > amount_b)
    return amount_b;
  else
    return amount_a;
}
static inline int16_t get_delta_ang(int16_t ang1, int16_t ang2){
	return (ang1 - ang2 + 8192 * 3 / 2) % 8192 - 8192 / 2;
}
/**
 *  @brief	基于angle_range的获取有方向的劣弧
 *  @attention   注意ang1和ang2需要共量程!
 */
#define get_minor_arc(ang1, ang2, angle_range)                                     \
  (fmodf(((ang1) - (ang2) + (angle_range)*1.5f), (angle_range)) -                  \
   (angle_range) / 2.f)
/**
 *  @brief  基于angle_range的获取有方向的优弧
 *  @attention  注意ang1和ang2需要共量程!
 */
#define get_major_arc(ang1, ang2, angle_range)                                     \
  (2*angle_range - get_minor_arc((ang1), (ang2), (angle_range)))           

// 当存在相对大地圆周变化时 用圆系特供限位函数
#define LIMIT_MIN_MAX_ARC(x, M1, M2, R)                                                                                       \
  (x) = (get_minor_arc(M1,M2,R)<0) ? ((get_minor_arc(x,M1,R)<=0) ? (M1) : ((get_minor_arc(x,M2,R)>=0) ? (M2) : (x)))          \
                                   : ((get_minor_arc(x,M2,R)<=0) ? (M2) : ((get_minor_arc(x,M1,R)>=0) ? (M1) : (x)))

/**
 *  @brief	在基于单个周期获得两刻度值的相对距离 也就是正负描述相对方向
 */
#define get_delta_ang_rel(ang1, ang2)                                              \
  (ang1-ang2)
/**
 *  @brief	将量程为2PI的两角映射于[-PI,PI]
 */
static inline float get_delta_ang_pi(float ang1, float ang2){
	float delta_previous = ang1 - ang2;
	if (delta_previous > PI)
		delta_previous -= 2 * PI;
	else if (delta_previous < -PI)
		delta_previous += 2 * PI;
	return delta_previous;
}

#ifdef __cplusplus
}
#endif

#endif
