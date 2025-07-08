#ifndef MOTOR_MATH_H_
#define MOTOR_MATH_H_
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifndef PI
#define PI (3.14159265358979323846f)
#endif // !PI

/**
 *  @brief	基于angle_range的获取有方向的劣弧
 *  @attention   注意ang1和ang2需要共量程!
 */
float m_get_minor_arc(float ang1, float ang2, float angle_range);
/**
 *	@brief	将周期数据限制在周期两端范围内
 */
static inline float M_CLAMP(float input, float max) {
  if (input > max)
    return (max);
  else if (input < -max)
    return -(max);
  else
    return input;
}
/*-----------------------------------------------------------------*/
#define m_range_map(scale, min, max)                                           \
  ((min) > (max)) ? (scale) : (min) + fmodf(((scale) - (min)), ((max) - (min)))
// 其他转换函数
#define m_rad2deg(_n) (180.f * (_n) / PI)
#define m_deg2rad(_n) (PI * (_n) / 180.f)

#define m_rpm2radps(_rpm) ((_rpm) * PI / 30.f) // rpm转rad/s
#define m_radps2rpm(_rpm) ((_rpm) * 30.f / PI) // rad/s转rpm

#define SATURATE(_IN, _MIN, _MAX)                                              \
  {                                                                            \
    if (_IN < _MIN)                                                            \
      _IN = _MIN;                                                              \
    else if (_IN > _MAX)                                                       \
      _IN = _MAX;                                                              \
  }
/*-----------------------------------------------------------------*/
typedef struct {
  float orig_val;
  float fltr_val;
  float ts; // 采样频率
  float fc; // 截止频率
  float a;  // 低通权重
} M_LPF_t;

#define M_HPF_t M_LPF_t

float M_LPF_update(M_LPF_t *filter, float new_value);
float M_HPF_update(M_HPF_t *filter, float new_value);

/*-----------------------------------------------------------------*/

float uint_to_float(int x_int, float x_min, float x_max, int bits);
uint16_t float_to_uint(float x, float x_min, float x_max, int bits);

#endif // !MOTOR_UTILS_H_
