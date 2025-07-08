#include "./motor_math.h"
float m_get_minor_arc(float ang1, float ang2, float angle_range) {
  return (fmodf(((ang1) - (ang2) + (angle_range) * 1.5f), (angle_range)) -
          (angle_range) / 2.f);
}
// 低通滤波
float M_LPF_update(M_LPF_t *filter, float new_value) {
  filter->a = filter->ts / (filter->ts + 1.0f / 2 / PI / filter->fc);
  filter->orig_val = new_value;
  filter->fltr_val =
      filter->fltr_val + filter->a * (filter->orig_val - filter->fltr_val);
  return filter->fltr_val;
}
// 高通滤波
float M_HPF_update(M_HPF_t *filter, float new_value) {
  filter->a = filter->ts / (filter->ts + 1.0f / 2 / PI / filter->fc);
  filter->fltr_val =
      filter->a * filter->fltr_val + filter->a * (new_value - filter->orig_val);
  filter->orig_val = new_value;
  return filter->fltr_val;
}
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
  /// converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
uint16_t float_to_uint(float x, float x_min, float x_max, int bits) {
  /// Converts a float to an unsigned int, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  uint16_t raw_set =
      (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
  return raw_set;
}
