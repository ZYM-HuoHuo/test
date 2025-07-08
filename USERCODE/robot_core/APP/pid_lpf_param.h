#ifndef _PID_PARA_H
#define _PID_PARA_H

#include "algorithm/pid.h"
#include "algorithm/pid_leso.h"
#include "Devices/MOTOR/motor_headers.h"
#include <string.h>

extern const pid_struct_t yaw_pid_ang_loop;
extern const pid_struct_t yaw_pid_rpm_loop;
extern const pid_struct_t pitch_pid_ang_loop;
extern const pid_struct_t pitch_pid_rpm_loop;

extern const pid_struct_t yaw_imu_pid_ang_loop;
extern const pid_struct_t yaw_imu_pid_rpm_loop;
extern const pid_struct_t yaw_imu_pid_ang_vision_loop;
extern const pid_struct_t yaw_imu_pid_rpm_vision_loop;
extern const pid_struct_t pitch_imu_pid_ang_loop;
extern const pid_struct_t pitch_imu_pid_rpm_loop;

extern const pid_struct_t shoot_left_fric_pid_init_val;
extern const pid_struct_t shoot_right_fric_pid_init_val;
extern const pid_struct_t dial_rpm_pid_init_val;
extern const pid_struct_t dial_ang_pid_init_val;

extern const pid_struct_t self_power_base_ratio_pid_init_val;
extern const pid_struct_t self_power_limit_pid_init_val;
extern const pid_struct_t self_power_unlimit_pid_init_val;
extern const pid_struct_t power_limit_init_val;
extern const pid_struct_t power_unlimit_init_val;
extern const pid_struct_t power_buffer_r_contain_pid_init;

#define INIT_PID_STRUCT(p,c) memcpy(&(p),&(c),sizeof(pid_struct_t));

extern const M_LPF_t pitch_lpf_init_val;
extern const M_LPF_t yaw_lpf_init_val;
extern const M_LPF_t R_fric_lpf_init_val;
extern const M_LPF_t L_fric_lpf_init_val;
extern const M_LPF_t dial_lpf_init_val;

#endif

