#ifndef _BASE_GIMBALE_H
#define _BASE_GIMBALE_H

#include "drv_conf.h"
#include HAL_INCLUDE

#include "algorithm/pid.h"
#include "algorithm/filter.h"
#include "algorithm/KF.h"
#include "algorithm/imu_fusion.h"

#include "APP/behaviour.h"
#include "APP/gimbal.h"

#define GIMBAL_WEIGHT 0

/**
 * @note 函数变量较多 应有层次 分类型 置入变量
 */

//IMU系控云台
HAL_StatusTypeDef gimbal_imu_ctrl(
                motors_t *motors,
                robot_t *robot,
                pid_struct_t pitch_imu_pid[2], pid_struct_t yaw_imu_pid[2],
                pid_struct_t pitch_vision_pid[2], pid_struct_t yaw_vision_pid[2],
                leso_para_t *pitch_imu_leso, leso_para_t *yaw_imu_leso,
                one_vec_kf_t *kalman_pitch_filter, one_vec_kf_t *kalman_yaw_filter);
//电机系控云台
HAL_StatusTypeDef gimbal_motor_ctrl(
                motors_t* motors,
                robot_t *robot,  
                pid_struct_t pitch_pid[2],pid_struct_t yaw_pid[2] );

float get_gimbal_angle_for_air(motors_t *motors, motor_t *gimbal_pmotor);
float get_gimbal_angle(motors_t *motors, motor_t *gimbal_pmotor);
int16_t get_gimbal_angle_old(motors_t *motors, motor_t *gimbal_pmotor);


#endif
