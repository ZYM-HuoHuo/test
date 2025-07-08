/**
 * @file motor_ctrl.h
 *
 * @brief 电机控制中间层
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#ifndef MOTOR_CTRL_H_
#define MOTOR_CTRL_H_

#include <stdint.h>
#include "./motor_conf.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CHASSIS_MOTORS = 0u,
    GIMBAL_MOTORS,
    SHOOTER_MOTORS,
    FRIC_MOTORS,
    DIAL_MOTORS,
    ARM_MOTORS,
    TEST_MOTORS,
    DEFAULT,
} motors_type_t;
/*-----------------------------------------------------------------*/

HAL_StatusTypeDef parse_motor_data(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *header,
                                   uint8_t *rx_buffer);
HAL_StatusTypeDef set_all_motor_output(motors_type_t motors_type);
/*-----------------------------------------------------------------*/
#define NumOfMotorErrSet(_motor_) (sizeof((_motor_).INFO.error_set) / sizeof(enum __ERRSET_T))
#define SetMotorErrorCode(_motor_, ERRORCODE) \
    (_motor_).INFO.error_set[((_motor_).INFO.EC_cnt++) % NumOfMotorErrSet(_motor_)] = (ERRORCODE)

#define can_of(_motor_)      ((CAN_HandleTypeDef *)((_motor_).pcom))
#define rx_stdid_of(_motor_) ((_motor_).type.rx_base_ID + (_motor_).type.offset_ID)
#define tx_stdid_of(_motor_) \
    ((_motor_).type.tx_base_ID + ((_motor_).AUX.QUAD ? 0 : (_motor_).type.offset_ID))
// 返回电机编码器范围
#define span_of(_motor_)          ((_motor_).type.measure_max - (_motor_).type.measure_min + 1)
// 返回电机输出轴距转子的减速比
#define RR_of(_motor_)            ((_motor_).type.reduction_ratio)
// 返回电机反馈频率
#define freq_of(_motor_)          ((_motor_).INFO.refresh_freq)
// 用反馈频率判断电机是否掉线
#define IS_MOTOR_OFFLINE(_motor_) (!(_motor_).INFO.is_online)

// 载入电机输出编码值(含滤波)
#define LOAD_MOTOR_LPFTFF(_motor_, set_data) \
    ((_motor_).T_ff = M_LPF_update(&(_motor_).lpf_fltr, (set_data)))
// 载入电机输出编码值(不含滤波)
#define LOAD_MOTOR_TFF(_motor_, set_data)  ((_motor_).T_ff = (set_data))
#define LOAD_MOTOR_VDES(_motor_, set_data) ((_motor_).V_des = (set_data))
#define LOAD_MOTOR_PDES(_motor_, set_data) ((_motor_).P_des = (set_data))
#define LOAD_MOTOR_KP(_motor_, set_data)   ((_motor_).AUX.Kp = (set_data))
#define LOAD_MOTOR_KD(_motor_, set_data)   ((_motor_).AUX.Kd = (set_data))

#define ENABLE_ALL_HIP_MOTOR(_motor_)                                  \
    ((_motor_).LF_hip.AUX.EN = true, ((_motor_).LB_hip.AUX.EN = true), \
     ((_motor_).RF_hip.AUX.EN = true), ((_motor_).RB_hip.AUX.EN = true))
#define DISABLE_ALL_HIP_MOTOR(_motor_)                                   \
    ((_motor_).LF_hip.AUX.EN = false, ((_motor_).LB_hip.AUX.EN = false), \
     ((_motor_).RF_hip.AUX.EN = false), ((_motor_).RB_hip.AUX.EN = false))
#define ENABLE_ALL_WHEEL_MOTOR(_motor_) \
    ((_motor_).R_wheel.AUX.EN = true, ((_motor_).L_wheel.AUX.EN = true))
#define DISABLE_ALL_WHEEL_MOTOR(_motor_) \
    ((_motor_).R_wheel.AUX.EN = false, ((_motor_).L_wheel.AUX.EN = false))
// 其他函数
motors_t *get_motors_ptr(void);
void set_motor_zero_angle(motor_t *pmotor, uint16_t zero_scale);
uint8_t is_motors_offline(motors_type_t motors_type);
MOTORLIB_StatusTypeDef shutdown_motor(motor_t *pmotor);
HAL_StatusTypeDef shutdown_all_motor(void);
MOTORLIB_StatusTypeDef shutdown_motors_of_structure(motors_type_t motors_type);
void update_motor_status(motors_type_t motors_type);

#ifdef __cplusplus
}
#endif

#endif
