#ifndef DEV_LK_H_
#define DEV_LK_H_
#include "../motor_ctrl.h"
#ifdef __cplusplus
extern "C" {
#endif
#ifndef DEFINED_LK_MOTOR_LIB
#define DEFINED_LK_MOTOR_LIB

#define MF9025_TRQE_DATA_MAX   2000.f   // 电机控制帧扭矩的最大编码值
#define MF9025_TRQE_MAX        4.2f     // 电机控制帧扭矩的最大实际值
#define MF9025_CURR_DATA_MAX   2048.f   // 电机反馈帧电流的最大编码值
#define MF9025_CURR_MAX        14.0625f // 电机反馈帧电流的最大实际值
// ↑由datasheet的峰值扭矩除去扭矩常数得
#define Kn_MK9025              0.32f //             转矩常数,单位N*m/A

#define LK_BROADCAST_TORQUE_ID 0x280U
#define LK_BROADCAST_SPEED_ID  0x281U
#define LK_BROADCAST_ANGLE_ID  0x282U
#define LK_BROADCAST_MIX_ID    0x288U

typedef enum {
    motor_model_LK_QUAD_MF9025 = 49025u,
} LK_QUAD_motor_model_t;

typedef motor_ins_t _LK_QUAD_Motor_t;
// MF9025
// #define MF9025_BROADCAST_TORQUE_ID 0x280U
#define RR_MF9025   1.f
#define SPAN_MF9025 (65535 - 0)
static const _LK_QUAD_Motor_t LK_motor_lib_MF9025 = {
    motor_model_LK_QUAD_MF9025, 0, 65535, RR_MF9025, 0x280, 0x140, 0
};
/*说明
///> measure_max:
    14bits encoder
///> reduction_ratio:
    Default is 1
///> tx_base_ID:
    MF9025 BROADCAST TORQUE ID is 0x280U
///> rx_base_ID:
    The feedback frame ID (referred to as "rx_base_ID")
*/

/**
 * @brief 检测电机是否为瓴控旗下的电机
 */
#define IS_LK_QUAD_MOTOR(motor) ((motor).type.model == motor_model_LK_QUAD_MF9025)
#endif

#pragma pack(push, 1)
/**
 * @ struct LK_raw_motor_data_t
 * @brief the struct that stores motor data receive from CAN
 */
typedef struct {
    ///<<< 原始反馈值 >>>///
    uint8_t CMD : 8;    // 命令字节
    uint8_t t_rotor;    // 电机温度,单位°C每LSB
    int16_t current;    // 转矩电流,范围-2048~2048,对应实际转矩电流为-14A~14A
    int16_t omega;      // 转子角速度,单位deg/s每LSB
    uint16_t raw_scale; // 编码器位置,为16bits编码器
} LK_raw_motor_data_t;

/**
 * @struct DM_motor_transmit_MIT_msg_t
 * @brief the struct stores motor data for transmission
 */
typedef struct {
    int16_t D[4];   ///< the array of 4 16bit data
    bool send_flag; ///< send flag of the pack
} LK_QUAD_motor_transmit_msg_t;
#pragma pack(pop)

#if NUM_OF_LK_QUAD_MOTOR
void LK_QUAD_motor_register(motor_t *pmotor, LK_QUAD_motor_model_t motor_model,
                            motor_work_mode_t work_mode, motor_com_type_t com_type, uint32_t *pcan,
                            uint8_t ID, float RR, const M_LPF_t *init_lpf,
                            motor_institution_t inst);
void LK_QUAD_motor_cancell(motor_t *pmotor);
void parse_LK_QUAD_motor_data(motor_t *pmotor, uint8_t *rx_buffer);
MOTORLIB_StatusTypeDef mount_LK_QUAD_onto_bus(motor_t *pmotor, LK_QUAD_motor_transmit_msg_t *mail);
MOTORLIB_StatusTypeDef set_LK_QUAD_motor_output(void);
MOTORLIB_StatusTypeDef enable_LK_QUAD_motor_output(CAN_HandleTypeDef *hcan, uint32_t ID_CODE,
                                                   motor_com_type_t com_type);
MOTORLIB_StatusTypeDef disable_LK_QUAD_motor_output(CAN_HandleTypeDef *hcan, uint32_t ID_CODE,
                                                    motor_com_type_t com_type);
MOTORLIB_StatusTypeDef stop_LK_QUAD_motor_output(CAN_HandleTypeDef *hcan, uint32_t ID_CODE,
                                                 motor_com_type_t com_type);
MOTORLIB_StatusTypeDef set_OnePack_LK_motor_output(CAN_HandleTypeDef *hcan,
                                                   LK_QUAD_motor_transmit_msg_t *tx_msg,
                                                   uint32_t id, motor_com_type_t com_type);
MOTORLIB_StatusTypeDef shut_LK_QUAD_motor_output(CAN_HandleTypeDef *hcan, uint32_t id,
                                                 motor_com_type_t com_type);
#endif
#ifdef __cplusplus
}
#endif
#endif // !DEV_LK_H_
