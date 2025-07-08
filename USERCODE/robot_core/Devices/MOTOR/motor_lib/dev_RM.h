#ifndef DEV_RM_H_
#define DEV_RM_H_
#include "../motor_ctrl.h"
#ifdef __cplusplus
extern "C" {
#endif
#define SET_MOTOR_TYPE(pmotor, motor_model) (pmotor->type.model = (void *)motor_model)

#ifndef DEFINED_RM_MOTOR_LIB
#define DEFINED_RM_MOTOR_LIB
#define C620_CURR_DATA_MAX   16384.f ///< C620 maximum output
#define C620_CURR_MAX        20.f    //
#define M3508_CURR_RATED     10.f    // 挂载C620下的额定电流
#define M3508_TQRE_RATED     3.0f    // 挂载C620下的额定扭矩
#define M3508_CURR_MAX       2.5f    // 最大堵转电流
#define M3508_TQRE_MAX       4.5f    // 最大堵转扭矩
#define Kn_M3508             0.3f    //< 转矩常数,单位N*m/A

#define C610_CURR_DATA_MAX   10000.f ///< C610 maximum current
#define C610_CURR_MAX        10.f    //
#define M2006_CURR_RATED     3.f     // 挂载C610下的额定电流
#define M2006_TQRE_RATED     1.f     // 挂载C610下的额定扭矩
#define Kn_M2006             0.18f   //< 转矩常数,单位N*m/A

#define GM6020_VOLT_DATA_MAX 25000.f    //< 2020年旧版GM6020为30000
#define GM6020_VOLT_MAX      25.2f      //
#define GM6020_VOLT_RATED    24.f       //< 额定电压
#define GM6020_CURR_DATA_MAX 8191.f     //
#define GM6020_CURR_RATED    1.62f      //< 额定电流
#define Kn_GM6020            0.741f     //< 转矩常数,单位N*m/A
#define Kn_RPM_GM6020        13.33f     //< 转速常数,单位RPM/V
#define Ke_GM6020            13885.416f //< 电机反电动势常数,单位(rpm/V)

typedef struct {
    motor_ins_t L;
    motor_ins_t H;
} _RM_QUAD_Motor_t;

typedef enum {
    motor_model_RM_QUAD_M2006 = 42006u, // 直接拿电机尺寸做标识,简单又暴力
    motor_model_RM_QUAD_M3508 = 43508u,
    motor_model_RM_QUAD_GM6020 = 46020u,
} RM_QUAD_motor_model_t;

// M2006 / C610
#define RR_M2006   36.f
#define SPAN_M2006 (8191 - 0)
static _RM_QUAD_Motor_t RM_motor_lib_M2006 = {
    .L = { motor_model_RM_QUAD_M2006, 0, 8191, RR_M2006, 0x200, 0x200, 0 },
    .H = { motor_model_RM_QUAD_M2006, 0, 8191, RR_M2006, 0x1ff, 0x200, 0 },
};
// M3508 / C620
#define RR_BARE_M3508 1.f              // 无减速箱
#define RR_M3508      (3591.f / 187.f) // 原装减速箱
#define RR_EX_M3508   (286.f / 17.f)   // GEARBOX减速箱
#define SPAN_M3508    (8191 - 0)
static _RM_QUAD_Motor_t RM_motor_lib_M3508 = {
    .L = { motor_model_RM_QUAD_M3508, 0, 8191, RR_M3508, 0x200, 0x200, 0 },
    .H = { motor_model_RM_QUAD_M3508, 0, 8191, RR_M3508, 0x1ff, 0x200, 0 },
};
// GM6020
#define RR_GM6020   1.f
#define SPAN_GM6020 (8191 - 0)
static _RM_QUAD_Motor_t RM_motor_lib_GM6020 = {
    .L = { motor_model_RM_QUAD_GM6020, 0, 8191, RR_GM6020, 0x1ff, 0x204, 0 },
    .H = { motor_model_RM_QUAD_GM6020, 0, 8191, RR_GM6020, 0x2ff, 0x204, 0 },
};
#define IS_RM_QUAD_MOTOR(motor)                         \
    ((motor).type.model == motor_model_RM_QUAD_M2006 || \
     (motor).type.model == motor_model_RM_QUAD_M3508 || \
     (motor).type.model == motor_model_RM_QUAD_GM6020)
#endif
/**
 * @ struct RM_raw_motor_data_t
 * @brief the struct that stores motor data receive from CAN
 */
typedef struct {
    ///<<< 原始反馈值 >>>///
    uint16_t raw_scale;
    //< 本次电机磁编码器输出值,取值0~8192
    int16_t raw_rpm;
    //< 电机转子旋转速度,单位圈每秒
    int16_t current;
    //< 电机输出电流
    uint8_t tempture;
    //< 电机内部温度,单位°C
} RM_raw_motor_data_t;
/**
 * @struct RM_QUAD_motor_transmit_msg_t
 * @brief the struct stores motor data for transmission
 */
typedef struct {
    int16_t D[4];   ///< the array of 4 16bit data
    bool send_flag; ///< send flag of the pack
} RM_QUAD_motor_transmit_msg_t;
#if NUM_OF_RM_QUAD_MOTOR
void RM_QUAD_motor_register(motor_t *pmotor, RM_QUAD_motor_model_t motor_model,
                            motor_work_mode_t work_mode, motor_com_type_t com_type, uint32_t *pcan,
                            uint8_t ID, float RR, const M_LPF_t *init_lpf,
                            motor_institution_t inst);
void RM_QUAD_motor_cancell(motor_t *pmotor);
void parse_RM_QUAD_motor_data(motor_t *pmotor, uint8_t *rx_buffer);
MOTORLIB_StatusTypeDef mount_RM_QUAD_onto_bus(motor_t *pmotor, RM_QUAD_motor_transmit_msg_t *mail);
MOTORLIB_StatusTypeDef set_RM_QUAD_motor_output(void);
MOTORLIB_StatusTypeDef set_OnePack_RM_motor_output(CAN_HandleTypeDef *hcan,
                                                   RM_QUAD_motor_transmit_msg_t *tx_msg,
                                                   uint32_t id, motor_com_type_t com_type);
MOTORLIB_StatusTypeDef shut_RM_QUAD_motor_output(CAN_HandleTypeDef *hcan, uint32_t id,
                                                 motor_com_type_t com_type);
#endif
#ifdef __cplusplus
}
#endif
#endif // !DEV_RM_H_
