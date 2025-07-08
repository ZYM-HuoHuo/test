#ifndef DEV_UT_H_
#define DEV_UT_H_

#include "../motor_ctrl.h"
#include "dev_DM.h"


#ifdef __cplusplus
extern "C" {
#endif

#ifndef DEFINED_UniTree_MOTOR_LIB
#define DEFINED_UniTree_MOTOR_LIB
// 控制帧幅值
#define GO_M8010_6_P_MAX     411774  // rad ,约等于65535圈(单圈绝对)
#define GO_M8010_6_V_MAX     804.0f  // rad/s
#define GO_M8010_6_T_MAX     127.99f // Nm
#define GO_M8010_6_KP_MAX    25.599f
#define GO_M8010_6_KD_MAX    25.599f
// 电机特性
#define GO_M8010_6_SPEED_MAX 30       // rad/s
#define GO_M8010_6_TRQE_MAX  23.7f    // Nm
#define GO_M8010_6_CURR_MAX  40.f     // A
#define Kn_GO_M8010_6        0.63895f // Nm/A
#define RR_GO_M8010_6        6.33     // 减速比
// 宇树电机标识
typedef enum {
    motor_model_UT_M8010_6 = 8010U,
} UT_motor_model_t;

typedef motor_ins_t _UT_Motor_t;
static const _UT_Motor_t UT_motor_lib_GO_M8010_6 = {
    motor_model_UT_M8010_6, 0, 8191, RR_GO_M8010_6, 0, 0, 0
};

#define IS_UT_MOTOR(motor) ((motor).type.model == motor_model_UT_M8010_6)

#endif

#pragma pack(1)
/**
 * @brief 电机模式控制信息
 */
typedef struct {
    uint8_t id : 4;     // 电机ID: 0,1...,13,14 15表示向所有电机广播数据(此时无返回)
    uint8_t status : 3; // 工作模式: 0.锁定 1.FOC闭环 2.编码器校准 3.保留
    uint8_t none : 1;   // 保留位
} RIS_Mode_t;           // 控制模式 1Byte

/**
 * @brief 电机状态控制信息
 */
typedef struct {
    int16_t tor_des; // 期望关节输出扭矩 unit: N.m      (q8)
    int16_t spd_des; // 期望关节输出速度 unit: rad/s    (q8)
    int32_t pos_des; // 期望关节输出位置 unit: rad      (q15)
    int16_t k_pos;   // 期望关节刚度系数 unit: -1.0-1.0 (q15)
    int16_t k_spd;   // 期望关节阻尼系数 unit: -1.0-1.0 (q15)

} RIS_Comd_t; // 控制参数 12Byte

/**
 * @brief 电机状态反馈信息
 */
typedef struct {
    int16_t torque;      // 实际关节输出扭矩 unit: N.m     (q8)
    int16_t speed;       // 实际关节输出速度 unit: rad/s   (q8)
    int32_t pos;         // 实际关节输出位置 unit: rad     (q15)
    int8_t temp;         // 电机温度: -128~127°C
    uint8_t MError : 3;  // 电机错误标识: 0.正常 1.过热 2.过流 3.过压 4.编码器故障
                         // 5-7.保留
    uint16_t force : 12; // 足端气压传感器数据 12bit (0-4095)
    uint8_t none : 1;    // 保留位
} RIS_Fbk_t;             // 状态数据 11Byte

typedef union {
    int32_t L;
    uint8_t u8[4];
    uint16_t u16[2];
    uint32_t u32;
    float F;
} COMData32;

// ━━━━━━━━━━━━━━━━━━━━━━━━━ 真正需要用的发送数据结构体 ━━━━━━━━━━━━━━━━━━━━━━━━━
typedef struct {
    // 定义 电机控制命令数据包
    uint8_t head[2];       // 包头               2  Byte
    RIS_Mode_t mode;       // 电机控制模式       1  Byte
    RIS_Comd_t comd;       // 电机期望数据       12 Byte
    uint16_t CRC16;        // CRC                2  Byte
} UT_motor_transmit_msg_t; // 电机控制命令数据包 17 Byte

// ━━━━━━━━━━━━━━━━━━━━━━━━━ 真正需要用的接收数据结构体 ━━━━━━━━━━━━━━━━━━━━━━━━━
typedef struct {
    uint8_t head[2];   // 包头          2  Byte
    RIS_Mode_t mode;   // 电机控制模式  1  Byte
    RIS_Fbk_t fbk;     // 电机反馈数据  11 Byte
    uint16_t CRC16;    // CRC           2  Byte
} UT_raw_motor_data_t; // 返回数据      16 Byte

#pragma pack()

//具体的RS485串口要在gpio.h or main.h 中配置
#define SET_485_DE_UP()   HAL_GPIO_WritePin(RS485_DIR1_GPIO_Port, RS485_DIR1_Pin, GPIO_PIN_SET)
#define SET_485_DE_DOWN() HAL_GPIO_WritePin(RS485_DIR1_GPIO_Port, RS485_DIR1_Pin, GPIO_PIN_RESET)


void UT_motor_dma_init(UART_HandleTypeDef *uart);
void UT_motor_register(motor_t *pmotor, UT_motor_model_t motor_model, UART_HandleTypeDef *uart,
                       uint8_t _id, const M_LPF_t *init_lpf, motor_institution_t inst, float_t kd,
                       float_t kp, float min_angle, float max_angle);
void set_UT_zero_angle(motor_t *p_motor);

MOTORLIB_StatusTypeDef mount_UT_onto_bus(motor_t *pmotor, UT_motor_transmit_msg_t *mail);
UT_motor_transmit_msg_t *convert_UT_tx_data(motor_t *pmotor, UT_motor_transmit_msg_t *p_data);
MOTORLIB_StatusTypeDef load_UT_motor_output(motor_t *pmotor, UT_motor_transmit_msg_t *tx_msg);

MOTORLIB_StatusTypeDef set_single_UT_motor_output(motor_t *pmotor, UT_motor_transmit_msg_t *tx_msg);
MOTORLIB_StatusTypeDef set_UT_motor_output(void);

MOTORLIB_StatusTypeDef parse_UT_motor_data(UART_HandleTypeDef *uart);

MOTORLIB_StatusTypeDef enable_UT_motor_output(motor_t *pmotor);
MOTORLIB_StatusTypeDef disable_UT_motor_output(motor_t *pmotor);
MOTORLIB_StatusTypeDef shut_all_UT_motor_output(void);


#ifdef __cplusplus
}
#endif
#endif
