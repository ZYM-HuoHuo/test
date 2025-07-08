#ifndef DEV_DM_H_
#define DEV_DM_H_
#include "../motor_ctrl.h"
#ifdef __cplusplus
extern "C" {
#endif
#ifndef DEFINED_DM_MOTOR_LIB
#define DEFINED_DM_MOTOR_LIB

#define DM8009_OMEGA_MAX     125.9685f
#define DM8009_MIT_P_MAX     12.5
#define DM8009_MIT_V_MAX     45
#define DM8009_MIT_T_MAX     54
#define DM8009_MIT_KP_MAX    1
#define DM8009_MIT_KD_MAX    4
#define DM8009_CURR_DATA_MAX 10000.f
#define DM8009_CURR_MAX      41.044777f // DM上位机看电机上电打印反馈
#define DM8009_TRQE_MAX      40.f
// 关于转矩电流与转矩关系
// https://blog.csdn.net/sy243772901/article/details/82925212
#define DM8009_MAG_FLUX      0.0044f     // 磁通量
#define DM8009_NP            21.f        // 极对数
#define DM8009_INTER_RR      (9.f / 1.f) // 内部减速比

#define Kn_DM8009            (DM8009_TRQE_MAX / DM8009_CURR_MAX)
// ((3.f/2.f)*DM8009_NP*DM8009_MAG_FLUX*DM8009_INTER_RR)
// // 1.2474

#define DM4310_MIT_P_MAX     12.5
#define DM4310_MIT_V_MAX     30
#define DM4310_MIT_T_MAX     10
#define DM4310_MIT_KP_MAX    500
#define DM4310_MIT_KD_MAX    5
#define DM4310_CURR_DATA_MAX 10000.f
#define DM4310_TRQE_RATED    3.f
#define DM4310_CURR_RATED    2.5f
#define DM4310_TRQE_MAX      7.f
#define DM4310_CURR_MAX      7.5f
#define Kn_DM4310            (DM4310_TRQE_MAX / DM4310_CURR_MAX)

#define DM4340_MIT_P_MAX     12.5
#define DM4340_MIT_V_MAX     10
#define DM4340_MIT_T_MAX     28
#define DM4340_MIT_KP_MAX    500
#define DM4340_MIT_KD_MAX    5
#define DM4340_CURR_DATA_MAX 10000.f
#define DM4340_TRQE_RATED    9.f
#define DM4340_CURR_RATED    2.5f
#define DM4340_TRQE_MAX      27.f
#define DM4340_CURR_MAX      8.f
#define DM4340_MAG_FLUX      0.0052f
#define DM4340_NP            14.f
#define DM4340_INTER_RR      (40.f / 1.f)
#define Kn_DM4340            (DM4340_TRQE_MAX / DM4340_CURR_MAX)

#define DM3519_MIT_P_MAX     12.5
#define DM3519_MIT_V_MAX     (874.175f)
#define DM3519_MIT_T_MAX     28
#define DM3519_MIT_KP_MAX    500
#define DM3519_MIT_KD_MAX    5
#define DM3519_CURR_DATA_MAX 16384.f
#define DM3519_TRQE_RATED    3.5f
#define DM3519_CURR_RATED    9.2f
#define DM3519_TRQE_MAX      7.8f
#define DM3519_CURR_MAX      20.5f
#define Kn_DM3519            (DM3519_TRQE_MAX / DM3519_CURR_MAX)

#define DM6006_MIT_P_MAX     12.5
#define DM6006_MIT_V_MAX     45
#define DM6006_MIT_T_MAX     12
#define DM6006_MIT_KP_MAX    500
#define DM6006_MIT_KD_MAX    5
#define DM6006_TRQE_RATED    4.f
#define DM6006_CURR_RATED    4.f
#define DM6006_TRQE_MAX      12.f
#define DM6006_CURR_MAX      13.f
#define Kn_DM6006            (DM6006_TRQE_MAX / DM6006_CURR_MAX)

typedef enum {
    motor_model_DM_DM8009 = 8009u,
    motor_model_DM_DM4310 = 4310u,
    motor_model_DM_DM4340 = 4340u,
    motor_model_DM_DM3519 = 3519u,
    motor_model_DM_DM6006 = 6006u,

    motor_model_DM_QUAD_DM8009 = 48009u,
    motor_model_DM_QUAD_DM4310 = 44310u,
    motor_model_DM_QUAD_DM4340 = 44340u,
    motor_model_DM_QUAD_DM3519 = 43519u,
} DM_motor_model_t;

typedef motor_ins_t _DM_Motor_t;
typedef struct {
    motor_ins_t L;
    motor_ins_t H;
} _DM_Motor_QUAD_t;
// DM8009
#define RR_DM8009   (1.f)       // 不带外部减速箱 为1 上位机设为9
#define SPAN_DM8009 (16383 - 0) // 在P_MAX为12.5时(即出厂时)所对应的最大编码值
static _DM_Motor_t DM_motor_lib_DM8009 = {
    motor_model_DM_DM8009, 0, SPAN_DM8009, RR_DM8009, 0x00, 0x10, 0
};
static _DM_Motor_QUAD_t DM_motor_lib_DM8009_QUAD = {
    .L = { motor_model_DM_QUAD_DM8009, 0, 8191, RR_DM8009, 0x3FE, 0x300, 0 },
    .H = { motor_model_DM_QUAD_DM8009, 0, 8191, RR_DM8009, 0x4FE, 0x300, 0 },
};
// DM4310
#define RR_DM4310   (1.f)       // 不带外部减速箱 为1 上位机设为10
#define SPAN_DM4310 (16383 - 0) // 在P_MAX为12.5时(即出厂时)所对应的最大编码值
static _DM_Motor_t DM_motor_lib_DM4310 = {
    motor_model_DM_DM4310, 0, SPAN_DM4310, RR_DM4310, 0x00, 0x10, 0
};
static _DM_Motor_QUAD_t DM_motor_lib_DM4310_QUAD = {
    .L = { motor_model_DM_QUAD_DM4310, 0, 8191, RR_DM4310, 0x3FE, 0x300, 0 },
    .H = { motor_model_DM_QUAD_DM4310, 0, 8191, RR_DM4310, 0x4FE, 0x300, 0 },
};
// DM4340
#define RR_DM4340   (1.f)       // 不带外部减速箱 为1 上位机设为40
#define SPAN_DM3430 (16383 - 0) // 在P_MAX为12.5时(即出厂时)所对应的最大编码值
static _DM_Motor_t DM_motor_lib_DM4340 = {
    motor_model_DM_DM4340, 0, SPAN_DM3430, RR_DM4340, 0x00, 0x10, 0
};
static _DM_Motor_QUAD_t DM_motor_lib_DM4340_QUAD = {
    .L = { motor_model_DM_QUAD_DM4340, 0, 8191, RR_DM4340, 0x3FE, 0x300, 0 },
    .H = { motor_model_DM_QUAD_DM4340, 0, 8191, RR_DM4340, 0x4FE, 0x300, 0 },
};

// DM3519
#define RR_DM3519        (3591.f / 187.f) // 带外部减速箱 此参数为原装减速箱减速比 若更换减速箱需要更改此处
#define RR_BARE_DM3519   1.f              // 不带外部减速箱
#define SPAN_DM3519      (16383 - 0) // 在P_MAX为12.5时(即出厂时)所对应的最大编码值
#define SPAN_DM3519_QUAD (8191 - 0)
static _DM_Motor_t DM_motor_lib_DM3519 = {
    motor_model_DM_DM3519, 0, SPAN_DM3519, RR_DM3519, 0x00, 0x10, 0
};
static _DM_Motor_QUAD_t DM_motor_lib_DM3519_QUAD = {
    .L = { motor_model_DM_QUAD_DM3519, 0, 8191, RR_DM3519, 0X200, 0x200, 0 },
    .H = { motor_model_DM_QUAD_DM3519, 0, 8191, RR_DM3519, 0X1FE, 0x200, 0 },
};

// DM6006
#define RR_DM6006   (1.f)       // 不带外部减速箱 为1 上位机设为6
#define SPAN_DM6006 (16383 - 0) // 在P_MAX为12.5时(即出厂时)所对应的最大编码值
static _DM_Motor_t DM_motor_lib_DM6006 = {
    motor_model_DM_DM6006, 0, SPAN_DM6006, RR_DM6006, 0x00, 0x10, 0
};
// DM6006截止24/12/7还未出QUAD模式

#define IS_DM_MOTOR(motor)                                                                         \
    ((motor).type.model == motor_model_DM_DM8009 || (motor).type.model == motor_model_DM_DM4310 || \
     (motor).type.model == motor_model_DM_DM4340 || (motor).type.model == motor_model_DM_DM3519 || \
     (motor).type.model == motor_model_DM_DM6006)
#define IS_DM_QUAD_MOTOR(motor)                          \
    ((motor).type.model == motor_model_DM_QUAD_DM8009 || \
     (motor).type.model == motor_model_DM_QUAD_DM4310 || \
     (motor).type.model == motor_model_DM_QUAD_DM4340 || \
     (motor).type.model == motor_model_DM_QUAD_DM3519)

// 寄存器列表
#define DM_REG_UV_Value  0  // 低压保护值	RW	(10.0,3.4E38]	float
#define DM_REG_KT_Value  1  // 扭矩系数	RW	[0.0,3.4E38]	float
#define DM_REG_OT_Value  2  // 过温保护值	RW	[80.0,200)	float
#define DM_REG_OC_Value  3  // 过流保护值	RW	(0.0,1.0)	float
#define DM_REG_ACC       4  // 加速度	RW	(0.0,3.4E38)	float
#define DM_REG_DEC       5  // 减速度	RW	[-3.4E38,0.0)	float
#define DM_REG_MAX_SPD   6  // 最大速度	RW	(0.0,3.4E38]	float
#define DM_REG_MST_ID    7  // 反馈ID	RW	[0,0x7FF]	uint32
#define DM_REG_ESC_ID    8  // 接收ID	RW	[0,0x7FF]	uint32
#define DM_REG_TIMEOUT   9  // 超时警报时间	RW	[0,2^32-1]	uint32
#define DM_REG_CTRL_MODE 10 // 控制模式	RW	[1,4]	uint32
#define DM_REG_Damp      11 // 电机粘滞系数	RO	/	float
#define DM_REG_Inertia   12 // 电机转动惯量	RO	/	float
#define DM_REG_hw_ver    13 // 保留	RO	/	uint32
#define DM_REG_sw_ver    14 // 软件版本号	RO	/	uint32
#define DM_REG_SN        15 // 保留	RO	/	uint32
#define DM_REG_NPP       16 // 电机极对数	RO	/	uint32
#define DM_REG_Rs        17 // 电机相电阻	RO	/	float
#define DM_REG_Ls        18 // 电机相电感	RO	/	float
#define DM_REG_Flux      19 // 电机磁链值	RO	/	float
#define DM_REG_Gr        20 // 齿轮减速比	RO	/	float
#define DM_REG_PMAX      21 // 位置映射范围	RW	(0.0,3.4E38]	float
#define DM_REG_VMAX      22 // 速度映射范围	RW	(0.0,3.4E38]	float
#define DM_REG_TMAX      23 // 扭矩映射范围	RW	(0.0,3.4E38]	float
#define DM_REG_I_BW      24 // 电流环控制带宽	RW	[100.0,10000.0]	float
#define DM_REG_KP_ASR    25 // 速度环Kp	RW	[0.0,3.4E38]	float
#define DM_REG_KI_ASR    26 // 速度环Ki	RW	[0.0,3.4E38]	float
#define DM_REG_KP_APR    27 // 位置环Kp	RW	[0.0,3.4E38]	float
#define DM_REG_KI_APR    28 // 位置环Ki	RW	[0.0,3.4E38]	float
#define DM_REG_OV_Value  29 // 过压保护值	RW	TBD	float
#define DM_REG_GREF      30 // 齿轮力矩效率	RW	(0.0,1.0]	float
#define DM_REG_Deta      31 // 速度环阻尼系数	RW	[1.0,30.0]	float
#define DM_REG_V_BW      32 // 速度环滤波带宽	RW	(0.0,500.0)	float
#define DM_REG_IQ_c1     33 // 电流环增强系数	RW	[100.0,10000.0]	float
#define DM_REG_VL_c1     34 // 速度环增强系数	RW	(0.0,10000.0]	float
#define DM_REG_can_br    35 // CAN波特率代码	RW	[0,4]	uint32
#define DM_REG_sub_ver   36 // 子版本号	RO	/	uint32
#define DM_REG_u_off     50 // u相偏置	RO	　	float
#define DM_REG_v_off     51 // v相偏置	RO	　	float
#define DM_REG_k1        52 // 补偿因子1	RO	　	float
#define DM_REG_k2        53 // 补偿因子2	RO	　	float
#define DM_REG_m_off     54 // 角度偏移	RO	　	float
#define DM_REG_dir       55 // 方向	RO	　	float
#define DM_REG_p_m       80 // 电机位置	RO	　	float
#define DM_REG_xout      81 // 输出轴位置	RO	　	float

#endif

#pragma pack(push, 1)
/**
 * @ struct DM_motor_data_t
 * @brief the struct that stores motor data receive from CAN
 */
typedef struct {
    ///<<< 原始反馈值 >>>///
    uint8_t ID : 4; // 电机反馈数据帧的低8位
    DM_ERROR_FLAG_t ERROR_FLAG : 4;
    uint16_t raw_scale : 14;
    //< 本次电机磁编码器输出值,为16bits无量纲编码值
    uint16_t raw_vel : 12;
    //< 电机转子经过上位机设定减速比后的旋转速度,为12bits无量纲编码值
    uint16_t torque : 12;
    //< 电机转子经过上位机设定减速比后的扭矩,为12bits无量纲编码值
    uint8_t t_mos : 8;
    //< mos温度,单位°C
    uint8_t t_rotor : 8;
    //< 线圈温度,单位°C
} DM_raw_motor_data_t;

/**
 * @struct DM_motor_transmit_MIT_msg_t
 * @brief the struct stores motor data for transmission
 */
typedef struct {
    int16_t expt_scale : 16;
    int16_t expt_vel : 12;
    int16_t Kp : 12;
    int16_t Kd : 12;
    int16_t torque_offset : 12;
} DM_motor_transmit_MIT_msg_t;

typedef struct {
    float expt_scale;
    uint16_t expt_vel_x100 : 16;
    uint16_t imax_x10000 : 16;
} DM_motor_transmit_E_MIT_msg_t;
#pragma pack(pop)
typedef struct {
    float expt_scale;
    float expt_vel;
} DM_motor_transmit_PDESVDES_msg_t;

typedef struct {
    float expt_vel;
    float reserved; // 与其他两个模态对齐
} DM_motor_transmit_VDES_msg_t;
/**
 * @struct DM_motor_transmit_msg_t
 * @brief the struct stores motor data for all work_mode's transmission
 */
typedef union {
    DM_motor_transmit_MIT_msg_t MIT_PACK;
    DM_motor_transmit_PDESVDES_msg_t PDESVDES_PACK;
    DM_motor_transmit_VDES_msg_t VDES_PACK;
    DM_motor_transmit_E_MIT_msg_t E_MIT_PACK;
} DM_motor_transmit_msg_t;

/**
 * @struct DM_QUAD_motor_transmit_msg_t
 * @brief the struct stores motor data for QUAD_mode's transmission
 */
typedef struct {
    int16_t D[4];   ///< the array of 4 16bit data
    bool send_flag; ///< send flag of the pack
} DM_QUAD_motor_transmit_msg_t;
typedef struct {
    ///<<< 原始反馈值 >>>///
    uint16_t raw_scale;
    //< 本次电机磁编码器输出值,取值0~8192
    int16_t raw_rpm_x100;
    //< 电机转子旋转速度 单位rpm 注意此量被放大100倍
    int16_t current_mA;
    //< 电机输出电流 单位为mA
    uint8_t t_rotor;
    //< 线圈温度,单位°C
    uint8_t t_pcb;
    //< pcb温度,单位°C
} DM_QUAD_raw_motor_data_t;

#if NUM_OF_DM_MOTOR
void DM_motor_register(motor_t *pmotor, DM_motor_model_t motor_model, motor_work_mode_t work_mode,
                       motor_com_type_t com_type, uint32_t *pcan, uint8_t ID, float RR,
                       const M_LPF_t *init_lpf, motor_institution_t inst);
void DM_motor_cancell(motor_t *pmotor);
void parse_DM_motor_data(motor_t *pmotor, uint8_t *rx_buffer);
MOTORLIB_StatusTypeDef mount_DM_onto_bus(motor_t *pmotor, DM_motor_transmit_msg_t *mail,
                                         uint8_t ID);
MOTORLIB_StatusTypeDef set_DM_motor_output(void);

MOTORLIB_StatusTypeDef enable_single_DM_motor_output(CAN_HandleTypeDef *hcan, uint32_t id,
                                                     motor_com_type_t com_type);
MOTORLIB_StatusTypeDef enable_DM_motor_output(CAN_HandleTypeDef *hcan);
MOTORLIB_StatusTypeDef disable_DM_motor_output(CAN_HandleTypeDef *hcan);
MOTORLIB_StatusTypeDef disable_single_DM_motor_output(CAN_HandleTypeDef *hcan, uint32_t id,
                                                      motor_com_type_t com_type);
MOTORLIB_StatusTypeDef set_single_DM_motor_pzero(CAN_HandleTypeDef *hcan, uint32_t id,
                                                 motor_com_type_t com_type);
MOTORLIB_StatusTypeDef set_DM_motor_pzero(CAN_HandleTypeDef *hcan);
MOTORLIB_StatusTypeDef clear_single_DM_motor_error(CAN_HandleTypeDef *hcan, uint32_t id,
                                                   motor_com_type_t com_type);
MOTORLIB_StatusTypeDef clear_DM_motor_error(CAN_HandleTypeDef *hcan);

MOTORLIB_StatusTypeDef set_DM_motor_output_MIT(CAN_HandleTypeDef *hcan,
                                               DM_motor_transmit_MIT_msg_t *tx_msg, uint32_t id,
                                               motor_com_type_t com_type);
MOTORLIB_StatusTypeDef set_DM_motor_output_PDESVDES(CAN_HandleTypeDef *hcan,
                                                    DM_motor_transmit_PDESVDES_msg_t *tx_msg,
                                                    uint32_t id, motor_com_type_t com_type);
MOTORLIB_StatusTypeDef set_DM_motor_output_VDES(CAN_HandleTypeDef *hcan,
                                                DM_motor_transmit_VDES_msg_t *tx_msg, uint32_t id,
                                                motor_com_type_t com_type);
MOTORLIB_StatusTypeDef set_DM_motor_output_E_MIT(CAN_HandleTypeDef *hcan,
                                                 DM_motor_transmit_E_MIT_msg_t *tx_msg, uint32_t id,
                                                 motor_com_type_t com_type);

MOTORLIB_StatusTypeDef read_single_DM_motor_pm(motor_t *pmotor, motor_com_type_t com_type);
MOTORLIB_StatusTypeDef read_single_DM_motor_xout(motor_t *pmotor, motor_com_type_t com_type);
MOTORLIB_StatusTypeDef read_single_DM_motor_can_br(motor_t *pmotor, motor_com_type_t com_type);
MOTORLIB_StatusTypeDef write_single_DM_motor_can_br(motor_t *pmotor, uint8_t can_br,
                                                    motor_com_type_t com_type);
MOTORLIB_StatusTypeDef write_single_DM_motor_kp_asr(motor_t *pmotor, float kp,
                                                    motor_com_type_t com_type);
MOTORLIB_StatusTypeDef write_single_DM_motor_ki_asr(motor_t *pmotor, float ki,
                                                    motor_com_type_t com_type);
MOTORLIB_StatusTypeDef storage_single_DM_motor_can_br(motor_t *pmotor, motor_com_type_t com_type);
MOTORLIB_StatusTypeDef storage_single_DM_motor_kp_asr(motor_t *pmotor, motor_com_type_t com_type);
MOTORLIB_StatusTypeDef storage_single_DM_motor_ki_asr(motor_t *pmotor, motor_com_type_t com_type);

#endif
#if NUM_OF_DM_QUAD_MOTOR
void DM_QUAD_motor_register(motor_t *pmotor, DM_motor_model_t motor_model,
                            motor_work_mode_t work_mode, motor_com_type_t com_type, uint32_t *pcan,
                            uint8_t ID, float RR, const M_LPF_t *init_lpf,
                            motor_institution_t inst);
void DM_QUAD_motor_cancell(motor_t *pmotor);
MOTORLIB_StatusTypeDef set_OnePack_DM_QUAD_motor_output(CAN_HandleTypeDef *hcan,
                                                        DM_QUAD_motor_transmit_msg_t *tx_msg,
                                                        uint32_t id);
MOTORLIB_StatusTypeDef set_OnePack_DM_QUAD_motor_output_SP(CAN_HandleTypeDef *hcan,
                                                           DM_QUAD_motor_transmit_msg_t *tx_msg,
                                                           uint32_t id);
MOTORLIB_StatusTypeDef mount_DM_QUAD_onto_bus(motor_t *pmotor, DM_QUAD_motor_transmit_msg_t *mail);
MOTORLIB_StatusTypeDef set_DM_QUAD_motor_output(void);
MOTORLIB_StatusTypeDef shut_DM_QUAD_several_motor_output(CAN_HandleTypeDef *hcan, uint32_t id);
void parse_DM_QUAD_motor_data(motor_t *pmotor, uint8_t *rx_buffer);
#endif
#ifdef __cplusplus
}
#endif
#endif // !DEV_DM_H_
