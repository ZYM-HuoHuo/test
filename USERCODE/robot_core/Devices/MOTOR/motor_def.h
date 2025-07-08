/**
 * @file motor_def.h
 *
 * @brief 电机类定义
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#ifndef MOTOR_DEF_H_
#define MOTOR_DEF_H_
#include "./motor_utils/motor_math.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif
/*-----------------------------------------------------------------*/
// 如果芯片使用FDCAN 则将此宏置1 否则置0
#if defined(STM32F407xx)
#ifndef FDCAN_CHIP
#define FDCAN_CHIP 0
#endif
#else
#ifndef FDCAN_CHIP
#define FDCAN_CHIP 1
#endif
#endif

#if FDCAN_CHIP
#ifndef CAN_HandleTypeDef
#define CAN_HandleTypeDef FDCAN_HandleTypeDef
#endif
#ifndef CAN_RxHeaderTypeDef
#define CAN_RxHeaderTypeDef FDCAN_RxHeaderTypeDef
#endif
#ifndef CAN_TxHeaderTypeDef
#define CAN_TxHeaderTypeDef FDCAN_TxHeaderTypeDef
#endif

#ifndef hcan1
#define hcan1 hfdcan1
#endif
#ifndef hcan2
#define hcan2 hfdcan2
#endif
#ifndef hcan3
#define hcan3 hfdcan3
#endif

#ifndef can_transmit_data
#define can_transmit_data fdcan_transmit_data
#endif
#endif
/*-----------------------------------------------------------------*/
#define MOTOR_DETECT_FREQ      DETECT_TIM_FREQ // 电机故障检测的频率 来自drv_conf 需要自己设置
#define MIN_MOTOR_REFRESH_FREQ (MOTOR_DETECT_FREQ * 0.8f)
///< MIN_MOTOR_REFRESH_FREQ为电机反馈数据刷新频率最低值,单位Hz,过低则会判断为离线
/*-----------------------------------------------------------------*/
// several definition
#ifndef CAN_DATA_LEN
#define CAN_DATA_LEN 8
#endif // !CAN_DATA_LEN

#define FILER_FIFO0 0
#define FILER_FIFO1 1

#define M_DISABLE   0
#define M_ENABLE    1
#define M_STOP      2
/*-----------------------------------------------------------------*/
typedef enum {
    MOTORLIB_OK = 0u,
    MOTORLIB_ERROR,
} MOTORLIB_StatusTypeDef;
/*-----------------------------------------------------------------*/
typedef enum {
    MotorDisable = 0x0u,
    MotorEnable = 0x1u,
    OverVoltage = 0x8u,
    LowVoltage = 0x9u,
    OverCurrent = 0xAu,
    MosOverHeat = 0xBu,
    RotorOverHeat = 0xCu,
    CommunicationLoss = 0xDu,
    Overload = 0xEu,
} DM_ERROR_FLAG_t;
/*-----------------------------------------------------------------*/
typedef enum {
    COM_NONE = 0u,
    COM_FDCAN,
    COM_CAN,
    COM_UART,
} motor_com_type_t; // 电机通信协议类型
typedef enum {
    M_NONE = 0u,
    M_CHASSIS,
    // HIP_MOTOR,
    // WHEEL_MOTOR,
    M_GIMBAL,
    M_SHOOTER,
    M_ARM,
    M_TEST,
} motor_institution_t;
typedef enum {
    WM_NONE = 0u,
    QUAD_CURR,       // 一拖四控电流
    QUAD_VDES,       // 一拖四控速
    MIT_TT,          // MIT单环力控
    MIT_VDES,        // MIT双环速控
    MIT_PDESVDES,    // MIT三环位控
    VDES,            // 速度模式
    PDESVDES,        // 位置速度模式
    E_MIT,           // 力位混控
} motor_work_mode_t; // 电机工作模式
typedef struct {
    uint16_t model;        // 一般都是电机尺寸,方便确定电机类型
    uint16_t measure_min;  // Lower limit of encoder output value
    uint16_t measure_max;  // Upper limit of encoder output value
    float reduction_ratio; // reduction_ratio
    uint16_t tx_base_ID;   // Identifier of the message received by the motor (sent
                           // by user)
    uint16_t rx_base_ID;   // Identifier of motor feedback (we receive) message
    uint16_t offset_ID;    // rx_hander.StdId=rx_base_ID+offset_ID
                           // offset_ID number starts from 1
} motor_ins_t;
typedef struct {
    ///<<< 中间处理值 >>>///
    int32_t raw_scale;
    //< 本次电机磁编码器输出值
    int32_t last_raw_scale;
    //< 上一次电机磁编码器输出值
    ///<<< 后续处理值 >>>///
    float rel_angle;
    //< 电机实际输出端距原点的绝对弧度值,取值0~2*PI(弧度制)
    // 此值由积分得出,原点默认为开机处的位置
    float abs_angle;
    //< 电机实际输出端距原点的相对弧度值,单位弧度制
    float omega;
    //< 电机实际输出端旋转速度,单位rad/s
    float rpm;
    //< 电机实际输出端旋转速度,单位r/min
    float current;
    //< 电机输出电流,单位A
    float torque;
    //< 电机输出扭矩,单位N*m
    uint8_t tempture;
    //< 电机内部温度,单位°C
} motor_data_t;
/*-----------------------------------------------------------------*/
/**
 * @brief the local data from reading REG
 */
typedef struct {
    float dat;
    bool read_flag;    // 读成功标志位
    bool write_flag;   // 写成功标志位
    bool storage_flag; // 存储成功标志位
} M_REG_t;
/*-----------------------------------------------------------------*/
typedef struct {
    struct {
        bool EN; // 电机期望使能与否

        float Kp; // MIT位置控制Kp
        float Kd; // MIT速度控制Kd

        M_REG_t p_m;    // 电机多圈转子位置    (来源于读电机寄存器)
        M_REG_t xout;   // 电机多圈出轴位置    (来源于读电机寄存器)
        M_REG_t can_br; // 电机CAN波特率代码   (来源于读电机寄存器)
        M_REG_t kp_asr; // 速度环Kp
        M_REG_t ki_asr; // 速度环Ki

        bool QUAD;     // 是否属于一拖四电机型
        uint8_t STATE; // 电机反馈状态
    } AUX;             // 辅助数据
    struct {
        enum __ERRSET_T {
            MotorError_None = 0u,              // 无逝
            MotorError_InsNotFound,            // 未找到电机型号
            MotorError_ComTypeError,           // 包实际通信协议与期望冲突
            MotorError_WorkModeConflict,       // 工作模式冲突
            MotorError_CanInitFail,            // CAN初始化寄了
            MotorError_OverHeat,               // 过热
            MotorError_MotorOffline,           // 电机离线
            MotorError_BusOverload,            // 总线上挂载的电机数量过多
            MotorError_NoSendingPortSpecified, // 未指定正确的发送com端口
            MotorError_TransmitMotorIdError,   // 发送时无法找对电机Id
            MotorError_TxIdDuplicate,          // 发送控制量的ID重复
            MotorError_Rxdataerror,            // 接收数据校验错误
        } error_set[15];                       // 错误状态集
        uint32_t EC_cnt;                       // error code

        uint32_t refresh_cnt;   // 未滤波的刷新频率
        M_LPF_t refresh_filter; // 刷新频率的滤波器
        float refresh_freq;     // 反馈数据频率

        bool is_online; // 电机在线状态
    } INFO;             // 电机数据信息
    enum register_state_t {
        M_UNREGISTERED = 0u,
        M_REGISTERED,
        M_SYNCHRONIZED
    } register_state;                // 电机注册状态
    motor_ins_t type;                // 电机型号
    motor_institution_t institution; // 电机所属机构
    motor_work_mode_t work_mode;     // 电机工作模式
    motor_com_type_t com_type;       // 电机通信协议类型
    uint32_t *pcom;                  // 发送端口 Handle pointer for CAN or UART
    M_LPF_t lpf_fltr;                // 电机输出滤波器

    motor_data_t real; // 电机实时数据
    float T_ff;        // 电机期望额外力矩 单位:Nm (ps:GM6020为电压控制
                       // 当使用GM6020时,该值物理含义为电压值 单位V) 当模式为E_MIT时
                       // 此值为扭矩限幅
    float V_des;       // 电机期望速度 单位:rad/s
    float P_des;       // 电机期望位置 单位:rad
} motor_t;
/*-----------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif // !MOTOR_DEF_H_
