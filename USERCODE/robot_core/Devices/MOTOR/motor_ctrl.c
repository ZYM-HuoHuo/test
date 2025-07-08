/**
 * @file motor_ctrl.c
 *
 * @brief 电机控制中间层
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#include "motor_ctrl.h"
#include "drv_conf.h"
#include "motor_def.h"
#include <stdint.h>

#ifndef HAL_INCLUDE
#include "main.h"
#else
#include HAL_INCLUDE
#endif // !HAL_INCLUDE
#include "./motor_conf.h"
#include "./motor_lib/dev_DM.h"
#include "./motor_lib/dev_LK.h"
#include "./motor_lib/dev_RM.h"
#include "./motor_lib/dev_UT.h"

motors_t motors = { 0 };

/**
 * @brief transfer raw data to pubilc struct only for can motors
 * @param[in]  rx_buffer  raw data(8 bytes) received
 * @return HAL_OK if success otherwise HAL_ERROR
 */
/*处理电机数据函数，会在CAN接收中断的回调函数内被调用*/
HAL_StatusTypeDef parse_motor_data(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *header,
                                   uint8_t *rx_buffer)
{
    for (uint8_t i = 0; i < NUM_OF_ALL_MOTOR; i++) {
        if (header->StdId == rx_stdid_of(motors._[i]) && hcan == can_of(motors._[i])) {
            motor_t *pmotor = NULL;
            pmotor = &motors._[i];
            // 统计数据反馈频率
            pmotor->INFO.refresh_cnt++;
            // 解算原始数据
            if (IS_RM_QUAD_MOTOR(*pmotor) == true) {
#if NUM_OF_RM_QUAD_MOTOR
                parse_RM_QUAD_motor_data(pmotor, rx_buffer);
#endif
            } else if (IS_DM_MOTOR(*pmotor) == true) {
#if NUM_OF_DM_MOTOR
                parse_DM_motor_data(pmotor, rx_buffer);
#endif
            } else if (IS_DM_QUAD_MOTOR(*pmotor) == true) {
#if NUM_OF_DM_QUAD_MOTOR
                parse_DM_QUAD_motor_data(pmotor, rx_buffer);
#endif
            } else if (IS_LK_QUAD_MOTOR(*pmotor) == true) {
#if NUM_OF_LK_QUAD_MOTOR
                parse_LK_QUAD_motor_data(pmotor, rx_buffer);
#endif
            }
            return HAL_OK;
        }
    }
    return HAL_ERROR;
}

///<<<发送控制数据总线>>>///
// 开辟内存
// 此处可清晰查看已有的控制帧头占用情况
// 如需添加新的控制帧，请先在此处查看是否有空缺
typedef struct {
    RM_QUAD_motor_transmit_msg_t _rm0x1FF, _rm0x200, _0x2FF;
    LK_QUAD_motor_transmit_msg_t _0x280;
    DM_QUAD_motor_transmit_msg_t _dm0x1FF, _dm0x200, _0x3FE, _0x4FE;
    DM_motor_transmit_msg_t _0x00[10]; // 使用_0x00[0],代表ID为0x01的发出的数据
                                       // ID:1, Master_ID:0x11, CAN_ID:0x01
} Tx_msg_canbus_t;
Tx_msg_canbus_t Tx_msg_on_hcan1_bus = { 0 }, Tx_msg_on_hcan2_bus = { 0 },
                Tx_msg_on_hcan3_bus = { 0 };

typedef struct {
    UT_motor_transmit_msg_t _0;
} Tx_msg_uartbus_t;
Tx_msg_uartbus_t Tx_msg_on_huart_bus = { 0 };

extern CAN_HandleTypeDef hcan1, hcan2
#if defined(STM32H723xx) || defined(STM32G473xx)
        ,
        hcan3;
extern UART_HandleTypeDef huart2;
#else
        ;
#endif

// CAN总线部署函数
static MOTORLIB_StatusTypeDef deploy_tx_msg_on_canbus(CAN_HandleTypeDef **this_ppcan,
                                                      Tx_msg_canbus_t **ppcanbus)
{
    MOTORLIB_StatusTypeDef rslt = MOTORLIB_OK;
    if (*this_ppcan == &hcan1) {
        *ppcanbus = &Tx_msg_on_hcan1_bus;
    } else if (*this_ppcan == &hcan2) {
        *ppcanbus = &Tx_msg_on_hcan2_bus;
    }
#if defined(STM32H723xx) || defined(STM32G473xx)
    else if (*this_ppcan == &hcan3) {
        *ppcanbus = &Tx_msg_on_hcan3_bus;
    }
#endif
    else {
        rslt = MOTORLIB_ERROR;
    }
    return rslt;
}

// UART总线部署函数
static MOTORLIB_StatusTypeDef deploy_tx_msg_on_uartbus(UART_HandleTypeDef **this_ppuart,
                                                       Tx_msg_uartbus_t **ppuartbus)
{
    MOTORLIB_StatusTypeDef rslt = MOTORLIB_OK;
    if (0) {
    }
#if defined(STM32H723xx) || defined(STM32G473xx)
    else if (*this_ppuart == &huart2) {
        *ppuartbus = &Tx_msg_on_huart_bus;
    }
#endif
    else {
        rslt = MOTORLIB_ERROR;
    }
    return rslt;
}

// 清空总线函数
static MOTORLIB_StatusTypeDef clear_all_tx_msg(void)
{
    memset(&Tx_msg_on_hcan1_bus, 0, sizeof(Tx_msg_canbus_t));
    memset(&Tx_msg_on_hcan2_bus, 0, sizeof(Tx_msg_canbus_t));
    memset(&Tx_msg_on_hcan3_bus, 0, sizeof(Tx_msg_canbus_t));

    memset(&Tx_msg_on_huart_bus, 0, sizeof(Tx_msg_uartbus_t));

    return MOTORLIB_OK;
}

// 设置响应电机的索引范围函数
static MOTORLIB_StatusTypeDef set_motor_index_range(motors_type_t type, uint8_t *num,
                                                    uint8_t *index)
{ // TODO: 电机库软初始化自适应
    MOTORLIB_StatusTypeDef rslt = MOTORLIB_OK;
    switch (type) {
    case CHASSIS_MOTORS:
        *num = CHASSIS_MOTORS_NUM;
        *index = CHASSIS_MOTORS_INDEX;
        break;
    case GIMBAL_MOTORS:
        *num = GIMBAL_MOTORS_NUM;
        *index = GIMBAL_MOTORS_INDEX;
        break;
    case SHOOTER_MOTORS:
        *num = SHOOTER_MOTORS_NUM;
        *index = SHOOTER_MOTORS_INDEX;
        break;
    case FRIC_MOTORS:
        *num = FRIC_MOTORS_NUM;
        *index = FRIC_MOTORS_INDEX;
        break;
    case DIAL_MOTORS:
        *num = DIAL_MOTORS_NUM;
        *index = DIAL_MOTORS_INDEX;
        break;
    case ARM_MOTORS:
        *num = ARM_MOTORS_NUM;
        *index = ARM_MOTORS_INDEX;
        break;
    case TEST_MOTORS:
        *num = TEST_MOTORS_NUM;
        *index = TEST_MOTORS_INDEX;
        break;
    case DEFAULT:
        *num = NUM_OF_ALL_MOTOR;
        *index = 0;
        break;
    default:
        *num = 0, *index = 0;
        rslt = MOTORLIB_ERROR;
    }
    return rslt;
}

/**
 * @brief   一次性发送所有电机的数据(未完全解耦
 * @param[in]  STRUCT_TYPE  机构类型
 * @return  MOTORLIB_OK if success otherwise MOTORLIB_ERROR
 */
HAL_StatusTypeDef set_all_motor_output(motors_type_t motors_type)
{
    MOTORLIB_StatusTypeDef rslt = MOTORLIB_OK;
    uint8_t motors_num = 0, motors_init_index = 0;
    // 清空总线数据
    rslt |= clear_all_tx_msg();
    // 选择发送机构
    rslt |= set_motor_index_range(motors_type, &motors_num, &motors_init_index);
    // 遍历机构电机
    for (uint8_t i = motors_init_index; i < motors_init_index + motors_num; i++) {
        // 检测电机是否注册
        if (!motors._[i].register_state) {
            continue;
        }
        // 检测电机是否指定发送端口
        if (motors._[i].pcom == NULL) {
            SetMotorErrorCode(motors._[i], MotorError_NoSendingPortSpecified);
            rslt |= MOTORLIB_ERROR;
            continue;
        }
        // 部署发送数据总线
        uint32_t *pbus = NULL;
        Tx_msg_uartbus_t *puartbus = NULL;
        UART_HandleTypeDef *this_puart = NULL;
        Tx_msg_canbus_t *pcanbus = NULL;
        CAN_HandleTypeDef *this_pcan = NULL;
        if (motors._[i].com_type == COM_UART) {
            UNUSED(pcanbus), UNUSED(this_pcan);
            UART_HandleTypeDef *this_puart = (UART_HandleTypeDef *)(motors._[i].pcom);
            rslt |= deploy_tx_msg_on_uartbus(&this_puart, &puartbus);
            pbus = (uint32_t *)puartbus;
        } else if (motors._[i].com_type == COM_CAN || motors._[i].com_type == COM_FDCAN) {
            UNUSED(puartbus), UNUSED(this_puart);
            CAN_HandleTypeDef *this_pcan = (CAN_HandleTypeDef *)(motors._[i].pcom);
            rslt |= deploy_tx_msg_on_canbus(&this_pcan, &pcanbus);
            pbus = (uint32_t *)pcanbus;
        } else {
            // 指定的电机通信协议不被支持
            SetMotorErrorCode(motors._[i], MotorError_ComTypeError);
            rslt |= MOTORLIB_ERROR;
            continue;
        }
        // 检测总线是否部署成功
        if (pbus == NULL) {
            SetMotorErrorCode(motors._[i], MotorError_NoSendingPortSpecified);
            rslt |= MOTORLIB_ERROR;
            continue;
        }
        // 根据电机类型挂载数据
        ///<<< RM电机 >>>///
        if (IS_RM_QUAD_MOTOR(motors._[i]) == true) {
#if NUM_OF_RM_QUAD_MOTOR
            RM_QUAD_motor_transmit_msg_t *RM_QUAD_mail = NULL;
            switch (motors._[i].type.tx_base_ID) {
            case 0x1ff:
                RM_QUAD_mail = &((Tx_msg_canbus_t *)pbus)->_rm0x1FF;
                break;
            case 0x200:
                RM_QUAD_mail = &((Tx_msg_canbus_t *)pbus)->_rm0x200;
                break;
            case 0x2ff:
                RM_QUAD_mail = &((Tx_msg_canbus_t *)pbus)->_0x2FF;
                break;
            }
            if (RM_QUAD_mail == NULL) {
                SetMotorErrorCode(motors._[i], MotorError_TransmitMotorIdError);
                rslt |= MOTORLIB_ERROR;
                continue;
            } else {
                rslt |= mount_RM_QUAD_onto_bus(&motors._[i], RM_QUAD_mail);
            }
#endif
        }
        ///<<< 瓴控电机 >>>///
        else if (IS_LK_QUAD_MOTOR(motors._[i])) {
#if NUM_OF_LK_QUAD_MOTOR
            LK_QUAD_motor_transmit_msg_t *LK_QUAD_mail = NULL;
            switch (motors._[i].type.tx_base_ID) {
            case 0x280:
                LK_QUAD_mail = &((Tx_msg_canbus_t *)pbus)->_0x280;
                break;
            }
            if (LK_QUAD_mail == NULL) {
                SetMotorErrorCode(motors._[i], MotorError_TransmitMotorIdError);
                rslt |= MOTORLIB_ERROR;
                continue;
            } else {
                rslt |= mount_LK_QUAD_onto_bus(&motors._[i], LK_QUAD_mail);
            }
#endif
        }
        ///<<< 达妙电机 >>>///
        else if (IS_DM_MOTOR(motors._[i]) == true) {
#if NUM_OF_DM_MOTOR
            DM_motor_transmit_msg_t *DM_mail = NULL;
            uint8_t ID = motors._[i].type.offset_ID;
            switch (motors._[i].type.tx_base_ID) {
            case 0x00:
                DM_mail = &((Tx_msg_canbus_t *)pbus)->_0x00[ID];
                break;
            }
            if (DM_mail == NULL) {
                SetMotorErrorCode(motors._[i], MotorError_TransmitMotorIdError);
                rslt |= MOTORLIB_ERROR;
                continue;
            } else {
                rslt |= mount_DM_onto_bus(&motors._[i], DM_mail, ID);
            }

#endif
        }
        // 一拖四的达妙电机控制
        else if (IS_DM_QUAD_MOTOR(motors._[i]) == true) {
#if NUM_OF_DM_QUAD_MOTOR
            DM_QUAD_motor_transmit_msg_t *DM_QUAD_mail == NULL;
            switch (motors._[i].type.tx_base_ID) {
            case 0x3FE:
                DM_QUAD_mail = &((Tx_msg_canbus_t *)pbus)->_0x3FE;
                break;
            case 0x4FE:
                DM_QUAD_mail = &((Tx_msg_canbus_t *)pbus)->_0x4FE;
                break;
            case 0x200:
                DM_QUAD_mail = &((Tx_msg_canbus_t *)pbus)->_dm0x200;
                break;
            case 0x1FF:
                DM_QUAD_mail = &((Tx_msg_canbus_t *)pbus)->_dm0x1FF;
                break;
            }
            if (DM_QUAD_mail == NULL) {
                SetMotorErrorCode(motors._[i], MotorError_TransmitMotorIdError);
                rslt |= MOTORLIB_ERROR;
                continue;
            } else {
                rslt |= mount_DM_QUAD_onto_bus(&motors._[i], DM_QUAD_mail);
            }
#endif
        }
        ///<<< 宇树电机 >>>///
        else if (IS_UT_MOTOR(motors._[i]) == true) {
#if NUM_OF_UT_MOTOR
            UT_motor_transmit_msg_t *UT_mail = NULL;
            switch (motors._[i].type.tx_base_ID) {
            case 0:
                UT_mail = &((Tx_msg_uartbus_t *)pbus)->_0;
                break;
            }
            if (UT_mail == NULL) {
                SetMotorErrorCode(motors._[i], MotorError_TransmitMotorIdError);
                rslt |= MOTORLIB_ERROR;
                continue;
            } else {
                rslt |= mount_UT_onto_bus(&motors._[i], UT_mail);
            }
#endif
        }
        ///<<< 未指定电机 >>>///
        else {
            SetMotorErrorCode(motors._[i], MotorError_InsNotFound);
            rslt |= MOTORLIB_ERROR;
            continue;
        }
    }
// 发送总线上的数据
#if NUM_OF_RM_QUAD_MOTOR
    rslt |= set_RM_QUAD_motor_output();
#endif
#if NUM_OF_LK_QUAD_MOTOR
    rslt |= set_LK_QUAD_motor_output();
#endif
#if NUM_OF_DM_MOTOR
    rslt |= set_DM_motor_output();
#endif
#if NUM_OF_DM_QUAD_MOTOR
    rslt |= set_DM_QUAD_motor_output();
#endif
#if NUM_OF_UT_MOTOR
    rslt |= set_UT_motor_output();
#endif
    return rslt == MOTORLIB_OK ? HAL_OK : HAL_ERROR;
}

///<<<其他功能函数>>>///
/**
 * @brief	刷新电机状态,检测电机是否离线,检测是否正确初始化
 */
void update_motor_status(motors_type_t motors_type)
{
    uint8_t motors_num, motors_init_index;
    switch (motors_type) {
    case CHASSIS_MOTORS:
        motors_num = CHASSIS_MOTORS_NUM;
        motors_init_index = CHASSIS_MOTORS_INDEX;
        break;
    case GIMBAL_MOTORS:
        motors_num = GIMBAL_MOTORS_NUM;
        motors_init_index = GIMBAL_MOTORS_INDEX;
        break;
    case SHOOTER_MOTORS:
        motors_num = SHOOTER_MOTORS_NUM;
        motors_init_index = SHOOTER_MOTORS_INDEX;
        break;
    case FRIC_MOTORS:
        motors_num = FRIC_MOTORS_NUM;
        motors_init_index = FRIC_MOTORS_INDEX;
        break;
    case DIAL_MOTORS:
        motors_num = DIAL_MOTORS_NUM;
        motors_init_index = DIAL_MOTORS_INDEX;
        break;
    case ARM_MOTORS:
        motors_num = ARM_MOTORS_NUM;
        motors_init_index = ARM_MOTORS_INDEX;
        break;
    case TEST_MOTORS:
        motors_num = TEST_MOTORS_NUM;
        motors_init_index = TEST_MOTORS_INDEX;
        break;
    case DEFAULT:
        motors_num = NUM_OF_ALL_MOTOR;
        motors_init_index = 0;
        break;
    default:
        motors_num = NUM_OF_ALL_MOTOR;
        motors_init_index = 0;
        break;
    }
    float dt = 1.f / MOTOR_DETECT_FREQ;

    for (uint8_t i = motors_init_index; i < motors_init_index + motors_num; i++) {
        if (!motors._[i].register_state) {
            continue;
        }
        if (dt)
            motors._[i].INFO.refresh_freq = M_LPF_update(&motors._[i].INFO.refresh_filter,
                                                         (motors._[i].INFO.refresh_cnt) / dt);
        motors._[i].INFO.refresh_cnt = 0;
        if (motors._[i].INFO.refresh_freq <= MIN_MOTOR_REFRESH_FREQ) {
            SetMotorErrorCode(motors._[i], MotorError_MotorOffline);
            motors._[i].INFO.is_online = false;
        } else
            motors._[i].INFO.is_online = true;
        if (motors._[i].real.tempture >= 60.f)
            SetMotorErrorCode(motors._[i], MotorError_OverHeat);
    }
}
/**
 *	@brief
 *设置电机库内注册的电机零点,基于给定zero_scale校正rel_angle，并不是对电机内寄存器进行操作
 *	@note  	于can初始化后调用
 *	@param 	zero_scale	零点位置 量程为该电机span
 */
void set_motor_zero_angle(motor_t *pmotor, uint16_t zero_scale)
{
    float temp_scale = 0;
    if (pmotor->real.raw_scale > span_of(*pmotor) / RR_of(*pmotor) + zero_scale) {
        temp_scale = -((span_of(*pmotor) - pmotor->real.raw_scale) + zero_scale);
    } else
        temp_scale = pmotor->real.raw_scale - zero_scale;

    pmotor->real.rel_angle = 2 * PI * temp_scale / (span_of(*pmotor) * RR_of(*pmotor));
    pmotor->real.abs_angle = pmotor->real.rel_angle;
}

/**
 * @brief 给所有电机强制发0
 * @note 建议放在一个低速定时器中 防止发太快把can发爆
 */
HAL_StatusTypeDef shutdown_all_motor(void)
{
    HAL_StatusTypeDef rslt = HAL_OK;
    for (uint8_t i = 0; i < NUM_OF_ALL_MOTOR; i++) {
        if (!motors._[i].register_state) {
            continue;
        }
        motors._[i].T_ff = 0;
        motors._[i].AUX.Kp = 0;
        motors._[i].AUX.Kd = 0;
#if NUM_OF_LK_QUAD_MOTOR
        if (IS_LK_QUAD_MOTOR(motors._[i]))
            motors._[i].AUX.EN = false;
#endif
#if NUM_OF_DM_MOTOR
        if (IS_DM_MOTOR(motors._[i]))
            motors._[i].AUX.EN = false;
#endif
#if NUM_OF_UT_MOTOR
        if (IS_UT_MOTOR(motors._[i]))
            motors._[i].AUX.EN = false, shut_all_UT_motor_output();
#endif
#if NUM_OF_RM_QUAD_MOTOR
        if (IS_RM_QUAD_MOTOR(motors._[i]))
            motors._[i].AUX.EN = false;
#endif
        rslt |= shutdown_motor(&motors._[i]);
    }

    return rslt;
}
/**
 * @brief 给特定电机发强制0
 * @param pmotor 特定电机指针
 * @attention 对于一拖四类型 调用此函数会关闭同控制帧的所有电机
 */
MOTORLIB_StatusTypeDef shutdown_motor(motor_t *pmotor)
{
    MOTORLIB_StatusTypeDef rslt = MOTORLIB_OK;
    if (IS_RM_QUAD_MOTOR(*pmotor)) {
#if NUM_OF_RM_QUAD_MOTOR
        shut_RM_QUAD_motor_output((CAN_HandleTypeDef *)pmotor->pcom, tx_stdid_of(*pmotor),
                                  pmotor->com_type);
#endif
    } else if (IS_LK_QUAD_MOTOR(*pmotor)) {
#if NUM_OF_LK_QUAD_MOTOR
        shut_LK_QUAD_motor_output((CAN_HandleTypeDef *)pmotor->pcom, tx_stdid_of(*pmotor),
                                  pmotor->com_type);
#endif
    } else if (IS_DM_MOTOR(*pmotor)) {
#if NUM_OF_DM_MOTOR
        disable_single_DM_motor_output((CAN_HandleTypeDef *)pmotor->pcom, tx_stdid_of(*pmotor),
                                       pmotor->com_type);
#endif
    }

    else if (IS_DM_QUAD_MOTOR(*pmotor)) {
#if NUM_OF_DM_QUAD_MOTOR
        shut_DM_QUAD_several_motor_output((CAN_HandleTypeDef *)pmotor->pcom, tx_stdid_of(*pmotor),
                                          pmotor->com_type);
#endif
    }
#if NUM_OF_UT_MOTOR
    else if (IS_UT_MOTOR(*pmotor)) {
        disable_UT_motor_output(pmotor);
    }
#endif
    else {
        rslt = MOTORLIB_ERROR;
    }
    return rslt;
}
/**
 * @brief 给特定机构的所有电机强制发0
 * @param motors_type 特定机构类型
 * @attention 对于一拖四类型 调用此函数会关闭同控制帧的所有电机
 */
MOTORLIB_StatusTypeDef shutdown_motors_of_structure(motors_type_t motors_type)
{
    MOTORLIB_StatusTypeDef rslt = MOTORLIB_OK;
    motors_t *pmotors = get_motors_ptr();
    uint8_t motors_num = 0, motors_init_index = 0;
    switch (motors_type) {
    case CHASSIS_MOTORS:
        motors_num = CHASSIS_MOTORS_NUM;
        motors_init_index = CHASSIS_MOTORS_INDEX;
        break;
    case GIMBAL_MOTORS:
        motors_num = GIMBAL_MOTORS_NUM;
        motors_init_index = GIMBAL_MOTORS_INDEX;
        break;
    case SHOOTER_MOTORS:
        motors_num = SHOOTER_MOTORS_NUM;
        motors_init_index = SHOOTER_MOTORS_INDEX;
        break;
    case FRIC_MOTORS:
        motors_num = FRIC_MOTORS_NUM;
        motors_init_index = FRIC_MOTORS_INDEX;
        break;
    case DIAL_MOTORS:
        motors_num = DIAL_MOTORS_NUM;
        motors_init_index = DIAL_MOTORS_INDEX;
        break;
    case ARM_MOTORS:
        motors_num = ARM_MOTORS_NUM;
        motors_init_index = ARM_MOTORS_INDEX;
        break;
    case TEST_MOTORS:
        motors_num = TEST_MOTORS_NUM;
        motors_init_index = TEST_MOTORS_INDEX;
        break;
    case DEFAULT:
        motors_num = NUM_OF_ALL_MOTOR;
        motors_init_index = 0;
        break;
    }
    if (!motors_num)
        return MOTORLIB_ERROR;
    for (uint8_t i = motors_init_index; i < motors_init_index + motors_num; i++) {
        if (!motors._[i].register_state) {
            continue;
        }
        rslt = shutdown_motor(&pmotors->_[i]);
        if (rslt != MOTORLIB_OK)
            break;
    }
    return rslt;
}
/**
 * @brief 检测某机构是否存在电机掉线的情况
 * @param motors_type 机构类型
 */
uint8_t is_motors_offline(motors_type_t motors_type)
{
    motors_t *pmotors = get_motors_ptr();
    uint8_t rslt = 0;
    uint8_t motors_num = 0, motors_init_index = 0;
    switch (motors_type) {
    case CHASSIS_MOTORS:
        motors_num = CHASSIS_MOTORS_NUM;
        motors_init_index = CHASSIS_MOTORS_INDEX;
        break;
    case GIMBAL_MOTORS:
        motors_num = GIMBAL_MOTORS_NUM;
        motors_init_index = GIMBAL_MOTORS_INDEX;
        break;
    case SHOOTER_MOTORS:
        motors_num = SHOOTER_MOTORS_NUM;
        motors_init_index = SHOOTER_MOTORS_INDEX;
        break;
    case FRIC_MOTORS:
        motors_num = FRIC_MOTORS_NUM;
        motors_init_index = FRIC_MOTORS_INDEX;
        break;
    case DIAL_MOTORS:
        motors_num = DIAL_MOTORS_NUM;
        motors_init_index = DIAL_MOTORS_INDEX;
        break;
    case ARM_MOTORS:
        motors_num = ARM_MOTORS_NUM;
        motors_init_index = ARM_MOTORS_INDEX;
        break;
    case TEST_MOTORS:
        motors_num = TEST_MOTORS_NUM;
        motors_init_index = TEST_MOTORS_INDEX;
        break;
    case DEFAULT:
        motors_num = NUM_OF_ALL_MOTOR;
        motors_init_index = 0;
        break;
    }
    if (!motors_num)
        return MOTORLIB_ERROR;
    for (uint8_t i = motors_init_index; i < motors_init_index + motors_num; i++) {
        if (!motors._[i].register_state) {
            continue;
        }
        rslt |= IS_MOTOR_OFFLINE(pmotors->_[i]);
    }
    return rslt;
}
motors_t *get_motors_ptr(void)
{
    return &motors;
};
