/**
 * @file motor_ctrl_DM.c
 *
 * @brief DM电机API
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#include <stdint.h>
#ifndef HAL_INCLUDE
#include "main.h"
#else
#include HAL_INCLUDE
#endif // !HAL_INCLUDE
#include "./dev_DM.h"

#if NUM_OF_DM_MOTOR
/**
 * @brief	达妙电机型号初始化函数
 * @param[in]	pmotor:要被初始化的电机
 * @param[in] motor_model:电机实际型号
 * @param[in]	work_mode:电机工作模式
 * @param[in]	com_type:电机通信类型
 * @param[in]	pcom:电机通信接口句柄指针
 * @param[in]	ID:电机ID号,由软件设置可知
 * @param[in]	RR:电机外套减速箱减速比 没有就填1
 * @param[in]	init_lpf:初始化滤波器
 * @param[in]	inst:电机所属机构类型
 */
void DM_motor_register(motor_t *pmotor, DM_motor_model_t motor_model, motor_work_mode_t work_mode,
                       motor_com_type_t com_type, uint32_t *pcan, uint8_t ID, float RR,
                       const M_LPF_t *init_lpf, motor_institution_t inst)
{
    memset(&pmotor->INFO.error_set, 0, sizeof(pmotor->INFO.error_set));
    // 电机类型初始化
    switch (motor_model) {
    case motor_model_DM_DM8009:
        pmotor->type = DM_motor_lib_DM8009;
        break;
    case motor_model_DM_DM4310:
        pmotor->type = DM_motor_lib_DM4310;
        break;
    case motor_model_DM_DM4340:
        pmotor->type = DM_motor_lib_DM4340;
        break;
    case motor_model_DM_DM3519:
        pmotor->type = DM_motor_lib_DM3519;
        break;
    case motor_model_DM_DM6006:
        pmotor->type = DM_motor_lib_DM6006;
        break;
    default:
        SetMotorErrorCode(*pmotor, MotorError_CanInitFail);
        return;
    }

    pmotor->pcom = pcan;
    pmotor->type.offset_ID = ID;
    pmotor->real.last_raw_scale = 0;
    pmotor->real.abs_angle = 0;

    pmotor->INFO.refresh_filter = (M_LPF_t){ .fc = 5e-1f, .ts = 1.f / MOTOR_DETECT_FREQ };

    pmotor->institution = inst;

    pmotor->work_mode = work_mode;
    pmotor->com_type = com_type;
    if (work_mode == QUAD_CURR || work_mode == QUAD_VDES) {
        SetMotorErrorCode(*pmotor, MotorError_WorkModeConflict);
    }

    pmotor->AUX.EN = false;
    pmotor->AUX.QUAD = false;

    pmotor->type.reduction_ratio = RR;

    memcpy(&pmotor->lpf_fltr, init_lpf, sizeof(M_LPF_t));

    pmotor->register_state = M_REGISTERED;
}
/**
 * @brief	达妙电机型号注销函数
 */
void DM_motor_cancell(motor_t *pmotor)
{
    memset(pmotor, 0, sizeof(motor_t));
}

/**
 * @brief 发送达妙电机控制指令 一帧CAN对应一个达妙电机
 * @param[in]  hcan    the CAN handler to transmit raw data
 * @param[in]  tx_msg   the pointer to MIT command data struct
 * @param[in]  id       range starts from 0x100
 * @return MOTORLIB_OK if success otherwise MOTORLIB_ERROR
 */
MOTORLIB_StatusTypeDef set_DM_motor_output_MIT(CAN_HandleTypeDef *hcan,
                                               DM_motor_transmit_MIT_msg_t *tx_msg, uint32_t id,
                                               motor_com_type_t com_type)
{
    HAL_StatusTypeDef HALrslt = HAL_OK;
    // 转换发送数据
    // uint16_t torque_offset =
    //     float_to_uint(0, -DM8009_MIT_T_MAX, DM8009_MIT_T_MAX, 12);
    // 注意,此部分值为编码值,不能直接设零!
    uint8_t tx_buffer[CAN_DATA_LEN] = { 0 };
    tx_buffer[0] = (uint8_t)((tx_msg->expt_scale & 0xFF00) >> 8);
    tx_buffer[1] = (uint8_t)(tx_msg->expt_scale & 0x00FF);
    tx_buffer[2] = (uint8_t)((tx_msg->expt_vel & 0x0FF0) >> 4);
    tx_buffer[3] = (uint8_t)((tx_msg->expt_vel & 0x000F) << 4 | ((tx_msg->Kp & 0x0FF0) >> 8));
    tx_buffer[4] = (uint8_t)(tx_msg->Kp & 0x000F);
    tx_buffer[5] = (uint8_t)((tx_msg->Kd & 0x0FF0) >> 4);
    tx_buffer[6] = (uint8_t)((tx_msg->Kd & 0x000F) << 4 | ((tx_msg->torque_offset & 0x0F00) >> 8));
    tx_buffer[7] = (uint8_t)(tx_msg->torque_offset & 0x00FF);
    bool is_fdcan = (com_type == COM_FDCAN) ? true : false;
    HALrslt |= can_transmit_data(hcan, id, tx_buffer, CAN_DATA_LEN, is_fdcan);
    return HALrslt == HAL_OK ? MOTORLIB_OK : MOTORLIB_ERROR;
}
/**
 * @brief 发送达妙电机控制指令 一帧CAN对应一个达妙电机
 * @note 位置速度模式 控制指令给定位置和速度
 * @details p_des:32 v_des:32 DLC:8byte
 * @param[in]  hcan    the CAN handler to transmit raw data
 * @param[in]  tx_msg   the pointer to PDES_VDES command data struct
 * @param[in]  id       CAN ID of the motor + 0x100
 * @return MOTORLIB_OK if success otherwise MOTORLIB_ERROR
 */
MOTORLIB_StatusTypeDef set_DM_motor_output_PDESVDES(CAN_HandleTypeDef *hcan,
                                                    DM_motor_transmit_PDESVDES_msg_t *tx_msg,
                                                    uint32_t id, motor_com_type_t com_type)
{
    HAL_StatusTypeDef HALrslt = HAL_OK;
    // 转换发送数据
    // uint16_t torque_offset =
    //     float_to_uint(0, -DM8009_MIT_T_MAX, DM8009_MIT_T_MAX, 12);
    // 注意,此部分值为编码值,不能直接设零!
    uint8_t tx_buffer[CAN_DATA_LEN] = { 0 };
    memcpy(tx_buffer, &tx_msg->expt_scale, CAN_DATA_LEN / 2);
    memcpy(&tx_buffer[4], &tx_msg->expt_vel, CAN_DATA_LEN / 2);
    bool is_fdcan = (com_type == COM_FDCAN) ? true : false;
    HALrslt |= can_transmit_data(hcan, id + 0x100, tx_buffer, CAN_DATA_LEN, is_fdcan);
    return HALrslt == HAL_OK ? MOTORLIB_OK : MOTORLIB_ERROR;
}

/**
 * @brief 发送达妙电机控制指令 一帧CAN对应一个达妙电机
 * @note 速度模式 控制指令给定速度
 * @details v_des:32 DLC:4byte
 * @param[in]  hcan    the CAN handler to transmit raw data
 * @param[in]  tx_msg   the pointer to VDES command data struct
 * @param[in]  id       CAN ID of the motor + 0x200
 * @return MOTORLIB_OK if success otherwise MOTORLIB_ERROR
 */
MOTORLIB_StatusTypeDef set_DM_motor_output_VDES(CAN_HandleTypeDef *hcan,
                                                DM_motor_transmit_VDES_msg_t *tx_msg, uint32_t id,
                                                motor_com_type_t com_type)
{
    HAL_StatusTypeDef HALrslt = HAL_OK;
    uint8_t tx_buffer[CAN_DATA_LEN / 2] = { 0 };
    memcpy(tx_buffer, tx_msg, CAN_DATA_LEN / 2);
    bool is_fdcan = (com_type == COM_FDCAN) ? true : false;
    HALrslt |= can_transmit_data(hcan, id + 0x200, tx_buffer, CAN_DATA_LEN / 2, is_fdcan);
    return HALrslt == HAL_OK ? MOTORLIB_OK : MOTORLIB_ERROR;
}

/**
 * @brief 发送达妙电机控制指令 一帧CAN对应一个达妙电机
 * @note 力位混控模式 控制指令给定位置速度饱和电流
 * @attention 其中速度项在电调中被除100
 * 饱和电流项在电调中被除10000得到比例再乘最大实际电流
 * @details p_des:32 v_des:16 imax:16 DLC:8byte
 * @param[in]  hcan    the CAN handler to transmit raw data
 * @param[in]  tx_msg   the pointer to E_MIT command data struct
 * @param[in]  id       CAN ID of the motor + 0x200
 * @return MOTORLIB_OK if success otherwise MOTORLIB_ERROR
 */
MOTORLIB_StatusTypeDef set_DM_motor_output_E_MIT(CAN_HandleTypeDef *hcan,
                                                 DM_motor_transmit_E_MIT_msg_t *tx_msg, uint32_t id,
                                                 motor_com_type_t com_type)
{
    HAL_StatusTypeDef HALrslt = HAL_OK;
    uint8_t tx_buffer[CAN_DATA_LEN] = { 0 };
    float f_expt_scale = tx_msg->expt_scale;
    memcpy(tx_buffer, &f_expt_scale, CAN_DATA_LEN / 2);
    tx_buffer[4] = (uint8_t)((tx_msg->expt_vel_x100) >> 8);
    tx_buffer[5] = (uint8_t)(tx_msg->expt_vel_x100);
    tx_buffer[6] = (uint8_t)((tx_msg->imax_x10000) >> 8);
    tx_buffer[7] = (uint8_t)(tx_msg->imax_x10000);
    bool is_fdcan = (com_type == COM_FDCAN) ? true : false;
    HALrslt |= can_transmit_data(hcan, id + 0x300, tx_buffer, CAN_DATA_LEN, is_fdcan);
    return HALrslt == HAL_OK ? MOTORLIB_OK : MOTORLIB_ERROR;
}

/**
 * @brief DM电机-将给定值转换为MIT控制指令
 * @param motor 对象电机指针
 * @return DM_motor_transmit_MIT_msg_t
 */
static DM_motor_transmit_MIT_msg_t convert_DM_MIT_tx_data(motor_t *motor)
{
    DM_motor_transmit_MIT_msg_t dm_cv_sol = { 0 };
    if (motor->work_mode == MIT_TT || motor->work_mode == MIT_VDES ||
        motor->work_mode == MIT_PDESVDES) {
        // 比例因子和阻尼因子同时设0时 电机输出给定t_ff
        if (motor->type.model == motor_model_DM_DM8009) {
            dm_cv_sol.expt_scale =
                    float_to_uint(motor->P_des, -DM8009_MIT_P_MAX, DM8009_MIT_P_MAX, 16);
            dm_cv_sol.expt_vel =
                    float_to_uint(motor->V_des, -DM8009_MIT_V_MAX, DM8009_MIT_V_MAX, 12);
            dm_cv_sol.Kp = (motor->work_mode == MIT_TT) ?
                                   0 :
                                   float_to_uint(motor->AUX.Kp, 0, DM8009_MIT_KP_MAX, 12);
            dm_cv_sol.Kd = (motor->work_mode == MIT_TT) ?
                                   0 :
                                   float_to_uint(motor->AUX.Kd, 0, DM8009_MIT_KD_MAX, 12);
            dm_cv_sol.torque_offset =
                    float_to_uint(motor->T_ff, -DM8009_MIT_T_MAX, DM8009_MIT_T_MAX, 12);
        } else if (motor->type.model == motor_model_DM_DM4310) {
            dm_cv_sol.expt_scale =
                    float_to_uint(motor->P_des, -DM4310_MIT_P_MAX, DM4310_MIT_P_MAX, 16);
            dm_cv_sol.expt_vel =
                    float_to_uint(motor->V_des, -DM4310_MIT_V_MAX, DM4310_MIT_V_MAX, 12);
            dm_cv_sol.Kp = (motor->work_mode == MIT_TT) ?
                                   0 :
                                   float_to_uint(motor->AUX.Kp, 0, DM4310_MIT_KP_MAX, 12);
            dm_cv_sol.Kd = (motor->work_mode == MIT_TT) ?
                                   0 :
                                   float_to_uint(motor->AUX.Kd, 0, DM4310_MIT_KD_MAX, 12);
            dm_cv_sol.torque_offset =
                    float_to_uint(motor->T_ff, -DM4310_MIT_T_MAX, DM4310_MIT_T_MAX, 12);
        } else if (motor->type.model == motor_model_DM_DM4340) {
            dm_cv_sol.expt_scale =
                    float_to_uint(motor->P_des, -DM4340_MIT_P_MAX, DM4340_MIT_P_MAX, 16);
            dm_cv_sol.expt_vel =
                    float_to_uint(motor->V_des, -DM4340_MIT_V_MAX, DM4340_MIT_V_MAX, 12);
            dm_cv_sol.Kp = (motor->work_mode == MIT_TT) ?
                                   0 :
                                   float_to_uint(motor->AUX.Kp, 0, DM4340_MIT_KP_MAX, 12);
            dm_cv_sol.Kd = (motor->work_mode == MIT_TT) ?
                                   0 :
                                   float_to_uint(motor->AUX.Kd, 0, DM4340_MIT_KD_MAX, 12);
            dm_cv_sol.torque_offset =
                    float_to_uint(motor->T_ff, -DM4340_MIT_T_MAX, DM4340_MIT_T_MAX, 12);
        } else if (motor->type.model == motor_model_DM_DM3519) {
            dm_cv_sol.expt_scale =
                    float_to_uint(motor->P_des, -DM3519_MIT_P_MAX, DM3519_MIT_P_MAX, 16);
            dm_cv_sol.expt_vel =
                    float_to_uint(motor->V_des, -DM3519_MIT_V_MAX, DM3519_MIT_V_MAX, 12);
            dm_cv_sol.Kp = (motor->work_mode == MIT_TT) ?
                                   0 :
                                   float_to_uint(motor->AUX.Kp, 0, DM3519_MIT_KP_MAX, 12);
            dm_cv_sol.Kd = (motor->work_mode == MIT_TT) ?
                                   0 :
                                   float_to_uint(motor->AUX.Kd, 0, DM3519_MIT_KD_MAX, 12);
            dm_cv_sol.torque_offset =
                    float_to_uint(motor->T_ff, -DM3519_MIT_T_MAX, DM3519_MIT_T_MAX, 12);
        } else if (motor->type.model == motor_model_DM_DM6006) {
            dm_cv_sol.expt_scale =
                    float_to_uint(motor->P_des, -DM6006_MIT_P_MAX, DM6006_MIT_P_MAX, 16);
            dm_cv_sol.expt_vel =
                    float_to_uint(motor->V_des, -DM6006_MIT_V_MAX, DM6006_MIT_V_MAX, 12);
            dm_cv_sol.Kp = (motor->work_mode == MIT_TT) ?
                                   0 :
                                   float_to_uint(motor->AUX.Kp, 0, DM6006_MIT_KP_MAX, 12);
            dm_cv_sol.Kd = (motor->work_mode == MIT_TT) ?
                                   0 :
                                   float_to_uint(motor->AUX.Kd, 0, DM6006_MIT_KD_MAX, 12);
            dm_cv_sol.torque_offset =
                    float_to_uint(motor->T_ff, -DM6006_MIT_T_MAX, DM6006_MIT_T_MAX, 12);
        }
    } else {
        SetMotorErrorCode(*motor, MotorError_WorkModeConflict);
        memset(&dm_cv_sol, 0, sizeof(DM_motor_transmit_MIT_msg_t));
    }
    return dm_cv_sol; // 用指针的话不知为啥motor_ctrl那收不到数据?
}
/**
 * @brief DM电机-将给定值转换为位置速度控制指令
 * @param motor 对象电机指针
 * @return DM_motor_transmit_PDESVDES_msg_t
 */
static DM_motor_transmit_PDESVDES_msg_t convert_DM_PDESVDES_tx_data(motor_t *motor)
{
    DM_motor_transmit_PDESVDES_msg_t dm_cv_sol = { 0 };
    if (motor->work_mode == PDESVDES) {
        if (motor->type.model == motor_model_DM_DM8009) {
            dm_cv_sol.expt_scale = motor->P_des; // M_CLAMP(motor->P_des, DM8009_MIT_P_MAX);
            dm_cv_sol.expt_vel = M_CLAMP(motor->V_des, DM8009_MIT_V_MAX);
        } else if (motor->type.model == motor_model_DM_DM4310) {
            dm_cv_sol.expt_scale = motor->P_des; // M_CLAMP(motor->P_des, DM4310_MIT_P_MAX);
            dm_cv_sol.expt_vel = M_CLAMP(motor->V_des, DM4310_MIT_V_MAX);
        } else if (motor->type.model == motor_model_DM_DM4340) {
            dm_cv_sol.expt_scale = motor->P_des; // M_CLAMP(motor->P_des, DM4340_MIT_P_MAX);
            dm_cv_sol.expt_vel = M_CLAMP(motor->V_des, DM4340_MIT_V_MAX);
        } else if (motor->type.model == motor_model_DM_DM3519) {
            dm_cv_sol.expt_scale = motor->P_des; // M_CLAMP(motor->P_des, DM3519_MIT_P_MAX);
            dm_cv_sol.expt_vel = M_CLAMP(motor->V_des, DM3519_MIT_V_MAX);
        } else if (motor->type.model == motor_model_DM_DM6006) {
            dm_cv_sol.expt_scale = motor->P_des; // M_CLAMP(motor->P_des, DM6006_MIT_P_MAX);
            dm_cv_sol.expt_vel = M_CLAMP(motor->V_des, DM6006_MIT_V_MAX);
        }
    } else {
        SetMotorErrorCode(*motor, MotorError_WorkModeConflict);
        memset(&dm_cv_sol, 0, sizeof(DM_motor_transmit_PDESVDES_msg_t));
    }
    return dm_cv_sol;
}
/**
 * @brief DM电机-将给定值转换为速度控制指令
 * @param motor 对象电机指针
 * @return DM_motor_transmit_VDES_msg_t
 */
static DM_motor_transmit_VDES_msg_t convert_DM_VDES_tx_data(motor_t *motor)
{
    DM_motor_transmit_VDES_msg_t dm_cv_sol = { 0 };
    if (motor->work_mode == VDES) {
        if (motor->type.model == motor_model_DM_DM8009) {
            dm_cv_sol.expt_vel = M_CLAMP(motor->V_des, DM8009_MIT_V_MAX);
        } else if (motor->type.model == motor_model_DM_DM4310) {
            dm_cv_sol.expt_vel = M_CLAMP(motor->V_des, DM4310_MIT_V_MAX);
        } else if (motor->type.model == motor_model_DM_DM4340) {
            dm_cv_sol.expt_vel = M_CLAMP(motor->V_des, DM4340_MIT_V_MAX);
        } else if (motor->type.model == motor_model_DM_DM3519) {
            dm_cv_sol.expt_vel = M_CLAMP(motor->V_des, DM3519_MIT_V_MAX);
        } else if (motor->type.model == motor_model_DM_DM6006) {
            dm_cv_sol.expt_vel = M_CLAMP(motor->V_des, DM6006_MIT_V_MAX);
        }
    } else {
        SetMotorErrorCode(*motor, MotorError_WorkModeConflict);
        memset(&dm_cv_sol, 0, sizeof(DM_motor_transmit_VDES_msg_t));
    }
    return dm_cv_sol;
}

/**
 * @brief DM电机-将给定值转换为速度控制指令
 * @param motor 对象电机指针
 * @return DM_motor_transmit_E_MIT_msg_t
 */
static DM_motor_transmit_E_MIT_msg_t convert_DM_E_MIT_tx_data(motor_t *motor)
{
    DM_motor_transmit_E_MIT_msg_t dm_cv_sol = { 0 };
    dm_cv_sol.expt_scale = motor->P_des;
    dm_cv_sol.expt_vel_x100 =
            (uint16_t)(((motor->V_des < 0) ? -motor->V_des : motor->V_des) * 100.f);
    if (motor->work_mode == E_MIT) {
        if (motor->type.model == motor_model_DM_DM8009) {
            dm_cv_sol.imax_x10000 = (uint16_t)(((motor->T_ff < 0) ? -motor->T_ff : motor->T_ff) /
                                               Kn_DM8009 / DM8009_CURR_MAX * 10000.f);
        } else if (motor->type.model == motor_model_DM_DM4310) {
            dm_cv_sol.imax_x10000 = (uint16_t)(((motor->T_ff < 0) ? -motor->T_ff : motor->T_ff) /
                                               Kn_DM4310 / DM4310_CURR_MAX * 10000.f);
        } else if (motor->type.model == motor_model_DM_DM4340) {
            dm_cv_sol.imax_x10000 = (uint16_t)(((motor->T_ff < 0) ? -motor->T_ff : motor->T_ff) /
                                               Kn_DM4340 / DM4340_CURR_MAX * 10000.f);
        } else if (motor->type.model == motor_model_DM_DM3519) {
            dm_cv_sol.imax_x10000 = (uint16_t)(((motor->T_ff < 0) ? -motor->T_ff : motor->T_ff) /
                                               Kn_DM3519 / DM3519_CURR_MAX * 10000.f);
        } else if (motor->type.model == motor_model_DM_DM6006) {
            dm_cv_sol.imax_x10000 = (uint16_t)(((motor->T_ff < 0) ? -motor->T_ff : motor->T_ff) /
                                               Kn_DM6006 / DM6006_CURR_MAX * 10000.f);
        }
    } else {
        SetMotorErrorCode(*motor, MotorError_WorkModeConflict);
        memset(&dm_cv_sol, 0, sizeof(DM_motor_transmit_E_MIT_msg_t));
    }
    return dm_cv_sol;
}

typedef struct {
    CAN_HandleTypeDef *hcan;
    motor_work_mode_t mode;
    motor_com_type_t com_type;
    DM_motor_transmit_msg_t *tx_msg;
    uint32_t id;
    float refresh_freq; // 刷新频率
    float weight;       // 权重
    bool had_refresh;   // 避免某包因为转去发使能/失能帧,导致发送出数值为0的死包
    bool is_online;     // 电机在线与否
                        //(不用refresh_freq来判断是为了保证离线判断的一致性)
} DM_msg_buffer_t;

DM_msg_buffer_t DM_msg_bfr[NUM_OF_DM_MOTOR] = { 0 };
static MOTORLIB_StatusTypeDef load_DM_motor_output(motor_t *pmotor, DM_motor_transmit_msg_t *tx_msg)
{
    static uint8_t i = 0;
    DM_msg_bfr[i] = (DM_msg_buffer_t){
        .hcan = (CAN_HandleTypeDef *)pmotor->pcom,
        .com_type = pmotor->com_type,
        .mode = pmotor->work_mode,
        .tx_msg = tx_msg,
        .id = tx_stdid_of(*pmotor),
        .refresh_freq = pmotor->INFO.refresh_freq,
        .had_refresh = true,
        .is_online = !IS_MOTOR_OFFLINE(*pmotor),
    };
    i++;
    if (i >= NUM_OF_DM_MOTOR)
        i = 0;
    return MOTORLIB_OK;
}

MOTORLIB_StatusTypeDef mount_DM_onto_bus(motor_t *pmotor, DM_motor_transmit_msg_t *mail, uint8_t ID)
{
    MOTORLIB_StatusTypeDef rslt = MOTORLIB_OK;
    // 校验是否已有数据填充
    uint8_t check_rst = 0;
    for (uint8_t i = 0; i < CAN_DATA_LEN; check_rst |= ((uint8_t *)mail)[i], i++)
        ;
    if (check_rst == 0) {
        // 数据未填充
        // 检测电机是否离线/未使能,若是则需要手动开启才能收到数据
        if (pmotor->AUX.EN == true) { // 期望使能电机
            /// 通过电机反馈状态(ERROR_FLAG)来确实模式
            /// 每次发送要么发使能帧,要么发送数据,这样最省带宽
            if ((IS_MOTOR_OFFLINE(*pmotor) == true) ||    // 电机离线
                (IS_MOTOR_OFFLINE(*pmotor) == false       // 电机未离线
                 && pmotor->AUX.STATE == MotorDisable)) { // 但电机反馈未使能
                // 电机离线/未真实使能,需要手动开启
                rslt |= enable_single_DM_motor_output((CAN_HandleTypeDef *)pmotor->pcom,
                                                      pmotor->type.tx_base_ID + ID,
                                                      pmotor->com_type);
            } else {
                // 存入缓冲区,用于规划动态发送频率
                if (pmotor->work_mode == MIT_TT || pmotor->work_mode == MIT_VDES ||
                    pmotor->work_mode == MIT_PDESVDES) {
                    DM_motor_transmit_MIT_msg_t convert_DM_MIT_tx_data(motor_t * motor); // 芝士声明
                    mail->MIT_PACK = convert_DM_MIT_tx_data(pmotor);
                } else if (pmotor->work_mode == PDESVDES) {
                    DM_motor_transmit_PDESVDES_msg_t convert_DM_PDESVDES_tx_data(motor_t * motor);
                    mail->PDESVDES_PACK = convert_DM_PDESVDES_tx_data(pmotor);
                } else if (pmotor->work_mode == VDES) {
                    DM_motor_transmit_VDES_msg_t convert_DM_VDES_tx_data(motor_t * motor);
                    mail->VDES_PACK = convert_DM_VDES_tx_data(pmotor);
                } else if (pmotor->work_mode == E_MIT) {
                    DM_motor_transmit_E_MIT_msg_t convert_DM_E_MIT_tx_data(motor_t * motor);
                    mail->E_MIT_PACK = convert_DM_E_MIT_tx_data(pmotor);
                } else {
                    // 指定工作模式不被支持
                    SetMotorErrorCode(*pmotor, MotorError_WorkModeConflict);
                    rslt |= MOTORLIB_ERROR;
                }
                rslt |= load_DM_motor_output(pmotor, mail);
            }
        } else { // 失能电机
            rslt |= disable_single_DM_motor_output((CAN_HandleTypeDef *)pmotor->pcom,
                                                   pmotor->type.tx_base_ID + ID, pmotor->com_type);
        }
    } else {
        // 发送控制量的ID重复
        SetMotorErrorCode(*pmotor, MotorError_TxIdDuplicate);
        rslt |= MOTORLIB_ERROR;
    }
    return rslt;
}

/**
 * @brief 将缓冲区的达妙电机控制数据全部发出
 * @return MOTORLIB_StatusTypeDef
 */
MOTORLIB_StatusTypeDef set_DM_motor_output(void)
{
#if 0
  // 自适应调整发送速率
  /**
   * 理想目标反馈频率应该为TIM7_IT_FREQ
   * 但实际因为电调内的代码在撞帧后不会自动重发
   * 经常会导致stdid低优先级的电机反馈频率下降
   * 所以需要设计一个控制器,使得每个电机的反馈频率都能近似相同
   * 控制器放大出的误差值将用于排名发送优先级
   * 然后只发优先级高的前n-1位电机数据
   */
  float total_rx_freq = 0;
  for (uint8_t i = 0; i < NUM_OF_DM_MOTOR; i++) {
    total_rx_freq += DM_msg_bfr[i].refresh_freq;
  }
  float target_rx_freq = total_rx_freq / NUM_OF_DM_MOTOR;
  for (uint8_t i = 0; i < NUM_OF_DM_MOTOR; i++) {
    DM_msg_bfr[i].weight =
        target_rx_freq - DM_msg_bfr[i].refresh_freq; // 散装kp控制器
  }

  // 冒泡排序一波,weight大的排前
  DM_msg_buffer_t tmp;
  for (uint8_t i = 0; i < NUM_OF_DM_MOTOR - 1; i++) {
    for (uint8_t j = 0; j < NUM_OF_DM_MOTOR - i - 1; j++) {
      if (DM_msg_bfr[j + 1].is_online == true &&
          // 电机未离线(离线电机的优先级调后点,在motor_ctrl里转去发使能帧了)
          DM_msg_bfr[j].weight < DM_msg_bfr[j + 1].weight) {
        memcpy(&tmp, &DM_msg_bfr[j], sizeof(DM_msg_buffer_t));
        memcpy(&DM_msg_bfr[j], &DM_msg_bfr[j + 1], sizeof(DM_msg_buffer_t));
        memcpy(&DM_msg_bfr[j + 1], &tmp, sizeof(DM_msg_buffer_t));
      }
    }
  }
  uint8_t SEND_NUM = NUM_OF_DM_MOTOR - 1;
#else
    uint8_t SEND_NUM = NUM_OF_DM_MOTOR;
#endif
    MOTORLIB_StatusTypeDef ret = MOTORLIB_OK;
    for (uint8_t i = 0; i < SEND_NUM; i++) {
        if (DM_msg_bfr[i].had_refresh == true) {
            if (DM_msg_bfr[i].mode == MIT_TT || DM_msg_bfr[i].mode == MIT_PDESVDES ||
                DM_msg_bfr[i].mode == MIT_VDES)
                ret |= set_DM_motor_output_MIT(DM_msg_bfr[i].hcan, &DM_msg_bfr[i].tx_msg->MIT_PACK,
                                               DM_msg_bfr[i].id, DM_msg_bfr[i].com_type);
            else if (DM_msg_bfr[i].mode == PDESVDES)
                ret |= set_DM_motor_output_PDESVDES(DM_msg_bfr[i].hcan,
                                                    &DM_msg_bfr[i].tx_msg->PDESVDES_PACK,
                                                    DM_msg_bfr[i].id, DM_msg_bfr[i].com_type);
            else if (DM_msg_bfr[i].mode == VDES)
                ret |= set_DM_motor_output_VDES(DM_msg_bfr[i].hcan,
                                                &DM_msg_bfr[i].tx_msg->VDES_PACK, DM_msg_bfr[i].id,
                                                DM_msg_bfr[i].com_type);
            else if (DM_msg_bfr[i].mode == E_MIT)
                ret |= set_DM_motor_output_E_MIT(DM_msg_bfr[i].hcan,
                                                 &DM_msg_bfr[i].tx_msg->E_MIT_PACK,
                                                 DM_msg_bfr[i].id, DM_msg_bfr[i].com_type);
            else
                ret |= MOTORLIB_ERROR;
        }
    }
    memset(&DM_msg_bfr[0], 0, sizeof(DM_msg_bfr)); // 清零,避免发送死包
    return ret;
}

/**
 * @brief 使能单个达妙电机输出
 * @param[in]  hcan   CAN句柄
 * @param[in]  id  电机CAN id
 * @param[in]  com_type 此包协议类型
 * @return MOTORLIB_OK
 */
MOTORLIB_StatusTypeDef enable_single_DM_motor_output(CAN_HandleTypeDef *hcan, uint32_t id,
                                                     motor_com_type_t com_type)
{
    HAL_StatusTypeDef HALrslt = HAL_OK;
    uint8_t enable_tx_buffer[CAN_DATA_LEN] = //
            { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC };
    bool is_fdcan = (com_type == COM_FDCAN) ? true : false;
    HALrslt |= can_transmit_data(hcan, id, enable_tx_buffer, CAN_DATA_LEN, is_fdcan);
    return HALrslt == HAL_OK ? MOTORLIB_OK : MOTORLIB_ERROR;
}

/**
 * @brief 使能所有达妙电机输出
 * @param[in]  hcan   CAN句柄
 * @param[in]  id  电机CAN id
 * @return MOTORLIB_OK
 */
MOTORLIB_StatusTypeDef enable_DM_motor_output(CAN_HandleTypeDef *hcan)
{
    MOTORLIB_StatusTypeDef ret = MOTORLIB_OK;
    motors_t *motor = get_motors_ptr();
    for (uint8_t i = 0; i < NUM_OF_ALL_MOTOR; i++) {
        if (IS_DM_MOTOR(motor->_[i]) == true) {
            ret |= enable_single_DM_motor_output(hcan, tx_stdid_of(motor->_[i]),
                                                 motor->_[i].com_type);
        }
    }
    return ret;
}

/**
 * @brief 失能单个达妙电机输出
 * @param[in]  hcan   CAN句柄
 * @param[in]  id  电机CAN id
 * @param[in]  com_type 此包协议类型
 * @return MOTORLIB_OK
 */
MOTORLIB_StatusTypeDef disable_single_DM_motor_output(CAN_HandleTypeDef *hcan, uint32_t id,
                                                      motor_com_type_t com_type)
{
    HAL_StatusTypeDef HALrslt = HAL_OK;
    uint8_t disable_tx_buffer[CAN_DATA_LEN] = //
            { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD };
    bool is_fdcan = (com_type == COM_FDCAN) ? true : false;
    HALrslt |= can_transmit_data(hcan, id, disable_tx_buffer, CAN_DATA_LEN, is_fdcan);
    return HALrslt == HAL_OK ? MOTORLIB_OK : MOTORLIB_ERROR;
}

/**
 * @brief 失能所有达妙电机输出
 * @param[in]  hcan   CAN句柄
 * @param[in]  id  电机CAN id
 * @return MOTORLIB_OK
 */
MOTORLIB_StatusTypeDef disable_DM_motor_output(CAN_HandleTypeDef *hcan)
{
    MOTORLIB_StatusTypeDef ret = MOTORLIB_OK;
    motors_t *motor = get_motors_ptr();
    for (uint8_t i = 0; i < NUM_OF_ALL_MOTOR; i++) {
        if (IS_DM_MOTOR(motor->_[i]) == true) {
            ret |= disable_single_DM_motor_output(hcan, tx_stdid_of(motor->_[i]),
                                                  motor->_[i].com_type);
        }
    }
    return ret;
}

/**
 * @brief 强制令单个DM电机的零点为当前位置
 * @param[in]  hcan   CAN句柄
 * @param[in]  id  电机CAN id
 * @param[in]  com_type 此包协议类型
 * @return MOTORLIB_OK
 */
MOTORLIB_StatusTypeDef set_single_DM_motor_pzero(CAN_HandleTypeDef *hcan, uint32_t id,
                                                 motor_com_type_t com_type)
{
    HAL_StatusTypeDef HALrslt = HAL_OK;
    uint8_t set_pzero_tx_buffer[CAN_DATA_LEN] = //
            { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE };
    bool is_fdcan = (com_type == COM_FDCAN) ? true : false;
    HALrslt |= can_transmit_data(hcan, id, set_pzero_tx_buffer, CAN_DATA_LEN, is_fdcan);
    return HALrslt == HAL_OK ? MOTORLIB_OK : MOTORLIB_ERROR;
}

/**
 * @brief 强制令所有DM电机的零点为当前位置
 * @param[in]  hcan   CAN句柄
 * @param[in]  id  电机CAN id
 * @return MOTORLIB_OK
 */
MOTORLIB_StatusTypeDef set_DM_motor_pzero(CAN_HandleTypeDef *hcan)
{
    MOTORLIB_StatusTypeDef ret = MOTORLIB_OK;
    motors_t *motor = get_motors_ptr();
    for (uint8_t i = 0; i < NUM_OF_ALL_MOTOR; i++) {
        if (IS_DM_MOTOR(motor->_[i]) == true) {
            ret |= set_single_DM_motor_pzero(hcan, tx_stdid_of(motor->_[i]), motor->_[i].com_type);
        }
    }
    return ret;
}

/**
 * @brief 清空单个DM电机错误
 * @param[in]  hcan   CAN句柄
 * @param[in]  id  电机CAN id
 * @param[in]  com_type 此包协议类型
 * @return MOTORLIB_OK if success otherwise MOTORLIB_ERROR
 */
MOTORLIB_StatusTypeDef clear_single_DM_motor_error(CAN_HandleTypeDef *hcan, uint32_t id,
                                                   motor_com_type_t com_type)
{
    HAL_StatusTypeDef HALrslt = HAL_OK;
    uint8_t clear_tx_buffer[CAN_DATA_LEN] = //
            { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB };
    bool is_fdcan = (com_type == COM_FDCAN) ? true : false;
    HALrslt |= can_transmit_data(hcan, id, clear_tx_buffer, CAN_DATA_LEN, is_fdcan);
    return HALrslt == HAL_OK ? MOTORLIB_OK : MOTORLIB_ERROR;
}

/**
 * @brief 清空所有DM电机的错误
 * @param[in]  hcan   CAN句柄
 * @param[in]  id  电机CAN id
 * @return MOTORLIB_OK
 */
MOTORLIB_StatusTypeDef clear_DM_motor_error(CAN_HandleTypeDef *hcan)
{
    MOTORLIB_StatusTypeDef ret = MOTORLIB_OK;
    motors_t *motor = get_motors_ptr();
    for (uint8_t i = 0; i < NUM_OF_ALL_MOTOR; i++) {
        if (IS_DM_MOTOR(motor->_[i]) == true) {
            ret |= clear_single_DM_motor_error(hcan, tx_stdid_of(motor->_[i]),
                                               motor->_[i].com_type);
        }
    }
    return ret;
}

/**
 * @brief 存储电机的指定可写寄存器数据
 * @attention 只支持驱动版本为V13之后的达妙电机
 * @param[in]  hcan   CAN句柄
 * @param[in]  id  电机CAN id
 * @param[in]  com_type 此包协议类型
 * @param[in]  RID  需要存储的电机内寄存器索引 (十进制)
 */
static MOTORLIB_StatusTypeDef storage_single_DM_motor_output(CAN_HandleTypeDef *hcan, uint16_t id,
                                                             uint8_t RID, motor_com_type_t com_type)
{
    HAL_StatusTypeDef HALrslt = HAL_OK;
    // 报文ID : 0x7FF, D0 : CANID_L, D1 : CANID_H, D2 : 0xAA, D3 : RID, D4 : 0x00,
    // D5 : 0x00, D6 : 0x00, D7 : 0x00
    uint8_t storage_tx_buffer[CAN_DATA_LEN] = //
            { (uint8_t)id, (uint8_t)id >> 8, 0xAA, RID, 0x00, 0x00, 0x00, 0x00 };
    bool is_fdcan = (com_type == COM_FDCAN) ? true : false;
    HALrslt |= can_transmit_data(hcan, 0x7FF, storage_tx_buffer, CAN_DATA_LEN, is_fdcan);
    return HALrslt == HAL_OK ? MOTORLIB_OK : MOTORLIB_ERROR;
}

/**
 * @brief 写入达妙DM系列电机的可写寄存器数据
 * @attention 只支持驱动版本为V13之后的达妙电机
 * @param[in]  hcan   CAN句柄
 * @param[in]  id  电机CAN id
 * @param[in]  com_type 此包协议类型
 * @param[in]  RID  允许修改的电机内寄存器索引 (十进制)
 * @param[in]  dat  写入的具体数据 (内容最大允许32位)
 */
static MOTORLIB_StatusTypeDef write_single_DM_motor_data(CAN_HandleTypeDef *hcan, uint16_t id,
                                                         uint8_t RID, uint8_t dat[4],
                                                         motor_com_type_t com_type)
{
    HAL_StatusTypeDef HALrslt = HAL_OK;
    // 报文ID : 0x7FF, D0 : CANID_L, D1 : CANID_H, D2 : 0x55, D3 : RID, D4 : dat1,
    // D5 : dat2, D6 : dat3, D7 : dat4
    uint8_t write_tx_buffer[CAN_DATA_LEN] = //
            { (uint8_t)id, (uint8_t)id >> 8, 0x55, RID, dat[0], dat[1], dat[2], dat[3] };
    bool is_fdcan = (com_type == COM_FDCAN) ? true : false;
    HALrslt |= can_transmit_data(hcan, 0x7FF, write_tx_buffer, CAN_DATA_LEN, is_fdcan);
    return HALrslt == HAL_OK ? MOTORLIB_OK : MOTORLIB_ERROR;
}

/**
 * @brief 读取达妙DM系列电机的可读寄存器数据
 * @attention 只支持驱动版本为V13之后的达妙电机
 * @param[in]  hcan   CAN句柄
 * @param[in]  id  电机CAN id
 * @param[in]  com_type 此包协议类型
 * @param[in]  RID  允许读取的电机内寄存器索引 (十进制)
 */
static MOTORLIB_StatusTypeDef read_single_DM_motor_data(CAN_HandleTypeDef *hcan, uint16_t id,
                                                        uint8_t RID, motor_com_type_t com_type)
{
    HAL_StatusTypeDef HALrslt = HAL_OK;
    // 报文ID : 0x7FF, D0 : CANID_L, D1 : CANID_H, D2 : 0x33, D3 : RID, D4 : 0x00,
    // D5 : 0x00, D6 : 0x00, D7 : 0x00
    uint8_t read_tx_buffer[CAN_DATA_LEN] = //
            { (uint8_t)id, (uint8_t)id >> 8, 0x33, RID, 0x00, 0x00, 0x00, 0x00 };
    bool is_fdcan = (com_type == COM_FDCAN) ? true : false;
    HALrslt |= can_transmit_data(hcan, 0x7FF, read_tx_buffer, CAN_DATA_LEN, is_fdcan);
    return HALrslt == HAL_OK ? MOTORLIB_OK : MOTORLIB_ERROR;
}

/******************************************************************************************/
// 有需要用到读、写、存储寄存器，需要自行添加函数，同时在parse中添加解析项
// read pm
MOTORLIB_StatusTypeDef read_single_DM_motor_pm(motor_t *pmotor, motor_com_type_t com_type)
{
    MOTORLIB_StatusTypeDef ret = MOTORLIB_OK;
    pmotor->AUX.p_m.read_flag = false;
    ret |= read_single_DM_motor_data(can_of(*pmotor), tx_stdid_of(*pmotor), DM_REG_p_m, com_type);
    return ret;
}
// read xout
MOTORLIB_StatusTypeDef read_single_DM_motor_xout(motor_t *pmotor, motor_com_type_t com_type)
{
    MOTORLIB_StatusTypeDef ret = MOTORLIB_OK;
    pmotor->AUX.xout.read_flag = false;
    ret |= read_single_DM_motor_data(can_of(*pmotor), tx_stdid_of(*pmotor), DM_REG_xout, com_type);
    return ret;
}
// read can_br
MOTORLIB_StatusTypeDef read_single_DM_motor_can_br(motor_t *pmotor, motor_com_type_t com_type)
{
    MOTORLIB_StatusTypeDef ret = MOTORLIB_OK;
    ret |= read_single_DM_motor_data(can_of(*pmotor), tx_stdid_of(*pmotor), DM_REG_can_br,
                                     com_type);
    return ret;
}

// write can_br
MOTORLIB_StatusTypeDef write_single_DM_motor_can_br(motor_t *pmotor, uint8_t can_br,
                                                    motor_com_type_t com_type)
{
    MOTORLIB_StatusTypeDef ret = MOTORLIB_OK;
    uint8_t dat[4] = { 0 };
    dat[0] = can_br;
    pmotor->AUX.can_br.write_flag = false;
    ret |= write_single_DM_motor_data(can_of(*pmotor), tx_stdid_of(*pmotor), DM_REG_can_br, dat,
                                      com_type);
    return ret;
}

// write vel_kp
MOTORLIB_StatusTypeDef write_single_DM_motor_kp_asr(motor_t *pmotor, float kp,
                                                    motor_com_type_t com_type)
{
    union _kp {
        uint8_t b[4];
        float f;
    } kp_u;
    kp_u.f = kp;
    MOTORLIB_StatusTypeDef ret = MOTORLIB_OK;
    uint8_t dat[4] = { 0 };
    for (uint8_t i = 0; i < 4; i++) {
        dat[i] = kp_u.b[i];
    }
    pmotor->AUX.kp_asr.write_flag = false;
    ret |= write_single_DM_motor_data(can_of(*pmotor), tx_stdid_of(*pmotor), DM_REG_KP_ASR, dat,
                                      com_type);
    return ret;
}

// write vel_ki
MOTORLIB_StatusTypeDef write_single_DM_motor_ki_asr(motor_t *pmotor, float ki,
                                                    motor_com_type_t com_type)
{
    union _ki {
        uint8_t b[4];
        float f;
    } ki_u;
    ki_u.f = ki;
    MOTORLIB_StatusTypeDef ret = MOTORLIB_OK;
    uint8_t dat[4] = { 0 };
    for (uint8_t i = 0; i < 4; i++) {
        dat[i] = ki_u.b[i];
    }
    pmotor->AUX.ki_asr.write_flag = false;
    ret |= write_single_DM_motor_data(can_of(*pmotor), tx_stdid_of(*pmotor), DM_REG_KI_ASR, dat,
                                      com_type);
    return ret;
}

// storage can_br
MOTORLIB_StatusTypeDef storage_single_DM_motor_can_br(motor_t *pmotor, motor_com_type_t com_type)
{
    MOTORLIB_StatusTypeDef ret = MOTORLIB_OK;
    pmotor->AUX.can_br.storage_flag = false;
    ret |= storage_single_DM_motor_output(can_of(*pmotor), tx_stdid_of(*pmotor), DM_REG_can_br,
                                          com_type);
    return ret;
}

// storage vel_kp
MOTORLIB_StatusTypeDef storage_single_DM_motor_kp_asr(motor_t *pmotor, motor_com_type_t com_type)
{
    MOTORLIB_StatusTypeDef ret = MOTORLIB_OK;
    pmotor->AUX.can_br.storage_flag = false;
    ret |= storage_single_DM_motor_output(can_of(*pmotor), tx_stdid_of(*pmotor), DM_REG_KP_ASR,
                                          com_type);
    return ret;
}

// storage vel_ki
MOTORLIB_StatusTypeDef storage_single_DM_motor_ki_asr(motor_t *pmotor, motor_com_type_t com_type)
{
    MOTORLIB_StatusTypeDef ret = MOTORLIB_OK;
    pmotor->AUX.can_br.storage_flag = false;
    ret |= storage_single_DM_motor_output(can_of(*pmotor), tx_stdid_of(*pmotor), DM_REG_KI_ASR,
                                          com_type);
    return ret;
}
/******************************************************************************************/

/**
 * @brief 解算达妙DM系列的电机
 * @param pmotor
 * @param rx_buffer
 */
DM_raw_motor_data_t dm_motor_temp;
void parse_DM_motor_data(motor_t *pmotor, uint8_t *rx_buffer)
{
    // 先判断是不是度参数的反馈帧
    if (rx_buffer[0] == (uint8_t)tx_stdid_of(*pmotor) &&
        rx_buffer[1] == (uint8_t)tx_stdid_of(*pmotor) >> 8) {
        // 返回数据最低位为D4 最高位为D7
        uint32_t _raw = (rx_buffer[7] << 24) | (rx_buffer[6] << 16) | (rx_buffer[5] << 8) |
                        rx_buffer[4];
        if (rx_buffer[2] == 0x33) {
            // 读反馈
            if (rx_buffer[3] == DM_REG_p_m)
                memcpy(&pmotor->AUX.p_m.dat, &_raw, sizeof(float)),
                        pmotor->AUX.p_m.read_flag = true;
            else if (rx_buffer[3] == DM_REG_xout)
                memcpy(&pmotor->AUX.xout.dat, &_raw, sizeof(float)),
                        pmotor->AUX.xout.read_flag = true;
            else if (rx_buffer[3] == DM_REG_can_br)
                memcpy(&pmotor->AUX.can_br.dat, &_raw, sizeof(uint8_t)),
                        pmotor->AUX.can_br.read_flag = true;
        } else if (rx_buffer[2] == 0x55) {
            if (rx_buffer[3] == DM_REG_can_br)
                memcpy(&pmotor->AUX.can_br.dat, &_raw, sizeof(float)),
                        pmotor->AUX.can_br.write_flag = true;
            // 写反馈
        } else if (rx_buffer[2] == 0xAA) {
            if (rx_buffer[3] == 0x01) {
                memcpy(&pmotor->AUX.p_m.dat, &_raw, sizeof(float)),
                        pmotor->AUX.p_m.storage_flag = true;
                memcpy(&pmotor->AUX.xout.dat, &_raw, sizeof(float)),
                        pmotor->AUX.xout.storage_flag = true;
                memcpy(&pmotor->AUX.can_br.dat, &_raw, sizeof(float)),
                        pmotor->AUX.can_br.storage_flag = true;
            }
            // 存反馈
        }
        return;
    }
    //  解算原始值
    // DM_raw_motor_data_t dm_motor_temp;
    dm_motor_temp.ID = (rx_buffer[0] & 0x0F);
    dm_motor_temp.ERROR_FLAG = (DM_ERROR_FLAG_t)(rx_buffer[0] >> 4);
    dm_motor_temp.raw_scale = rx_buffer[1] << 8 | rx_buffer[2];
    dm_motor_temp.raw_vel = (rx_buffer[3] << 4) | (rx_buffer[4] >> 4);
    dm_motor_temp.torque = ((rx_buffer[4] & 0xF) << 8 | rx_buffer[5]);
    dm_motor_temp.t_mos = rx_buffer[6];
    dm_motor_temp.t_rotor = rx_buffer[7];
    //  解算实际输出轴数据(过外置减速箱后)
    if (pmotor->type.model == motor_model_DM_DM8009) { // DM8009
        pmotor->real.omega =
                uint_to_float(dm_motor_temp.raw_vel, -DM8009_MIT_V_MAX, DM8009_MIT_V_MAX, 12) /
                RR_of(*pmotor);
        pmotor->real.rpm = pmotor->real.omega / 2.f / PI * 60.f;
        pmotor->real.torque =
                uint_to_float(dm_motor_temp.torque, -DM8009_MIT_T_MAX, DM8009_MIT_T_MAX, 12) *
                RR_of(*pmotor);
        pmotor->real.current = pmotor->real.torque * Kn_DM8009;
    } else if (pmotor->type.model == motor_model_DM_DM4310) { // DM4310
        pmotor->real.omega =
                uint_to_float(dm_motor_temp.raw_vel, -DM4310_MIT_V_MAX, DM4310_MIT_V_MAX, 12) /
                RR_of(*pmotor);
        pmotor->real.rpm = pmotor->real.omega / 2.f / PI * 60.f;
        pmotor->real.torque =
                uint_to_float(dm_motor_temp.torque, -DM4310_MIT_T_MAX, DM4310_MIT_T_MAX, 12) *
                RR_of(*pmotor);
        pmotor->real.current = pmotor->real.torque * Kn_DM4310;
    } else if (pmotor->type.model == motor_model_DM_DM4340) { // DM4340
        pmotor->real.omega =
                uint_to_float(dm_motor_temp.raw_vel, -DM4340_MIT_V_MAX, DM4340_MIT_V_MAX, 12) /
                RR_of(*pmotor);
        pmotor->real.rpm = pmotor->real.omega / 2.f / PI * 60.f;
        pmotor->real.torque =
                uint_to_float(dm_motor_temp.torque, -DM4340_MIT_T_MAX, DM4340_MIT_T_MAX, 12) *
                RR_of(*pmotor);
        pmotor->real.current = pmotor->real.torque * Kn_DM4340;
    } else if (pmotor->type.model == motor_model_DM_DM3519) { // DM3519
        // 注意 无论带不带减速箱 新电机默认反馈数据为
        // 经过原装减速箱后的绝对位置和减速后的速度以及减速后的扭矩
        // !!!!!! 日常使用裸3519 需要在上位机中修改3519减速比为1 !!!!!!
        pmotor->real.omega =
                uint_to_float(dm_motor_temp.raw_vel, -DM3519_MIT_V_MAX, DM3519_MIT_V_MAX, 12) /
                RR_of(*pmotor);
        pmotor->real.rpm = pmotor->real.omega / 2.f / PI * 60.f;
        pmotor->real.torque =
                uint_to_float(dm_motor_temp.torque, -DM3519_MIT_T_MAX, DM3519_MIT_T_MAX, 12) *
                RR_of(*pmotor);
        pmotor->real.current = pmotor->real.torque * Kn_DM3519;
    } else if (pmotor->type.model == motor_model_DM_DM6006) {
        pmotor->real.omega =
                uint_to_float(dm_motor_temp.raw_vel, -DM6006_MIT_V_MAX, DM6006_MIT_V_MAX, 12) /
                RR_of(*pmotor);
        pmotor->real.rpm = pmotor->real.omega / 2.f / PI * 60.f;
        pmotor->real.torque =
                uint_to_float(dm_motor_temp.torque, -DM6006_MIT_T_MAX, DM6006_MIT_T_MAX, 12) *
                RR_of(*pmotor);
    }

    if (dm_motor_temp.ERROR_FLAG == MosOverHeat || dm_motor_temp.ERROR_FLAG == RotorOverHeat ||
        dm_motor_temp.ERROR_FLAG == Overload)
        SetMotorErrorCode(*pmotor, MotorError_OverHeat);
    pmotor->real.tempture = dm_motor_temp.t_rotor;
    if (dm_motor_temp.ERROR_FLAG == CommunicationLoss)
        SetMotorErrorCode(*pmotor, MotorError_MotorOffline);
    pmotor->AUX.STATE = dm_motor_temp.ERROR_FLAG;

    // debug
    // if(pmotor->type.offset_ID == 1){
    //   if(pmotor->AUX.EN == true)
    //     pmotor->AUX.STATE = MotorEnable;
    //   else
    //     pmotor->AUX.STATE = MotorDisable;
    // }

    // 通过对相邻时刻的编码器值之差积分获得算上减速比的相对输出角度与绝对输出角度
    pmotor->real.raw_scale = dm_motor_temp.raw_scale;
    float diff_output_ang = m_get_minor_arc(dm_motor_temp.raw_scale, pmotor->real.last_raw_scale,
                                            span_of(*pmotor)) *
                            2 * PI / (span_of(*pmotor) * RR_of(*pmotor));

    // 防止第一次获取编码值时 diff_output_ang出现非期望的累计
    if (pmotor->real.last_raw_scale != pmotor->real.raw_scale &&
        pmotor->register_state == M_REGISTERED) {
        pmotor->register_state = M_SYNCHRONIZED;
        diff_output_ang = 0;
    }
    pmotor->real.last_raw_scale = pmotor->real.raw_scale;
    pmotor->real.abs_angle += diff_output_ang;
    pmotor->real.rel_angle += diff_output_ang;
    pmotor->real.rel_angle = m_range_map(pmotor->real.rel_angle, 0, 2 * PI);

    // pmotor->real.rel_angle =
    //     dm_motor_temp.raw_scale * 2 * PI / (span_of(*pmotor) *
    //     RR_of(*pmotor));
}
#endif
