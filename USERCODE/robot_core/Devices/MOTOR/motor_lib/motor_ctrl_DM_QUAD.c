/**
 * @file motor_ctrl_DM_QUAD.c
 *
 * @brief 一拖四固件的达妙(dajiang)电机API
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#ifndef HAL_INCLUDE
#include "main.h"
#else
#include HAL_INCLUDE
#endif // !HAL_INCLUDE
#include "./dev_DM.h"

#if NUM_OF_DM_QUAD_MOTOR
/**
 * @brief	一拖四达妙电机型号注册函数
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
void DM_QUAD_motor_register(motor_t *pmotor, DM_motor_model_t motor_model,
                            motor_work_mode_t work_mode, motor_com_type_t com_type, uint32_t *pcan,
                            uint8_t ID, float RR, const M_LPF_t *init_lpf, motor_institution_t inst)
{
    memset(&pmotor->INFO.error_set, 0, sizeof(pmotor->INFO.error_set));
    switch (motor_model) {
    case motor_model_DM_QUAD_DM8009:
        if (ID > 4)
            pmotor->type = DM_motor_lib_DM8009_QUAD.H;
        else
            pmotor->type = DM_motor_lib_DM8009_QUAD.L;
        break;
    case motor_model_DM_QUAD_DM4310:
        if (ID > 4)
            pmotor->type = DM_motor_lib_DM4310_QUAD.H;
        else
            pmotor->type = DM_motor_lib_DM4310_QUAD.L;
        break;
    case motor_model_DM_QUAD_DM4340:
        if (ID > 4)
            pmotor->type = DM_motor_lib_DM4340_QUAD.H;
        else
            pmotor->type = DM_motor_lib_DM4340_QUAD.L;
        break;
    case motor_model_DM_QUAD_DM3519:
        if (ID > 4)
            pmotor->type = DM_motor_lib_DM3519_QUAD.H;
        else
            pmotor->type = DM_motor_lib_DM3519_QUAD.L;
        break;
    default:
        SetMotorErrorCode(*pmotor, MotorError_CanInitFail);
        return;
    }
    pmotor->pcom = pcan;
    pmotor->type.offset_ID = ID;
    pmotor->real.last_raw_scale = 0;
    pmotor->real.abs_angle = 0;

    pmotor->INFO.refresh_filter = (M_LPF_t){ .fc = 1.f, .ts = 1.f / MOTOR_DETECT_FREQ };

    pmotor->institution = inst;

    pmotor->work_mode = work_mode;
    pmotor->com_type = com_type;
    if (work_mode != QUAD_CURR /*&& work_mode != QUAD_VDES*/) {
        SetMotorErrorCode(*pmotor, MotorError_WorkModeConflict);
    }

    pmotor->AUX.EN = true; // 一拖四下的达妙是默认使能的
    pmotor->AUX.QUAD = true;

    pmotor->type.reduction_ratio = RR; // 电机减速比

    memcpy(&pmotor->lpf_fltr, init_lpf, sizeof(M_LPF_t));

    pmotor->register_state = M_REGISTERED;
}
/**
 * @brief	一拖四达妙电机型号注销函数
 */
void DM_QUAD_motor_cancell(motor_t *pmotor)
{
    memset(pmotor, 0, sizeof(motor_t));
}

MOTORLIB_StatusTypeDef set_OnePack_DM_QUAD_motor_output(CAN_HandleTypeDef *hcan,
                                                        DM_QUAD_motor_transmit_msg_t *tx_msg,
                                                        uint32_t id, motor_com_type_t com_type)
{
    HAL_StatusTypeDef HALrslt = HAL_OK;
    uint8_t tx_buffer[CAN_DATA_LEN];
    uint16_t *ptx_msg = (uint16_t *)(tx_msg->D);
    for (uint8_t i = 0; i < 4; i++) {
        tx_buffer[2 * i + 1] = (uint8_t)(ptx_msg[i] >> 8); // 控制值高8位
        tx_buffer[2 * i] = (uint8_t)(ptx_msg[i]);          // 控制值低8位
    }
    bool is_fdcan = (com_type == COM_FDCAN) ? true : false;
    HALrslt |= can_transmit_data(hcan, id, tx_buffer, CAN_DATA_LEN, is_fdcan);
    return HALrslt == HAL_OK ? MOTORLIB_OK : MOTORLIB_ERROR;
}
MOTORLIB_StatusTypeDef set_OnePack_DM_QUAD_motor_output_SP(CAN_HandleTypeDef *hcan,
                                                           DM_QUAD_motor_transmit_msg_t *tx_msg,
                                                           uint32_t id, motor_com_type_t com_type)
{
    HAL_StatusTypeDef HALrslt = HAL_OK;
    uint8_t tx_buffer[CAN_DATA_LEN];
    uint16_t *ptx_msg = (uint16_t *)(tx_msg->D);
    for (uint8_t i = 0; i < 4; i++) {
        tx_buffer[2 * i] = (uint8_t)(ptx_msg[i] >> 8); // 控制值低8位
        tx_buffer[2 * i + 1] = (uint8_t)(ptx_msg[i]);  // 控制值高8位
    }
    bool is_fdcan = (com_type == COM_FDCAN) ? true : false;
    HALrslt |= can_transmit_data(hcan, id, tx_buffer, CAN_DATA_LEN, is_fdcan);
    return HALrslt == HAL_OK ? MOTORLIB_OK : MOTORLIB_ERROR;
}

/**
 * @brief 把DM电机的扭矩值转成控制编码值
 * @note 扭矩 = 电流 x 线圈数 x 磁极数 x 磁极强度
 * @param motor 电机指针
 * @return int
 */
static int convert_DM_QUAD_tx_data(motor_t *motor)
{
    int tx_data = 0;
    if (motor->work_mode == QUAD_CURR) {
        if (motor->type.model == motor_model_DM_QUAD_DM8009) {
            tx_data = (int)(motor->T_ff / Kn_DM8009 / DM8009_CURR_MAX * DM8009_CURR_DATA_MAX);
        } else if (motor->type.model == motor_model_DM_QUAD_DM4310) {
            tx_data = (int)(motor->T_ff / Kn_DM4310 / DM4310_CURR_MAX * DM4310_CURR_DATA_MAX);
        } else if (motor->type.model == motor_model_DM_QUAD_DM3519) {
            tx_data = (int)(motor->T_ff / Kn_DM3519 / DM3519_CURR_MAX * DM3519_CURR_DATA_MAX);
        }
    } else {
        SetMotorErrorCode(*motor, MotorError_WorkModeConflict);
        tx_data = 0;
    }
    return tx_data;
}

typedef struct {
    CAN_HandleTypeDef *hcan;
    motor_com_type_t com_type;
    motor_work_mode_t mode;
    DM_QUAD_motor_transmit_msg_t *tx_msg; // D[4]
    uint32_t id;
    float refresh_freq;
    bool is_online;
} DM_QUAD_msg_buffer_t;

DM_QUAD_msg_buffer_t DM_QUAD_msg_bfr[NUM_OF_DM_QUAD_MOTOR] = { 0 };

volatile uint8_t DM_QUAD_msg_bfr_childcount = 0;
static MOTORLIB_StatusTypeDef load_DM_QUAD_motor_output(motor_t *pmotor,
                                                        DM_QUAD_motor_transmit_msg_t *tx_msg)
{
    MOTORLIB_StatusTypeDef rslt = MOTORLIB_OK;
    DM_QUAD_msg_bfr[DM_QUAD_msg_bfr_childcount] = (DM_QUAD_msg_buffer_t){
        .hcan = (CAN_HandleTypeDef *)pmotor->pcom,
        .com_type = pmotor->type.com_type,
        .mode = pmotor->work_mode,
        .tx_msg = tx_msg,
        .id = tx_stdid_of(*pmotor),
        .refresh_freq = pmotor->INFO.refresh_freq,
        .is_online = !IS_MOTOR_OFFLINE(*pmotor),
    };
    if (DM_QUAD_msg_bfr[DM_QUAD_msg_bfr_childcount].com_type !=
        DM_QUAD_msg_bfr[((DM_QUAD_msg_bfr_childcount - 1) < 0) ? 0 :
                                                                 (DM_QUAD_msg_bfr_childcount - 1)]
                .com_type) {
        SetMotorErrorCode(*pmotor, MotorError_ComTypeError);
        rslt = MOTORLIB_ERROR;
    }
    DM_QUAD_msg_bfr_childcount++;
    return rslt;
}

MOTORLIB_StatusTypeDef mount_DM_QUAD_onto_bus(motor_t *pmotor, DM_QUAD_motor_transmit_msg_t *mail);
{
    MOTORLIB_StatusTypeDef rslt = MOTORLIB_OK;
    if (pmotor->AUX.EN == true) {
        if (pmotor->work_mode == QUAD_CURR || pmotor->work_mode == QUAD_VDES) {
            uint8_t mail_index = pmotor->type.offset_ID - 1;
            if (mail_index > 3)
                mail_index = mail_index - 3 - 1;
            if (mail->D[mail_index] == 0) {
                int convert_DM_QUAD_tx_data(motor_t *);
                mail->D[mail_index] = convert_DM_QUAD_tx_data(pmotor);
                rslt |= load_DM_QUAD_motor_output(pmotor, mail);
            } else {
                // 发送控制量的ID重复
                SetMotorErrorCode(*pmotor, MotorError_TxIdDuplicate);
                rslt |= MOTORLIB_ERROR;
            }
        } else {
            SetMotorErrorCode(*pmotor, MotorError_WorkModeConflict);
            rslt |= MOTORLIB_ERROR;
            continue;
        }
    } else {
        // 期望失能则跳过本电机的数据装载
        return MOTORLIB_OK;
    }
    return rslt;
}

MOTORLIB_StatusTypeDef set_DM_QUAD_motor_output(void)
{
    motors_t *motor = get_motors_ptr();
    MOTORLIB_StatusTypeDef ret = MOTORLIB_OK;
    for (uint8_t i = 0; i < DM_QUAD_msg_bfr_childcount; i++) {
        if (!DM_QUAD_msg_bfr[i].tx_msg->send_flag) {
            if (DM_QUAD_msg_bfr[i].id != DM_motor_lib_DM3519_QUAD.L.tx_base_ID &&
                DM_QUAD_msg_bfr[i].id != DM_motor_lib_DM3519_QUAD.H.tx_base_ID) {
                ret |= set_OnePack_DM_QUAD_motor_output(DM_QUAD_msg_bfr[i].hcan,
                                                        DM_QUAD_msg_bfr[i].tx_msg,
                                                        DM_QUAD_msg_bfr[i].id,
                                                        DM_QUAD_msg_bfr[i].com_type);
            } else {
                ret |= set_OnePack_DM_QUAD_motor_output_SP(DM_QUAD_msg_bfr[i].hcan,
                                                           DM_QUAD_msg_bfr[i].tx_msg,
                                                           DM_QUAD_msg_bfr[i].id,
                                                           DM_QUAD_msg_bfr[i].com_type);
            }
        }
        DM_QUAD_msg_bfr[i].tx_msg->send_flag = true; // 发送成功后，标记为已发送
    }
    for (uint8_t i = 0; i < DM_QUAD_msg_bfr_childcount; i++) {
        DM_QUAD_msg_bfr[i].tx_msg->send_flag = false; // 重置发送标记
    }
    memset(&DM_QUAD_msg_bfr[0], 0, sizeof(DM_QUAD_msg_bfr)); // 清零,避免发送死包
    DM_QUAD_msg_bfr_childcount = 0;                          // 重置
    return ret;
}
MOTORLIB_StatusTypeDef shut_DM_QUAD_several_motor_output(CAN_HandleTypeDef *hcan, uint32_t id)
{
    uint8_t tx_buffer[CAN_DATA_LEN] = { 0 };
    return set_OnePack_DM_QUAD_motor_output(hcan, (DM_QUAD_motor_transmit_msg_t *)&tx_buffer, id);
}
void parse_DM_QUAD_motor_data(motor_t *pmotor, uint8_t *rx_buffer)
{
    //  解算原始值
    DM_QUAD_raw_motor_data_t motor_temp;
    motor_temp.raw_scale = ((rx_buffer[0] << 8) | rx_buffer[1]); // 当前位置值高8位|当前位置值低8位
    motor_temp.raw_rpm_x100 = (int16_t)(((rx_buffer[2] << 8) | rx_buffer[3]));
    motor_temp.current_mA = ((rx_buffer[4] << 8) | rx_buffer[5]);
    motor_temp.t_rotor = rx_buffer[6];
    motor_temp.t_pcb = rx_buffer[7];

    pmotor->real.tempture = motor_temp.t_rotor;
    //  解算实际输出轴数据(过外置减速箱后)
    if (pmotor->type.model == motor_model_DM_QUAD_DM8009) {
        pmotor->real.current = (float)motor_temp.current_mA / 1000.f;
        pmotor->real.torque = pmotor->real.current * Kn_DM8009;
        pmotor->real.omega = (float)motor_temp.raw_rpm_x100 * PI / 30.f / RR_of(*pmotor) / 100.f;
        pmotor->real.rpm = (float)motor_temp.raw_rpm_x100 / RR_of(*pmotor) / 100.f;
    } else if (pmotor->type.model == motor_model_DM_QUAD_DM4310) {
        pmotor->real.current = (float)motor_temp.current_mA / 1000.f;
        pmotor->real.torque = pmotor->real.current * Kn_DM4310;
        pmotor->real.omega = (float)motor_temp.raw_rpm_x100 * PI / 30.f / RR_of(*pmotor) / 100.f;
        pmotor->real.rpm = (float)motor_temp.raw_rpm_x100 / RR_of(*pmotor) / 100.f;
    } else if (pmotor->type.model == motor_model_DM_QUAD_DM4340) {
        pmotor->real.current = (float)motor_temp.current_mA / 1000.f;
        pmotor->real.torque = pmotor->real.current * Kn_DM4340;
        pmotor->real.omega = (float)motor_temp.raw_rpm_x100 * PI / 30.f / RR_of(*pmotor) / 100.f;
        pmotor->real.rpm = (float)motor_temp.raw_rpm_x100 / RR_of(*pmotor) / 100.f;
    } else if (pmotor->type.model == motor_model_DM_QUAD_DM3519) {
        pmotor->real.current = (float)motor_temp.current_mA * DM3519_CURR_MAX /
                               DM3519_CURR_DATA_MAX; // 一拖四3519反馈的就是A
        pmotor->real.torque = pmotor->real.current * Kn_DM3519;
        pmotor->real.omega = motor_temp.raw_rpm_x100 * PI / 30.f /
                             RR_of(*pmotor); // 一拖四3519反馈机械转子真实rpm
        pmotor->real.rpm = motor_temp.raw_rpm_x100 / RR_of(*pmotor);
    }

    // 通过对相邻时刻的编码器值之差积分获得算上减速比的相对输出角度与绝对输出角度
    pmotor->real.raw_scale = motor_temp.raw_scale;
    float diff_output_ang =
            m_get_minor_arc(motor_temp.raw_scale, pmotor->real.last_raw_scale, span_of(*pmotor)) *
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
}
#endif
