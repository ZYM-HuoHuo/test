/**
 * @file motor_ctrl_LK.c
 *
 * @brief LK电机API

 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#ifndef HAL_INCLUDE
#include "main.h"
#else
#include HAL_INCLUDE
#endif // !HAL_INCLUDE
#include "./dev_LK.h"

#if NUM_OF_LK_QUAD_MOTOR
/**
 * @brief	一拖四瓴控电机型号初始化函数
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
void LK_QUAD_motor_register(motor_t *pmotor, LK_QUAD_motor_model_t motor_model,
                            motor_work_mode_t work_mode, motor_com_type_t com_type, uint32_t *pcan,
                            uint8_t ID, float RR, const M_LPF_t *init_lpf, motor_institution_t inst)
{
    memset(&pmotor->INFO.error_set, 0, sizeof(pmotor->INFO.error_set));
    // 电机类型初始化
    switch (motor_model) {
    case motor_model_LK_QUAD_MF9025:
        pmotor->type = LK_motor_lib_MF9025;
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

    pmotor->AUX.EN = false;
    pmotor->AUX.STATE = M_DISABLE;
    pmotor->AUX.QUAD = true;

    pmotor->type.reduction_ratio = RR;

    memcpy(&pmotor->lpf_fltr, init_lpf, sizeof(M_LPF_t));

    pmotor->register_state = M_REGISTERED;
}
/**
 * @brief	一拖四LK电机型号注销函数
 */
void LK_QUAD_motor_cancell(motor_t *pmotor)
{
    memset(pmotor, 0, sizeof(motor_t));
}

/**
 * @brief 解算瓴控MF系列的电机
 * @param pmotor
 * @param rx_buffer
 */
void parse_LK_QUAD_motor_data(motor_t *pmotor, uint8_t *rx_buffer)
{
    LK_raw_motor_data_t lk_motor_temp = { 0 };
    //   解算原始值
    memcpy((void *)&lk_motor_temp, rx_buffer, CAN_DATA_LEN);
    if (lk_motor_temp.CMD == 0xA1) // 转矩闭环反馈命令
    {
        //  解算实际输出轴数据(过外置减速箱后)
        pmotor->real.tempture = lk_motor_temp.t_rotor;
        if (pmotor->type.model == motor_model_LK_QUAD_MF9025) { // LK9025
            pmotor->real.current = lk_motor_temp.current * MF9025_CURR_MAX / MF9025_CURR_DATA_MAX;
            pmotor->real.torque = pmotor->real.current * Kn_MK9025;
        }
        pmotor->real.omega = m_deg2rad(lk_motor_temp.omega) / RR_of(*pmotor);
        pmotor->real.rpm = lk_motor_temp.omega / 6.f;
        // 通过对相邻时刻的编码器值之差积分获得算上减速比的相对输出角度与绝对输出角度
        pmotor->real.raw_scale = lk_motor_temp.raw_scale;
        float diff_output_ang = m_get_minor_arc(lk_motor_temp.raw_scale,
                                                pmotor->real.last_raw_scale, span_of(*pmotor)) *
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
    } else if (lk_motor_temp.CMD == 0x88 || lk_motor_temp.CMD == 0x9B) { // 电机运行反馈命令
        pmotor->AUX.STATE = M_ENABLE;
    } else if (lk_motor_temp.CMD == 0x81) { // 电机停止反馈命令
        pmotor->AUX.STATE = M_STOP;
    } else if (lk_motor_temp.CMD == 0x80) { // 电机失能反馈命令
        pmotor->AUX.STATE = M_DISABLE;
    } else
        return;
}
/**
 * @brief set up to four LK motors output(in boardcast mode)
 * @param[in]  hcan the CAN handler to transmit raw data
 * @param[in]  tx_msg the pointer to torque data struct (4
 * motor data per struct)
 * @param[in]  id   torque_ctrl_mode's range init in 0x280
 * @return MOTORLIB_OK if success otherwise MOTORLIB_ERROR
 */
MOTORLIB_StatusTypeDef set_OnePack_LK_motor_output(CAN_HandleTypeDef *hcan,
                                                   LK_QUAD_motor_transmit_msg_t *tx_msg,
                                                   uint32_t id, motor_com_type_t com_type)
{
    HAL_StatusTypeDef HALrslt = HAL_OK;
    uint8_t tx_buffer[CAN_DATA_LEN];
    memcpy(tx_buffer, (uint8_t *)(tx_msg->D), CAN_DATA_LEN);
    bool is_fdcan = (com_type == COM_FDCAN) ? true : false;
    HALrslt |= can_transmit_data(hcan, LK_BROADCAST_TORQUE_ID, tx_buffer, CAN_DATA_LEN, is_fdcan);
    return (HALrslt == HAL_OK) ? MOTORLIB_OK : MOTORLIB_ERROR;
}
MOTORLIB_StatusTypeDef shut_LK_QUAD_motor_output(CAN_HandleTypeDef *hcan, uint32_t id,
                                                 motor_com_type_t com_type)
{
    uint8_t tx_buffer[CAN_DATA_LEN] = { 0 };
    return set_OnePack_LK_motor_output(hcan, (LK_QUAD_motor_transmit_msg_t *)&tx_buffer, id,
                                       com_type);
}

/**
 * @brief 把LK电机的扭矩值转成控制编码值
 * @param motor 电机指针
 * @return int
 */
static int convert_LK_QUAD_tx_data(motor_t *motor)
{
    int tx_data = 0;
    if (motor->work_mode == QUAD_CURR) {
        tx_data = (int)(motor->T_ff / MF9025_TRQE_MAX * MF9025_TRQE_DATA_MAX);
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
    LK_QUAD_motor_transmit_msg_t *tx_msg;
    uint32_t id;
    float refresh_freq;
    bool had_refresh; // 避免某包因为转去发使能/失能帧,导致发送出数值为0的死包
    bool is_online;
} LK_QUAD_msg_buffer_t;

LK_QUAD_msg_buffer_t LK_QUAD_msg_bfr[NUM_OF_LK_QUAD_MOTOR] = { 0 };

volatile uint8_t LK_QUAD_msg_bfr_childcount = 0;
static MOTORLIB_StatusTypeDef load_LK_QUAD_motor_output(motor_t *pmotor,
                                                        LK_QUAD_motor_transmit_msg_t *tx_msg)
{
    MOTORLIB_StatusTypeDef rslt = MOTORLIB_OK;
    LK_QUAD_msg_bfr[LK_QUAD_msg_bfr_childcount] = (LK_QUAD_msg_buffer_t){
        .hcan = (CAN_HandleTypeDef *)pmotor->pcom,
        .com_type = pmotor->com_type,
        .mode = pmotor->work_mode,
        .tx_msg = tx_msg,
        .id = tx_stdid_of(*pmotor),
        .refresh_freq = pmotor->INFO.refresh_freq,
        .had_refresh = true,
        .is_online = !IS_MOTOR_OFFLINE(*pmotor),
    };
    if (LK_QUAD_msg_bfr[LK_QUAD_msg_bfr_childcount].com_type !=
        LK_QUAD_msg_bfr[((LK_QUAD_msg_bfr_childcount - 1) < 0) ? 0 :
                                                                 (LK_QUAD_msg_bfr_childcount - 1)]
                .com_type) {
        SetMotorErrorCode(*pmotor, MotorError_ComTypeError);
        rslt |= MOTORLIB_ERROR;
    }
    LK_QUAD_msg_bfr_childcount++;
    return rslt;
}
/**
 * @brief 将LK电机缓冲区的数据挂载到发送总线
 * @return MOTORLIB_StatusTypeDef
 */
MOTORLIB_StatusTypeDef mount_LK_QUAD_onto_bus(motor_t *pmotor, LK_QUAD_motor_transmit_msg_t *mail)
{
    MOTORLIB_StatusTypeDef rslt = MOTORLIB_OK;
    if (pmotor->AUX.EN == true) {
        if (pmotor->work_mode == QUAD_CURR || pmotor->work_mode == QUAD_VDES) {
            uint8_t mail_index = pmotor->type.offset_ID - 1;
            if (mail_index > 3)
                mail_index = mail_index - 3 - 1;
            if (mail->D[mail_index] == 0) {
                mail->D[mail_index] = convert_LK_QUAD_tx_data(pmotor); // 转换数据
                rslt |= load_LK_motor_output(pmotor, mail);
            } else {
                // 发送控制量的ID重复
                SetMotorErrorCode(*pmotor, MotorError_TxIdDuplicate);
                rslt |= MOTORLIB_ERROR;
            }
        } else {
            // 指定工作模式不被支持
            SetMotorErrorCode(*pmotor, MotorError_WorkModeConflict);
            rslt |= MOTORLIB_ERROR;
        }
    } else {
        // 期望失能则跳过本电机的数据装载
        return MOTORLIB_OK;
    }
    return rslt;
}
/**
 * @brief 将缓冲区的瓴控电机控制数据全部发出
 * @return MOTORLIB_StatusTypeDef
 */
MOTORLIB_StatusTypeDef set_LK_QUAD_motor_output(void)
{
    motors_t *motor = get_motors_ptr();
    MOTORLIB_StatusTypeDef ret = MOTORLIB_OK;
    for (uint8_t i = 0; i < LK_QUAD_msg_bfr_childcount; i++) {
        if (LK_QUAD_msg_bfr[i].had_refresh == true) {
            if (!LK_QUAD_msg_bfr[i].tx_msg->send_flag) {
                ret |= set_OnePack_LK_motor_output(LK_QUAD_msg_bfr[i].hcan,
                                                   LK_QUAD_msg_bfr[i].tx_msg, LK_QUAD_msg_bfr[i].id,
                                                   LK_QUAD_msg_bfr[i].com_type);
            }
            LK_QUAD_msg_bfr[i].tx_msg->send_flag = true; // 发送成功后，标记为已发送
        }
    }
    for (uint8_t i = 0; i < LK_QUAD_msg_bfr_childcount; i++) {
        LK_QUAD_msg_bfr[i].tx_msg->send_flag = false; // 重置发送标记
    }
    memset(&LK_QUAD_msg_bfr[0], 0, sizeof(LK_QUAD_msg_bfr)); // 清零,避免发送死包
    LK_QUAD_msg_bfr_childcount = 0;                          // 重置
    return ret;
}

// ID_CODE 形式为 0 0 0 0 每一位代表电机函数作用电机 因此ID_CODE最大为 1111 即
// 0xF
MOTORLIB_StatusTypeDef enable_LK_QUAD_motor_output(CAN_HandleTypeDef *hcan, uint32_t ID_CODE,
                                                   motor_com_type_t com_type)
{
    if (ID_CODE > 0XF)
        return MOTORLIB_ERROR;
    uint8_t target_motor[4] = { (ID_CODE & 0x1), (ID_CODE & 0x2) >> 1, (ID_CODE & 0x4) >> 2,
                                (ID_CODE & 0x8) >> 3 };
    HAL_StatusTypeDef HALrslt = HAL_OK;
    uint8_t clear_tx_buffer[CAN_DATA_LEN] = { 0 };  // 清除电机错误
    uint8_t enable_tx_buffer[CAN_DATA_LEN] = { 0 }; // 电机使能
    for (uint8_t i = 0; i < 4; i++) {
        if (target_motor[i]) {
            clear_tx_buffer[i * 2] = 0x9B;
            enable_tx_buffer[i * 2] = 0x88;
        }
    }
    bool is_fdcan = (com_type == COM_FDCAN) ? true : false;
    HALrslt |=
            can_transmit_data(hcan, LK_BROADCAST_MIX_ID, clear_tx_buffer, CAN_DATA_LEN, is_fdcan);
    HALrslt |=
            can_transmit_data(hcan, LK_BROADCAST_MIX_ID, enable_tx_buffer, CAN_DATA_LEN, is_fdcan);
    return (HALrslt == HAL_OK) ? MOTORLIB_OK : MOTORLIB_ERROR;
}
// 关闭电机
MOTORLIB_StatusTypeDef disable_LK_QUAD_motor_output(CAN_HandleTypeDef *hcan, uint32_t ID_CODE,
                                                    motor_com_type_t com_type)
{
    HAL_StatusTypeDef HALrslt = HAL_OK;
    if (ID_CODE > 0XF)
        return MOTORLIB_ERROR;
    uint8_t target_motor[4] = { (ID_CODE & 0x1), (ID_CODE & 0x2) >> 1, (ID_CODE & 0x4) >> 2,
                                (ID_CODE & 0x8) >> 3 };
    uint8_t disable_tx_buffer[CAN_DATA_LEN] = { 0 };
    for (uint8_t i = 0; i < 4; i++) {
        if (target_motor[i]) {
            disable_tx_buffer[i * 2] = 0x80;
        }
    }
    bool is_fdcan = (com_type == COM_FDCAN) ? true : false;
    HALrslt |=
            can_transmit_data(hcan, LK_BROADCAST_MIX_ID, disable_tx_buffer, CAN_DATA_LEN, is_fdcan);
    return (HALrslt == HAL_OK) ? MOTORLIB_OK : MOTORLIB_ERROR;
}
// 停止输出
MOTORLIB_StatusTypeDef stop_LK_QUAD_motor_output(CAN_HandleTypeDef *hcan, uint32_t ID_CODE,
                                                 motor_com_type_t com_type)
{
    HAL_StatusTypeDef HALrslt = HAL_OK;
    if (ID_CODE > 0XF)
        return MOTORLIB_ERROR;
    uint8_t target_motor[4] = { (ID_CODE & 0x1), (ID_CODE & 0x2) >> 1, (ID_CODE & 0x4) >> 2,
                                (ID_CODE & 0x8) >> 3 };
    uint8_t stop_tx_buffer[CAN_DATA_LEN] = { 0 };
    for (uint8_t i = 0; i < 4; i++) {
        if (target_motor[i]) {
            stop_tx_buffer[i * 2] = 0x81;
        }
    }
    bool is_fdcan = (com_type == COM_FDCAN) ? true : false;
    HALrslt |= can_transmit_data(hcan, LK_BROADCAST_MIX_ID, stop_tx_buffer, CAN_DATA_LEN, is_fdcan);
    return (HALrslt == HAL_OK) ? MOTORLIB_OK : MOTORLIB_ERROR;
}
#endif
