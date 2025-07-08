/**
 * @file motor_ctrl_UT.c
 *
 * @brief UT电机API
 * @version 1.4
 * @author yjy
 * @date 2024.12.17
 * @note v1.1 删除了dma接收，可以正常多电机一收一发
 * @note v1.2 适配电机库，将接收结构体抽象到motor_t,本文件只需关注收发
 * @date 2025.1.14
 * @note v1.3 添加了dma，注意要设置dma的ram分区，并将解析和发送函数解耦
 * @date 2025.1.16
 * @note v1.4 1.添加了dma过半中断，解决了接收溢出的问题 2.添加了零点设置函数
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#include "crc.h"
#include "./dev_UT.h"
#include "../motor_ctrl.h"
#include <stdint.h>
#include <string.h>


#include HAL_INCLUDE

#if NUM_OF_UT_MOTOR

extern UART_HandleTypeDef UNITREE_UART_HANDLE;
extern DMA_HandleTypeDef UNITREE_DMA_HANDLE;

typedef struct {
    UT_motor_transmit_msg_t *tx_msg;
    uint32_t id;
    float refresh_freq; // 刷新频率
    bool had_refresh;   // 避免某包因为转去发使能/失能帧,导致发送出数值为0的死包
    bool is_online;     // 电机在线与否
                        //(不用refresh_freq来判断是为了保证离线判断的一致性)
} UT_msg_buffer_t;
UT_msg_buffer_t UT_msg_bfr[NUM_OF_UT_MOTOR] = { 0 };

#if defined(__GNUC__)
UT_motor_transmit_msg_t _txData __attribute__((section(".ram_DMA")));
UT_raw_motor_data_t _rxData __attribute__((section(".ram_DMA")));
#elif defined(__CC_ARM)
UT_motor_transmit_msg_t _txData;
UT_raw_motor_data_t _rxData;
#endif


/**
 * @description: 宇树电机dma初始化
 * @param uart
 * @return {*}
 */
void UT_motor_dma_init(UART_HandleTypeDef *uart)
{
    __HAL_UART_ENABLE_IT(&UNITREE_UART_HANDLE, UART_IT_IDLE);
    HAL_UARTEx_ReceiveToIdle_DMA(uart, (uint8_t *)&_rxData, sizeof(_rxData));
    __HAL_DMA_DISABLE_IT(&UNITREE_DMA_HANDLE, DMA_IT_HT);
}

/**
 * @description: 宇树电机初始化
 * @param pmotor
 * @param motor_model
 * @param id
 * @param uart
 * @param _id
 * @param init_lpf
 * @param inst
 * @param kd
 * @param kp
 * @param min_angle
 * @param max_angle
 * @return {*}
 */
void UT_motor_register(motor_t *pmotor, UT_motor_model_t motor_model, UART_HandleTypeDef *uart,
                       uint8_t _id, const M_LPF_t *init_lpf, motor_institution_t inst, float_t kd,
                       float_t kp, float min_angle, float max_angle)
{
    memset(&pmotor->INFO.error_set, 0, sizeof(pmotor->INFO.error_set));
    switch (motor_model) {
    case motor_model_UT_M8010_6:
        pmotor->type = UT_motor_lib_GO_M8010_6;
        pmotor->type.offset_ID = _id;
        break;
    default:
        SetMotorErrorCode(*pmotor, MotorError_CanInitFail);
        return;
    }

    pmotor->AUX.EN = false;
    pmotor->AUX.QUAD = false;
    pmotor->AUX.Kd = kd;
    pmotor->AUX.Kp = kp;
    pmotor->AUX.QUAD = false;
    pmotor->AUX.STATE = 0;

    pmotor->real.last_raw_scale = 0;
    pmotor->real.abs_angle = 0;

    pmotor->INFO.refresh_filter = (M_LPF_t){ .fc = 1.f, .ts = 1.f / MOTOR_DETECT_FREQ };
    memcpy(&pmotor->lpf_fltr, init_lpf, sizeof(M_LPF_t));

    pmotor->register_state = M_REGISTERED;
    pmotor->institution = inst;
    pmotor->work_mode = E_MIT;
    pmotor->com_type = COM_UART;
    pmotor->pcom = (uint32_t *)uart;
    pmotor->angle_limit_min = min_angle;
    pmotor->angle_limit_max = max_angle;
}

/** 
 * @description: 宇树电机设置零点函数
 * @param {RS485_MOTOR_send} *motor_s
 * @return {int}
 */
void set_UT_zero_angle(motor_t *p_motor)
{
    p_motor->real.rel_angle = 0;
    p_motor->real.abs_angle = 0;
}

/** 
 * @description: 宇树电机加载数据函数
 * @param {RS485_MOTOR_send} *motor_s
 * @return {int}
 */
MOTORLIB_StatusTypeDef load_UT_motor_output(motor_t *pmotor, UT_motor_transmit_msg_t *tx_msg)
{
    static uint8_t i = 0;
    UT_msg_bfr[i] = (UT_msg_buffer_t){
        .tx_msg = tx_msg,
        .id = tx_stdid_of(*pmotor),
        .refresh_freq = pmotor->INFO.refresh_freq,
        .had_refresh = true,
        .is_online = !IS_MOTOR_OFFLINE(*pmotor),
    };
    i++;
    if (i >= NUM_OF_UT_MOTOR)
        i = 0;
    return MOTORLIB_OK;
}

/** 
 * @description: 数据发送中间处理函数
 * @param {RS485_MOTOR_send} *motor_s
 * @return {int}
 */
UT_motor_transmit_msg_t *convert_UT_tx_data(motor_t *pmotor, UT_motor_transmit_msg_t *p_data)
{
    p_data->head[0] = 0xFE;
    p_data->head[1] = 0xEE;
    float v_des = pmotor->V_des * RR_of(*pmotor);
    float p_des = pmotor->P_des * RR_of(*pmotor);
    float t_ff = pmotor->T_ff * RR_of(*pmotor);
    SATURATE(pmotor->AUX.Kp, 0.0f, 25.599f);
    SATURATE(pmotor->AUX.Kd, 0.0f, 25.599f);
    SATURATE(t_ff, -127.99f, 127.99f);
    SATURATE(v_des, -804.00f, 804.00f);
    SATURATE(p_des, -411774.0f, 411774.0f);
    p_data->mode.status = pmotor->AUX.STATE;
    p_data->mode.id = pmotor->type.offset_ID;
    p_data->comd.k_pos = pmotor->AUX.Kp / 25.6f * 32768;
    p_data->comd.k_spd = pmotor->AUX.Kd / 25.6f * 32768;
    p_data->comd.pos_des = p_des / 6.2832f * 32768;
    p_data->comd.spd_des = v_des / 6.2832f * 256;
    p_data->comd.tor_des = t_ff * 256;

    p_data->CRC16 = Get_CRC16_Check_Sum((uint8_t *)p_data, 15, 0);

    return p_data;
}

MOTORLIB_StatusTypeDef mount_UT_onto_bus(motor_t *pmotor, UT_motor_transmit_msg_t *mail)
{
    MOTORLIB_StatusTypeDef return_flag = MOTORLIB_ERROR;
    if (pmotor->AUX.EN == 1) { // 期望使能电机
        convert_UT_tx_data(pmotor, mail);
        return_flag |= load_UT_motor_output(pmotor, mail);

    } else { // 失能电机
        return_flag |= disable_UT_motor_output(pmotor);
    }
    return return_flag;
}

/**
 * @description: 宇树电机发送数据深拷贝函数
 * @param {motor_t} *pmotor
 * @param {UT_motor_transmit_msg_t} tx_msg
 * @return {MOTORLIB_StatusTypeDef}
 * @TODO: 直接将数据放到bus而不是拷贝一次，但是要处理单独发送的数据
 */
void deepCopyUT_motor_transmit_msg(UT_motor_transmit_msg_t *tx_msg)
{
    _txData.head[0] = tx_msg->head[0];
    _txData.head[1] = tx_msg->head[1];
    _txData.mode = tx_msg->mode;
    _txData.comd = tx_msg->comd;
    _txData.CRC16 = tx_msg->CRC16;
}
/**
 * @description: 宇树电机发送函数
 * @param {motor_t} *pmotor
 * @param {UT_motor_transmit_msg_t} tx_msg
 * @return {MOTORLIB_StatusTypeDef}
 */
MOTORLIB_StatusTypeDef set_single_UT_motor_output(motor_t *pmotor, UT_motor_transmit_msg_t *tx_msg)
{
    SET_485_DE_UP();
    deepCopyUT_motor_transmit_msg(tx_msg);
    MOTORLIB_StatusTypeDef ret = (MOTORLIB_StatusTypeDef)HAL_UART_Transmit_DMA(
            (UART_HandleTypeDef *)pmotor->pcom, (uint8_t *)&_txData,
            sizeof(UT_motor_transmit_msg_t));
    SET_485_DE_DOWN();
    return ret;
}

/** 
 * @description: 数据接收中间处理函数
 * @param {RS485_MOTOR_send} *motor_s
 * @return {MOTORLIB_StatusTypeDef}
 */
void extract_data(UT_raw_motor_data_t *motor_r)
{
    motor_t *pmotor = NULL;
    motors_t *p_motors = get_motors_ptr();

    for (uint8_t i = 0; i < NUM_OF_ALL_MOTOR; i++) {
        if (p_motors->_[i].type.offset_ID == motor_r->mode.id && IS_UT_MOTOR(p_motors->_[i])) {
            pmotor = &p_motors->_[i];
            break;
        } else if (i == NUM_OF_ALL_MOTOR - 1) {
            return;
        }
    }
    if (pmotor->type.model == motor_model_UT_M8010_6) {
        if (motor_r->CRC16 != Get_CRC16_Check_Sum((uint8_t *)(motor_r), 14, 0)) {
            SetMotorErrorCode(*pmotor, MotorError_Rxdataerror);
            return;
        } else {
            pmotor->INFO.refresh_cnt++; // 统计数据反馈频率
            pmotor->AUX.STATE = motor_r->mode.status;
            pmotor->INFO.EC_cnt = motor_r->fbk.MError;
            pmotor->real.last_raw_scale = pmotor->real.raw_scale;
            pmotor->real.raw_scale = motor_r->fbk.pos;
            pmotor->real.abs_angle = 6.2832f * ((float)motor_r->fbk.pos) / 32768 / RR_of(*pmotor);
            pmotor->real.rel_angle = m_range_map(pmotor->real.abs_angle, 0, 2 * PI);
            pmotor->real.omega = ((float)motor_r->fbk.speed / 256) * 6.2832f;
            pmotor->real.rpm = m_radps2rpm(pmotor->real.omega);
            pmotor->real.torque = ((float)motor_r->fbk.torque) / 256;
            pmotor->real.current = pmotor->real.torque;
            pmotor->real.tempture = motor_r->fbk.temp;
            pmotor->output = pmotor->real.torque / pmotor->real.current;
        }
    }
}

/**
 * @description: 宇树电机接受解析函数,放在uart_callback中
 * @param {UART_HandleTypeDef} *uart
 * @return {MOTORLIB_StatusTypeDef}
 */
MOTORLIB_StatusTypeDef parse_UT_motor_data(UART_HandleTypeDef *uart)
{
    uint16_t rxlen = 0;
    MOTORLIB_StatusTypeDef ret = MOTORLIB_ERROR;
    extract_data(&_rxData);
    ret = (MOTORLIB_StatusTypeDef)HAL_UARTEx_ReceiveToIdle_DMA(uart, (uint8_t *)&_rxData,
                                                               sizeof(_rxData));
    __HAL_DMA_DISABLE_IT(&UNITREE_DMA_HANDLE, DMA_IT_HT);
    return ret;
}

/**
 * @description: 所有宇树发送电机函数
 * @param {RS485_MOTOR_send} *motor_s
 * @return {*}
 */
MOTORLIB_StatusTypeDef set_UT_motor_output()
{
    uint8_t SEND_NUM = NUM_OF_UT_MOTOR;
    MOTORLIB_StatusTypeDef ret = MOTORLIB_ERROR;

    motors_t *p_motors = get_motors_ptr();
    for (uint8_t i = 0; i < SEND_NUM; i++) {
        uint8_t id = UT_msg_bfr[i].id;
        if (UT_msg_bfr[i].had_refresh == true) {
            ret = set_single_UT_motor_output(&p_motors->_[id - 1], UT_msg_bfr[i].tx_msg);
        }
    }
    memset(&UT_msg_bfr[0], 0, sizeof(UT_msg_bfr)); // 清零,避免发送死包
    return ret;
}

/**
 * @description: 宇树电机使能函数
 * @param {RS485_MOTOR_send} *motor_s
 * @return {*}
 */
MOTORLIB_StatusTypeDef enable_UT_motor_output(motor_t *pmotor)
{
    pmotor->AUX.EN = true;
    pmotor->AUX.STATE = 1;

    UT_motor_transmit_msg_t p_data;
    memset(&p_data, 0, sizeof(UT_motor_transmit_msg_t));
    convert_UT_tx_data(pmotor, &p_data);
    MOTORLIB_StatusTypeDef ret = set_single_UT_motor_output(pmotor, &p_data);
    return ret;
}

/**
 * @description: 宇树电机失能函数
 * @param {RS485_MOTOR_send} *motor_s
 * @return {*}
 */
MOTORLIB_StatusTypeDef disable_UT_motor_output(motor_t *pmotor)
{
    pmotor->AUX.EN = false;
    pmotor->AUX.STATE = 0;

    UT_motor_transmit_msg_t p_data;
    memset(&p_data, 0, sizeof(UT_motor_transmit_msg_t));
    convert_UT_tx_data(pmotor, &p_data);
    MOTORLIB_StatusTypeDef ret = set_single_UT_motor_output(pmotor, &p_data);
    return ret;
}
/** 
 * @description: 所有宇树电机关停函数
 * @param {RS485_MOTOR_send} *motor_s
 * @return {*}
 */
MOTORLIB_StatusTypeDef shut_all_UT_motor_output()
{
    MOTORLIB_StatusTypeDef ret = MOTORLIB_ERROR;
    motors_t *p_motors = get_motors_ptr();
    for (int8_t i = 0; i < NUM_OF_UT_MOTOR; i++) {
        uint8_t id = UT_msg_bfr[i].id;
        ret = disable_UT_motor_output(&p_motors->_[id - 1]);
    }
    return ret;
}

#endif
