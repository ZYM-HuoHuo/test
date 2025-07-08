/**
 * @file vofa.c
 *
 * @brief 串口发送上位机(vofa)
 *
 *
 * @copyright SCNU-PIONEER (c) 2023-2024
 *
 */
#include "app/robot.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

#include "usbd_cdc_if.h"
#include "tools/buzzer.h"
#include "tools/ws2812.h"
#include "tools/vofa.h"

#include "algorithm/crc.h"
#include "algorithm/filter.h"
#include "algorithm/util.h"

#include "drv_conf.h"
#include "Base/drv_uart.h"
#include "Devices/MOTOR/motor_headers.h"

#include "APP/behaviour.h"
#include "APP/imu_bmi088.h"

#if USE_VOFA_UART
extern UART_HandleTypeDef VOFA_UART_HANDLE;
#define host_uart VOFA_UART_HANDLE
#define MAX_FLOATS_NUM 15 // 最大浮点数数量
#define buf_len ((MAX_FLOATS_NUM * (4 + 3 + 1) + 1) + 1)
// 发送精度最大小数点前四位+小数点后两位

char buffer[buf_len] = {0}; // 选择足够大的缓冲区
uint16_t tx_data_len = 0;
struct {
  float MAX_TX_FREQ;
  float MIN_TX_TICK;
  float REAL_TX_FREQ;
} VOFA_INFO = {0};
float dfloats[MAX_FLOATS_NUM];

void update_vofa(void) {

  static uint32_t tickstart = 0;
  if (host_uart.gState == HAL_UART_STATE_READY &&
      HAL_GetTick() - tickstart > VOFA_INFO.MIN_TX_TICK + 10) {
    VOFA_INFO.REAL_TX_FREQ = 1000.f / (HAL_GetTick() - tickstart);

/***传输数据始***/
#if ENABLE_MOTOR_POWER_PARAM_CALIBR_FLAG
    // 电机标定
    extern motor_power_parameter_calibrate_bag_t MPPCB;
    float Dfloats[MAX_FLOATS_NUM] = {
        MPPCB.real_power,      // 实际功率,单位W
        MPPCB.set_ctrl_value,  // 电机设定电流,单位与A呈线性相关
        MPPCB.real_ctrl_value, // 电机实际电流,单位与A呈线性相关
        MPPCB.real_omega, // 电机实际角速度,单位与rad/s呈线性相关
        MPPCB.fitted_power, // 功率模型拟合功率,单位W
    };
#else
    extern chassis_power_lim_t chassis_power_lim;
    cap_data_t *cap_data = get_cap_data();
    chassis_power_lim_t *p_lim = &chassis_power_lim;
    float real_power = cap_data->input_voltage * cap_data->input_current;
    float real_output_power =
        cap_data->input_voltage * cap_data->output_current;
    float raw_total_power = (p_lim->raw_power[0] + p_lim->raw_power[1]) / 2;
    extern float power_limit_ratio;
    float k = pow2f(VCAP_MAX) - pow2f(VCAP_MIN);
    float real_cap_energy_ratio = // 实际电容剩余能量百分比
        (pow2f(cap_data->cap_voltage) - pow2f(VCAP_MIN)) / k;
    extern float soft_HP_MAX;
    extern float soft_HP;
		extern float set_charge_power;
    float Dfloats[MAX_FLOATS_NUM] = {
        p_lim->power_limit,
        real_power,
        p_lim->real_power[0] + p_lim->real_power[1],
        p_lim->set_total_power,
        power_limit_ratio,
        p_lim->referee_power_buffer,
        p_lim->expt_referee_power_buffer,
        real_cap_energy_ratio,
        p_lim->expt_cap_energy_ratio,
        soft_HP_MAX,
        soft_HP,
        real_output_power,
				cap_data->input_current,
				cap_data->output_current,
				set_charge_power,
    };
#endif
    memcpy(dfloats, Dfloats, sizeof(dfloats));
    /***传输数据末***/

    char *buffer_ptr = buffer;

    tx_data_len = 0;
    for (char i = 0; i < MAX_FLOATS_NUM; i++) {
      tx_data_len +=
          snprintf(&buffer_ptr[tx_data_len], sizeof(buffer) - tx_data_len,
                   "%.3f,", Dfloats[i]);
    }
    // 添加 \r\n 并发送数据
    if (tx_data_len > 0) {
      buffer[tx_data_len - 2] = '\r';
      buffer[tx_data_len - 1] = '\n';
      // HAL_UART_Transmit_DMA(&host_uart, (uint8_t *)buffer, tx_data_len);
      HAL_UART_Transmit(&host_uart, (uint8_t *)buffer, tx_data_len, 0xffffffff);
    }

    // 固定起始位1位,数据位8位,无校验位,停止位1位,数据量tx_data_len字节
    VOFA_INFO.MIN_TX_TICK =
        1000.f * tx_data_len * (1 + 8 + 0 + 1) / host_uart.Init.BaudRate;
    VOFA_INFO.MAX_TX_FREQ = 1000.f / VOFA_INFO.MIN_TX_TICK;
    tickstart = HAL_GetTick();
  }
}
#endif
