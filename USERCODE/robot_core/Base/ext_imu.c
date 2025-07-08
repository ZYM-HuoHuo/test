#include "ext_imu.h"
#include "Base/drv_uart.h"
#include "Base/drv_can.h"

#include "APP/imu_bmi088.h"

#include "crc.h"
#include "usart.h"
#include "dma.h"

#include <string.h>

imu_data_t ext_imu_data = {.raw_data = {.a_fullscale = ACCEL_RANGE_G * 9.7833f,.g_fullscale = GYRO_RANGE_DPS * PI / 180.0f}};
ext_imu_pack_t ext_imu_pack;
#if USE_EXT_IMU_UART
uint8_t ext_imu_rx_buffer[2*PACK_SIZE];
int8_t ext_imu_rx_size = 0;
uint8_t imu_ext_offset = 0;

extern UART_HandleTypeDef EXT_IMU_UART_HANDLE;
HAL_StatusTypeDef ext_imu_recv_dma_init(void)
{
	HAL_StatusTypeDef result = uart_recv_dma_init(&(EXT_IMU_UART_HANDLE), ext_imu_rx_buffer, 2 * PACK_SIZE);
	__HAL_UART_ENABLE_IT(&(EXT_IMU_UART_HANDLE), UART_IT_IDLE);
	return result;
}
void send_ext_imu_data_fp(UART_HandleTypeDef *huart, float *fp_data){
    uint8_t pack_len = sizeof(ext_imu_pack_t);
    uint8_t buffer[pack_len];
    buffer[0] = EXT_IMU_HEADER; // header
    memcpy(&buffer[1], fp_data, sizeof(ext_imu_data_fp_t));
    uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)buffer, (pack_len - 2)/4);
    buffer[pack_len - 2] = crc & 0xFF;
    buffer[pack_len - 1] = (crc >> 8) & 0xFF;
    uart_send_async(huart, buffer, pack_len);
}

uint8_t parse_ext_imu_data_fp(uint8_t *buffer){
    if(buffer == NULL) return 1;
    if(buffer[0] == 0XAA){
        uint16_t crc_received = (buffer[PACK_SIZE-1] | (buffer[PACK_SIZE-2] << 8));
        uint16_t crc_calculated = HAL_CRC_Calculate(&hcrc, (uint32_t *)buffer, (PACK_SIZE-2)/4);
        if(crc_received == crc_calculated){
            memcpy(&ext_imu_pack.data, &buffer[1], sizeof(ext_imu_data_fp_t));
        }
    }
    return 0;
}
void ext_uart_imu_idle_handle(UART_HandleTypeDef *huart){
    if (huart->Instance == EXT_IMU_UART &&
      __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    DMA_Stream_TypeDef *uhdma = huart->hdmarx->Instance;
    // 获取DMA接收到的一帧的长度
    ext_imu_rx_size = (int32_t)(2 * PACK_SIZE - uhdma->NDTR) - imu_ext_offset;
    // 验证数据长度
    if (ext_imu_rx_size == PACK_SIZE || ext_imu_rx_size == -PACK_SIZE) {
      if (!parse_ext_imu_data_fp(&ext_imu_rx_buffer[imu_ext_offset])) {
        // 数据正确
      } else {
        // 数据错误
      }
      // 设置帧偏移，实现双缓冲
      // 当偏移在中间时，下一次应该在起始，若在起始则下一次在中间
      imu_ext_offset = (int32_t)(imu_ext_offset == 0) * PACK_SIZE;
    } else if (ext_imu_rx_size != 0) {
      // 数据长度错误，需要重置DMA并且将偏移设回起始
      // some bytes lost, reset DMA buffer
      CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);
      __HAL_DMA_DISABLE(huart->hdmarx);
      imu_ext_offset = 0;
      uhdma->NDTR = (uint32_t)(PACK_SIZE * 2);
      __HAL_DMA_CLEAR_FLAG(huart->hdmarx,
                           __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmarx));
      __HAL_DMA_CLEAR_FLAG(huart->hdmarx,
                           __HAL_DMA_GET_HT_FLAG_INDEX(huart->hdmarx));
      __HAL_DMA_CLEAR_FLAG(huart->hdmarx,
                           __HAL_DMA_GET_TE_FLAG_INDEX(huart->hdmarx));
      __HAL_DMA_ENABLE(huart->hdmarx);
      SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    }
  }
}
#endif

#if USE_EXT_CAN_IMU
HAL_StatusTypeDef ext_imu_can_init(void){
  HAL_StatusTypeDef rslt = HAL_OK;
  #if defined(STM32F446xx) || defined(STM32F427xx) || defined(STM32F407xx)
	extern CAN_HandleTypeDef IMU_USE_CAN_HANDLE;
  rslt |= can_user_init(&IMU_USE_CAN_HANDLE, (uint32_t[]){EXT_IMU_CAN_ID1,EXT_IMU_CAN_ID2,0,0},1);
  #elif defined(STM32H723xx)
	extern FDCAN_HandleTypeDef IMU_USE_CAN_HANDLE;
  rslt |= fdcan_user_init(&IMU_USE_CAN_HANDLE, (uint32_t[]){EXT_IMU_CAN_ID1,EXT_IMU_CAN_ID2},1);
  #endif
  return rslt;
}
extern uint32_t send_delta_tick;
HAL_StatusTypeDef ext_imu_can_send(imu_data_t *imu){
  uint8_t buffer1[8];
  uint8_t buffer2[8];
  HAL_StatusTypeDef rslt = HAL_OK;
  float tx_pack[4] = {imu->fp_data.ax,imu->eular.roll,imu->fp_data.az,imu->eular.pitch};
  memcpy(buffer1, tx_pack, CAN_DATA_LEN);
  memcpy(buffer2, tx_pack+2, CAN_DATA_LEN);
  #if defined(STM32F446xx) || defined(STM32F427xx) || defined(STM32F407xx)
	extern CAN_HandleTypeDef IMU_USE_CAN_HANDLE;
  rslt |= can_transmit_data(&IMU_USE_CAN_HANDLE, EXT_IMU_CAN_ID1, buffer1, CAN_DATA_LEN);
  rslt |= can_transmit_data(&IMU_USE_CAN_HANDLE, EXT_IMU_CAN_ID2, buffer2, CAN_DATA_LEN);
  #elif defined(STM32H723xx)
	extern FDCAN_HandleTypeDef IMU_USE_CAN_HANDLE;
  rslt |= fdcan_transmit_data(&IMU_USE_CAN_HANDLE, EXT_IMU_CAN_ID1, buffer1, CAN_DATA_LEN);
  rslt |= fdcan_transmit_data(&IMU_USE_CAN_HANDLE, EXT_IMU_CAN_ID2, buffer2, CAN_DATA_LEN);
  #endif
  return rslt;
}
uint32_t ext_imu_data1_freq_cnt = 0;
uint32_t ext_imu_data2_freq_cnt = 0;
uint32_t sensor_delta_tick = 0;
void parse_ext_imu_can_data(uint16_t stdid, uint8_t *rx_buffer) {
  static bool recv_flag[2] = {0};
  imu_data_t *imu = &ext_imu_data;
  if(stdid == EXT_IMU_CAN_ID1){
    // 根据需求更改
    float buf[2];
    memcpy(buf, rx_buffer, CAN_DATA_LEN);
    imu->fp_data.ax = buf[0];
    imu->eular.roll = buf[1];
    ext_imu_data1_freq_cnt++;
    recv_flag[0] = true;
  }
  else if(stdid == EXT_IMU_CAN_ID2){
    // 根据需求更改
    float buf[2];
    memcpy(buf, rx_buffer, CAN_DATA_LEN);
    imu->fp_data.az = buf[0];
    imu->eular.pitch = buf[1];
    ext_imu_data2_freq_cnt++;
    recv_flag[1] = true; 
  }
  if(recv_flag[0] && recv_flag[1]){ // 表示两个包都接收完毕
    recv_flag[0] = recv_flag[1] = false;
    // cal_ext_imu();
  }
}
float ext_imu_data1_freq = 0;
float ext_imu_data2_freq = 0;
void update_ext_imu_can_status(void){
  float dt = 1.f/DETECT_TIM_FREQ;
  ext_imu_data1_freq = dt == 0? 0 : ext_imu_data1_freq_cnt / dt;
  ext_imu_data1_freq_cnt = 0;
  ext_imu_data2_freq = dt == 0? 0 : ext_imu_data2_freq_cnt / dt;
  ext_imu_data2_freq_cnt = 0;
}
#endif
static imu_acc_cal_t ext_acc_cali = {
	.accel_T = {{1.010860 ,0.015129 ,-0.001459},
 				{0.001142 ,1.009152 ,0.006399},
 				{-0.005477 ,0.002071 ,1.013539}},
  	.accel_offs = {-34.944336 ,-3.310059 ,107.792969}
};
 imu_gyro_cal_t ext_gyro_cali = {
  .gx_bias = -6.96746206f,
	.gy_bias = 10.4042711f,
	.gz_bias = -10.5023422f,
};
int8_t cal_ext_imu(void){
  extern imu_data_t imu_data;
	UNUSED(ext_acc_cali);
	UNUSED(ext_gyro_cali);
//  ext_imu_data.temperature = imu_data.temperature;
//  ext_imu_data.temperature = 27;
//  ext_imu_data.fp_data = imu_correct(&ext_imu_data.raw_data, &ext_acc_cali, &ext_gyro_cali, ext_imu_data.temperature);
//  imu_dcm_update(&ext_imu_data.fp_data, &ext_gyro_cali, sensor_delta_tick / (float)25600);
//	imu_dcm_get_eular(&ext_imu_data.eular);
	return 0;
}
