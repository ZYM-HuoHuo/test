#ifndef EXT_IMU_H_
#define EXT_IMU_H_

#include "Base/drv_uart.h"
#include "drv_conf.h"
#include "APP/behaviour.h"
typedef struct {
    float gx; float gy; float gz;
	float ax; float ay; float az;
}ext_imu_data_fp_t;

typedef struct {
    int16_t gx; int16_t gy; int16_t gz;
	int16_t ax; int16_t ay; int16_t az;
}ext_imu_data_raw_t;

typedef struct {
    uint32_t header;
    ext_imu_data_fp_t data;
    uint32_t crc;
}ext_imu_pack_t;

#if USE_EXT_IMU_UART
#define EXT_IMU_HEADER 0xAA
#define PACK_SIZE (4 + 4 * 6 + 4)
HAL_StatusTypeDef ext_imu_recv_dma_init(void);
void send_ext_imu_data_fp(UART_HandleTypeDef *huart, float *data);
void ext_uart_imu_idle_handle(UART_HandleTypeDef *huart);
#endif
#if USE_EXT_CAN_IMU
#define EXT_IMU_CAN_ID1 0x666
#define EXT_IMU_CAN_ID2 0x667
HAL_StatusTypeDef ext_imu_can_init(void);
HAL_StatusTypeDef ext_imu_can_send(imu_data_t *imu);
void parse_ext_imu_can_data(uint16_t stdid, uint8_t *rx_buffer);
void update_ext_imu_can_status(void);
int8_t cal_ext_imu(void);
#endif

#endif // !IMU_EXT_H_
