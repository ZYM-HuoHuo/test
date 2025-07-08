#ifndef _DRV_CAN_H
#define _DRV_CAN_H

#include "drv_conf.h"
#include HAL_INCLUDE

#if MOTOR_CAN_ENABLE == 1

HAL_StatusTypeDef can_user_init(CAN_HandleTypeDef *hcan, uint32_t FilterId[4],
                                bool FIFO);
HAL_StatusTypeDef can_transmit_data(CAN_HandleTypeDef *hcan, uint16_t stdid,
                                    uint8_t *tx_data, uint8_t size,
                                    bool is_fdcan);

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

HAL_StatusTypeDef old_can_user_init(CAN_HandleTypeDef* hcan);
#endif

#endif

