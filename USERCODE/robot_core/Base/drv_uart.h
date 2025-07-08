#ifndef _UART_DRV_H
#define _UART_DRV_H

#include "drv_conf.h"
#include HAL_INCLUDE

#define UART_TIME_OUT 5

uint32_t uart_send(UART_HandleTypeDef* huart,uint8_t *p_data,uint16_t size);
HAL_StatusTypeDef uart_send_async(UART_HandleTypeDef* huart,uint8_t *p_data,uint16_t size);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

HAL_StatusTypeDef uart_recv_dma_init(UART_HandleTypeDef* huart,uint8_t* rx_buffer,uint16_t len);
//HAL_StatusTypeDef uart_ui_dma_init(UART_HandleTypeDef* huart,uint8_t* tx_buffer,uint16_t len);

#endif

