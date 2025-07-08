/**
 * @file drv_uart.c
 * @author Lizi
 * @brief 串口应用层
 * @version 0.1
 *
 * @copyright SCNU-PIONEER (c) 2022-2023
 *
 */
#include "drv_uart.h"
#include <string.h>
uint8_t a_send_queue[128], i_tx_write = 0, i_tx_read = 0;
extern uint16_t psc_cnt;
/**
  * @brief      enable uart receive uart it and initialize DMA
  * @param[in]  huart   pointer to the uart handle
  * @param[in]  rx_buffer   pointer to the receive buffer
  * @param[in]  len   buffer length
  * @return     set HAL_OK or HAL_BUSY
  */
	/*串口初�?�化DMA缓冲区函�?*/
HAL_StatusTypeDef uart_recv_dma_init(UART_HandleTypeDef* huart,uint8_t* rx_buffer,uint16_t len){
	HAL_UART_StateTypeDef state = huart->RxState;
	if (state == HAL_UART_STATE_READY){
		__HAL_LOCK(huart);
		huart->pRxBuffPtr = (uint8_t *)rx_buffer;
		huart->RxXferSize = len;
		huart->ErrorCode  = HAL_UART_ERROR_NONE;

		/* Enable the DMA Stream */
		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)rx_buffer, len);
		//HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);
		/* 
		 * Enable the DMA transfer for the receiver request by setting the DMAR bit
		 * in the UART CR3 register 
		 */
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
	
		__HAL_UART_CLEAR_OREFLAG(huart);
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		
		//__HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);
		
		__HAL_UNLOCK(huart);


		return HAL_OK;
	}else{
		return HAL_BUSY;
	}
}

//HAL_StatusTypeDef uart_ui_dma_init(UART_HandleTypeDef* huart,uint8_t* tx_buffer,uint16_t len){
//	while(HAL_DMA_GetState(huart->hdmatx) != HAL_DMA_STATE_READY);
//    //while(__HAL_DMA_GET_COUNTER(&hdma_usart1_tx));
//	
//    //�ر�DMA
//    __HAL_DMA_DISABLE(huart->hdmatx);

//    //��ʼ��������
//    HAL_UART_Transmit_DMA(huart, buffer, length);
//}

/**
  * @brief      send a couple of characters to uart
  * @param[in]  huart   pointer to the uart handle
  * @param[in]  p_data   pointer to the data buffer
  * @param[in]  size   data length to send
  * @return     the remaining size unsend, zero for success
  */
uint32_t uart_send(UART_HandleTypeDef* huart,uint8_t *p_data,uint16_t size){
	uint32_t start_tick = HAL_GetTick();
	while(size){
		WRITE_REG(huart->Instance->DR,*p_data);
		while(__HAL_UART_GET_FLAG(huart,UART_FLAG_TXE)!=SET){
			if(HAL_GetTick() - start_tick > UART_TIME_OUT) return size;
		}
		size--;
		p_data++;
	}
	return 0;
//	return HAL_UART_Transmit(huart,p_data,size,UART_TIME_OUT);
}

HAL_StatusTypeDef flag;
/**
  * @brief      send a couple of characters to uart (non-blocking)
  * @param[in]  huart   pointer to the uart handle
  * @param[in]  p_data   pointer to the data buffer
  * @param[in]  size   data length to send
  * @return      HAL status
  */
HAL_StatusTypeDef uart_send_async(UART_HandleTypeDef* huart,uint8_t *p_data,uint16_t size){
	// while(HAL_DMA_GetState(huart->hdmatx) != HAL_DMA_STATE_READY);
    // //while(__HAL_DMA_GET_COUNTER(&hdma_usart1_tx));
	
    // //�ر�DMA
    // __HAL_DMA_DISABLE(huart->hdmatx);

    // //��ʼ��������
    // return HAL_UART_Transmit_DMA(huart, p_data, size);
	return HAL_UART_Transmit(huart, p_data, size, 150);
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
		// ��F7ϵ���ǿ��Բ�д�ģ�F1����д
		// __HAL_DMA_CLEAR_FLAG(huart->hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmarx)); //���DMA2_Steam7������ɱ�־
		// HAL_UART_DMAStop(huart);		//��������Ժ�رմ���DMA,ȱ����һ�������
}

