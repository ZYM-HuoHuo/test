/**
 * @file drv_can.c
 *
*
 * @brief can应用层
 *
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#include "drv_can.h"

#if MOTOR_CAN_ENABLE == 1

#include "Devices/MOTOR/motor_headers.h"
#include "Base/ext_imu.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
/**
  * @brief  can fifo 0 rx callback, get motor feedback info
  * @param[in]  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @note this is a weak function in HAL
  * @return None
  */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_buffer[CAN_DATA_LEN];
	//判断读取FIFO0是否有效
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_buffer) == HAL_OK){
		#if USE_EXT_CAN_IMU == 1
      //处理外置IMU响应数据
			parse_motor_data(hcan,&rx_header,rx_buffer);
		#else
      //处理电机数据
			parse_motor_data(hcan,&rx_header,rx_buffer);
    #endif
    #if BOARDS_MODE
      board_interact_parse(rx_buffer,rx_header.StdId);
    #endif 
	}
}

/**
  * @brief  can fifo 1 rx callback, get motor feedback info
  * @param[in]  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @return None
  * @note this is a weak function in HAL
  */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_buffer[CAN_DATA_LEN];
  //判断读取FIFO1是否有效
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_buffer) == HAL_OK){
    #if USE_SUPER_CAP == 1
      //处理超级电容响应数据
      if(rx_header.StdId == CAP_RESPONSE_ID){
        parse_cap_data(&rx_header,rx_buffer);
      }else{
        parse_motor_data(hcan,&rx_header,rx_buffer);
      }
    #else
      parse_motor_data(hcan,&rx_header,rx_buffer);
    #endif
    #if USE_EXT_CAN_IMU 
      parse_ext_imu_can_data(rx_header.StdId, rx_buffer);
      #endif
    #if BOARDS_MODE
      board_interact_parse(rx_buffer,rx_header.StdId);
    #endif 
	}
}

/**
 * @brief  init can filter, start can, enable can rx interrupt
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @return HAL_OK if success otherwise HAL_ERROR
 */
HAL_StatusTypeDef CAN_INIT_FLAG = HAL_OK;
HAL_StatusTypeDef can_user_init(CAN_HandleTypeDef *hcan, uint32_t FilterId[4],
                                bool FIFO) {
  static uint8_t can1_init_FilterNum = 0;
  static uint8_t can2_init_FilterNum = 0;
  if ((hcan != &hcan1 && hcan != &hcan2) ||
      (hcan == &hcan1 && can1_init_FilterNum > 13) ||
      (hcan == &hcan2 && can2_init_FilterNum > 13))
    return HAL_ERROR;

  CAN_FilterTypeDef can_filter;
  can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
  can_filter.FilterScale = CAN_FILTERSCALE_16BIT;
  can_filter.FilterActivation = ENABLE; // enable can filter
	can_filter.SlaveStartFilterBank  = 14;
  // can的FilterBank参数是用来配置过滤器组的(标准库叫做FilterNumber)
  // 双CAN系列的产品有27个滤波器,其中can1的滤波器号为0~13,can2为14~27
  // 为了一路can配置8个id(list模式)(FIFO0与FIFO1各自4个)
  // 就不应该反复塞到0 or 14滤波器里(应该吧?)
  // 这里就采用了起始滤波器+FilterNum选定通道
  if (hcan->Instance == CAN1) {
    can_filter.FilterBank = 0 + can1_init_FilterNum;
    can1_init_FilterNum++;
  } else {
    can_filter.FilterBank = 14 + can2_init_FilterNum;
    can2_init_FilterNum++;
  }
	uint32_t real_filterId[4];
	for(uint8_t i = 0;i < 4; i++){
		if(FilterId[i] != 0)
			real_filterId[i] = FilterId[i]<<5;
		else
			real_filterId[i] = 0;
	}
//	for(uint8_t i = 0;i < 4; i++)
//		real_filterId[i] = FilterId[i]<<5;
  can_filter.FilterIdHigh = real_filterId[0];
  can_filter.FilterIdLow = real_filterId[1];
  can_filter.FilterMaskIdHigh = real_filterId[2];
  can_filter.FilterMaskIdLow = real_filterId[3];
  can_filter.FilterFIFOAssignment = FIFO;
  CAN_INIT_FLAG |= HAL_CAN_ConfigFilter(hcan, &can_filter);

  if (FIFO == CAN_FILTER_FIFO0)
//    CAN_INIT_FLAG |=
        HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  else
//    CAN_INIT_FLAG |=
        HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

  __HAL_CAN_ENABLE_IT(hcan, CAN_IT_BUSOFF);

  CAN_INIT_FLAG |= HAL_CAN_Start(hcan); // start can
  return CAN_INIT_FLAG;
}

HAL_StatusTypeDef can_transmit_data(CAN_HandleTypeDef *hcan, uint16_t stdid,
                                    uint8_t *tx_data, uint8_t size,
                                    bool is_fdcan) {
  UNUSED(is_fdcan);
  CAN_TxHeaderTypeDef tx_header;
  uint32_t can_mailbox;
  tx_header.DLC = size;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.StdId = stdid;
  tx_header.TransmitGlobalTime = DISABLE;
  return HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &can_mailbox);
}



#define MOTOR_ID_OFFSET 0x201U
HAL_StatusTypeDef old_can_user_init(CAN_HandleTypeDef* hcan){
  CAN_FilterTypeDef  can_filter;

  can_filter.FilterMode =  CAN_FILTERMODE_IDLIST;
  can_filter.FilterScale = CAN_FILTERSCALE_16BIT;
  can_filter.FilterIdHigh = (MOTOR_ID_OFFSET + 0) << 5;
  can_filter.FilterIdLow  = (MOTOR_ID_OFFSET + 1) << 5;
  can_filter.FilterMaskIdHigh = (MOTOR_ID_OFFSET + 2) << 5;
  can_filter.FilterMaskIdLow  = (MOTOR_ID_OFFSET + 3) << 5;
  can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0; 
  can_filter.FilterActivation = ENABLE;           
  can_filter.SlaveStartFilterBank  = 14;

  if(hcan->Instance == CAN2){
       can_filter.FilterBank = 14;
  }else{
       can_filter.FilterBank = 0;
  }


  if(HAL_CAN_ConfigFilter(hcan, &can_filter) == HAL_OK){ // init can filter
  	//no error HAL_CAN_ERROR_NOT_INITIALIZED
  	can_filter.FilterBank = can_filter.FilterBank + 1;
  	can_filter.FilterIdHigh = (MOTOR_ID_OFFSET + 4) << 5;//32位ID，高16位
  	can_filter.FilterIdLow  = (MOTOR_ID_OFFSET + 5) << 5;//低16位
  	can_filter.FilterMaskIdHigh = (MOTOR_ID_OFFSET + 6) << 5;
  	can_filter.FilterMaskIdLow  = (MOTOR_ID_OFFSET + 7) << 5;
  	can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  	HAL_CAN_ConfigFilter(hcan, &can_filter);
	  
	can_filter.FilterBank = can_filter.FilterBank + 1;
  	can_filter.FilterIdHigh = (MOTOR_ID_OFFSET + 8) << 5;//32位ID，高16位
  	can_filter.FilterIdLow  = (MOTOR_ID_OFFSET + 9) << 5;//低16位
  	can_filter.FilterMaskIdHigh = (MOTOR_ID_OFFSET + 10) << 5;
  	can_filter.FilterMaskIdLow  = 0;
  	can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  	HAL_CAN_ConfigFilter(hcan, &can_filter);

  	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // enable can rx interrupt
  	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
	__HAL_CAN_ENABLE_IT(hcan,CAN_IT_BUSOFF);
	
  	return HAL_CAN_Start(hcan);                          // start can
  }else{
  	return  HAL_ERROR;
  }
}

#endif

