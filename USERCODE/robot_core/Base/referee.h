#ifndef _REFEREE_SYS_H
#define _REFEREE_SYS_H

#include "drv_conf.h"
#include HAL_INCLUDE

#if USE_REFEREE_UART == 1

#include "referee_conf.h"

#define REFEREE_RX_BUFFER_LEN 256U
#define REFEREE_TX_BUFFER_LEN 128U

#define REFEREE_TEST_CMDID 0x0201
#define REFEREE_SYS_MAX_LOST 5

void referee_buffer_clr(void);

uint16_t get_referee_rx_flags(uint16_t cmdid);
void clear_referee_rx_flags(uint16_t cmdid);

void inc_referee_rx_lost(void);
uint32_t is_referee_sys_offline(void);

HAL_StatusTypeDef referee_recv_dma_init(void);

void referee_uart_idle_handle(UART_HandleTypeDef *huart);
uint16_t send_referee_single(UART_HandleTypeDef* huart,uint16_t cmd_id,uint8_t *p_struct, uint16_t len);


extern ext_game_status_t		game_status;		//0x0001
extern ext_game_robot_status_t	game_robot_status;	//0x0201
extern ext_robot_hurt_t			robot_hurt;			//0x0206
extern ext_shoot_data_t			shoot_data;			//0x0207
extern ext_power_heat_data_t	power_heat_data;	//0x0202
extern ext_rfid_status_t		rfid_status;		//0x0209
extern ext_game_robot_pos_t		game_robot_pos;		//0x0203

#endif

#endif

