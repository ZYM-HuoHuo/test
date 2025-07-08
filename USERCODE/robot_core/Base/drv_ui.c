/**
 * @file drv_ui.c
 * @author zwy
*
 * @brief UI中间层
 *
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#include "drv_ui.h"
#include HAL_INCLUDE
#include "Base/referee.h"
#include "Base/referee_conf.h"
#include "Base/drv_uart.h"
#include "Base/graphic_conf.h"
#include "algorithm/crc.h"

#include <math.h>
#include <string.h>

extern TIM_HandleTypeDef IMU_INIT_TIM_HANDLE;
extern RNG_HandleTypeDef hrng;
extern UART_HandleTypeDef REFEREE_UART_HANDLE;
#define UI_UART_HANDLE REFEREE_UART_HANDLE

uint8_t test_status = 0;
static uint8_t tx_buffer[128];
bool is_added[UI_ELEMENT_NUM] = {0};
bool char_modify_buf[CHAR_NUM] = {0}; // 缓存为空 1：缓存非空
uint8_t send_type = SEND_S_GRAPHIC;

UI_chars_t dynamic_chars;
UI_figure_7_t dynamic_layer_data1;
UI_figure_7_t dynamic_layer_data2;
UI_figure_7_t dynamic_layer_data3;

uint8_t send_static_graphic_flag = 0; // 0:不需要 1:data1 2:data2 3:data3
uint8_t send_static_char_flag = 0; // 0:不需要 n:chars index+1
UI_chars_t static_chars;
UI_figure_7_t static_layer_data1;
UI_figure_7_t static_layer_data2;
UI_figure_7_t static_layer_data3;

#define LOAD_DYNAMIC_GRAPHIC_DATA(_index_) \
{if (_index_ < 7)\
memcpy(&dynamic_layer_data1.figures._[_index_], &buffer, sizeof(interaction_figure_t));\
else if (7 <= _index_ && _index_ < 14)\
memcpy(&dynamic_layer_data2.figures._[_index_ - 7], &buffer, sizeof(interaction_figure_t));\
else if (14 <= _index_ && _index_ < 21)\
memcpy(&dynamic_layer_data3.figures._[_index_ - 14], &buffer, sizeof(interaction_figure_t));}
#define LOAD_STATIC_GRAPHIC_DATA(_index_) \
{if (_index_ < 28)\
memcpy(&static_layer_data1.figures._[_index_], &buffer, sizeof(interaction_figure_t));\
else if (28 <= _index_ && _index_ < 35)\
memcpy(&static_layer_data2.figures._[_index_ - 7], &buffer, sizeof(interaction_figure_t));\
else if (35 <= _index_ && _index_ < 42)\
memcpy(&static_layer_data3.figures._[_index_ - 14], &buffer, sizeof(interaction_figure_t));}

#define LOAD_DYNAMIC_CHAR_DATA(_index_)\
memcpy(&dynamic_chars._[_index_],&buffer,sizeof(UI_figure_char_t));
#define LOAD_STATIC_CHAR_DATA(_index_)\
memcpy(&static_chars._[_index_],&buffer,sizeof(UI_figure_char_t));

/**
 * @brief 实例化所有graphic 填充值入UI_figure_7_t中
 * @note 初始化时调用
 */
void UI_create(){
	// static uint8_t num = 0;
	// UI_figure_1_t temp_add;
	// interaction_figure_t buffer ={
	// 	.figure_name[0] = 0x30,
	// 	.figure_name[1] = 0x23,
	// 	.figure_name[2] = 0x22 + num,
	// 	.operate_type = OP_TYPE_ADD,
	// };
	// memcpy(&temp_add.figures, &buffer, sizeof(interaction_figure_t));
	// temp_add.UI_header.data_cmd_id = 0x104;
	// temp_add.UI_header.sender_ID = game_robot_status.robot_id;
	// temp_add.UI_header.receiver_ID = game_robot_status.robot_id + 0x100;
	// send_referee_single(&UI_UART_HANDLE, 0x0301, (uint8_t *)&temp_add, sizeof(UI_figure_7_t));
	// if(++num >= UI_ELEMENT_NUM)
	// enable_UI_update_flag = 1;
}
/**
 * @brief 删除graphic
 * @param rank graphic ID start on 0
 */
bool UI_delete(uint16_t *graphic_data){
	interaction_figure_t buffer;
	uint8_t rank = graphic_data[11];
	buffer.figure_name[0] = 0x30;
	buffer.figure_name[1] = 0x23;
	buffer.figure_name[2] = 0x22 + rank;
	buffer.operate_type = OP_TYPE_DEL;
	LOAD_DYNAMIC_GRAPHIC_DATA(rank);
	return true;
}
/**
 * @brief 编辑graphic
 * @param rank graphic ID start on 0
 * @param graphic_data 图形数据数组 要先转换为(uint16_t *)类型
 */
bool UI_modify_graphic(uint16_t *graphic_data){
	interaction_figure_t buffer;
	uint8_t rank = graphic_data[11];
	buffer.figure_name[0] = 0x30;
	buffer.figure_name[1] = 0x23;
	buffer.figure_name[2] = 0x22 + rank;
	if(is_added[rank] == true)
		buffer.operate_type = OP_TYPE_MOD;
	else
		buffer.operate_type = OP_TYPE_ADD;
	is_added[rank] = true;
	// 把graphic_data的数据赋值到buffer中
	buffer.figure_type = graphic_data[0];
	buffer.layer = graphic_data[1];
	buffer.color = graphic_data[2];
	buffer.details_a = graphic_data[3];
	buffer.details_b = graphic_data[4];
	buffer.width = graphic_data[5];
	buffer.start_x = graphic_data[6];
	buffer.start_y = graphic_data[7];
	buffer.details_c = graphic_data[8];
	buffer.details_d = graphic_data[9];
	buffer.details_e = graphic_data[10];
	uint8_t static_index = (rank>21) ? (rank-21):rank;
	if(buffer.layer == DYNAMIC_LAYER && rank < 21){
		LOAD_DYNAMIC_GRAPHIC_DATA(rank);
	}
	else if(buffer.layer == STATIC_LAYER && rank >= 21 && rank < 42 ){
	    LOAD_STATIC_GRAPHIC_DATA(rank);
		// 已改变的静态图超出7 需要大于1个周期抢占
		send_static_graphic_flag = (static_index<7)?1:((static_index<14)?2:3);
	}
	else return false;
	return true;
}
bool UI_modify_char(Char *char_data)
{
	UI_figure_char_t buffer;
	interaction_figure_t *char_config = &buffer.char_figure.config;
	uint8_t char_rank = char_data->config.char_rank;
	if(char_rank < 42)
		return false;
	uint8_t char_index = char_rank - GRAPHIC_NUM; // 0-20
	char_modify_buf[char_index] = true;

	char_config->figure_name[0] = 0x30;
	char_config->figure_name[1] = 0x23;
	char_config->figure_name[2] = 0x22 + char_rank;

	if(is_added[char_rank] == true)
		char_config->operate_type = OP_TYPE_MOD;
	else
		char_config->operate_type = OP_TYPE_ADD;
	is_added[char_rank] = true;

	char_config->figure_type = CHAR;
	char_config->layer = char_data->config.layer;
	char_config->color = char_data->config.color;
	char_config->details_a = char_data->config.size;
	char_config->details_b = strlen(char_data->text.str);
	memcpy(buffer.char_figure.data,char_data->text.str,sizeof(char_data->text.str));
	if(char_config->layer == DYNAMIC_GRAPHIC_NUM && char_rank < 52){
		LOAD_DYNAMIC_CHAR_DATA(char_index);
	}
	else if(char_config->layer == STATIC_GRAPHIC_NUM && char_rank >= 52 && char_rank < 62){
	    LOAD_STATIC_CHAR_DATA(char_index);
		send_static_char_flag = char_index+1;
	}
	else return false;
	return true;
}

// 函数只能显示7个动态元素 暂时还没修
// void show(uint8_t rank)
// {
// 	uint8_t appear = 1;
// 	UI_figure_1_t buffer;

// 	if (rank < 7)
// 		memcpy(&buffer.figures, &dynamic_layer_data1.figures._[rank], sizeof(interaction_figure_t));

// 	if (appear == 1)
// 		buffer.figures.operate_type = OP_TYPE_ADD;
// 	else if (!appear)
// 		buffer.figures.operate_type = OP_TYPE_DEL;
// 	else if (appear == 2)
// 		buffer.figures.operate_type = OP_TYPE_MOD;

// 	if (rank < 7)
// 		memcpy(&dynamic_layer_data1.figures._[rank], &buffer.figures, sizeof(interaction_figure_t));
// }

void UI_send()
{	
	UI_figure_7_t *UI_graphic7_data;
	UI_figure_char_t* UI_char_data;
	update_send_index();
	if(send_type != SEND_S_CHAR && send_type != SEND_CHAR){ // 发非字符
		if(send_type == 0){
			if(send_static_graphic_flag == 1){
				UI_graphic7_data = &static_layer_data1;
			}
			else if(send_static_graphic_flag == 2){
				UI_graphic7_data = &static_layer_data2;
			}
			else if(send_static_graphic_flag == 3){
				UI_graphic7_data = &static_layer_data3;
			}
			send_static_graphic_flag--;
		}
		else if(send_type == SEND_DATA1){
			UI_graphic7_data = &dynamic_layer_data1;
		}
		else if(send_type == SEND_DATA2){
			UI_graphic7_data = &dynamic_layer_data2;
		}
		else if(send_type == SEND_DATA3){
			UI_graphic7_data = &dynamic_layer_data3;
		}

		UI_graphic7_data->UI_header.data_cmd_id = 0x104;
		UI_graphic7_data->UI_header.sender_ID = game_robot_status.robot_id;
		UI_graphic7_data->UI_header.receiver_ID = game_robot_status.robot_id + 0x100;

		// uint32_t delay =0;
		// __HAL_TIM_SetCounter(&IMU_INIT_TIM_HANDLE,0);
		// __HAL_TIM_ENABLE(&IMU_INIT_TIM_HANDLE);
		// while((delay = __HAL_TIM_GetCounter(&IMU_INIT_TIM_HANDLE))<100 * 1000);
		// __HAL_TIM_DISABLE(&IMU_INIT_TIM_HANDLE);

		send_referee_single(&UI_UART_HANDLE, 0x0301, (uint8_t *)UI_graphic7_data, sizeof(UI_figure_7_t));
	}
	else // 发字符
	{
		static uint8_t send_char_collect_index[CHAR_NUM] = {0}; // 用于收集已修改的字符对象 为空时收集一次
		static uint8_t index = 0;
		static uint8_t collect_num = 0; // 表示本周期收集器有效内容数
		uint8_t enable_collect_flag = 1;
		uint8_t send_char_tmp_index[CHAR_NUM] = {0};
		uint8_t step = 0;
		for(uint8_t i = 0;i<CHAR_NUM;i++){
			if(char_modify_buf[i]){
				send_char_tmp_index[step] = char_modify_buf[i];
				step++;
			}
			if(send_char_collect_index[i])
			enable_collect_flag = 0; // 收集器里非空则不允许新的收集周期
		}
		if(enable_collect_flag){ // 更新周期
			memcpy(send_char_collect_index, send_char_tmp_index, CHAR_NUM);
			collect_num = step;
			index = 0;
		}
		
		if(send_char_collect_index[index]>10) // 判断静态还是动态
		UI_char_data = &static_chars._[send_char_collect_index[index]-10];
		else
		UI_char_data = &dynamic_chars._[send_char_collect_index[index]];
		if(index < collect_num)index++; // 未发完缓存
		else memset(send_char_collect_index,0,sizeof(send_char_collect_index));	// 已发完缓存

		UI_char_data->UI_header.data_cmd_id = 0x110;
		UI_char_data->UI_header.sender_ID = game_robot_status.robot_id;
		UI_char_data->UI_header.receiver_ID = game_robot_status.robot_id + 0x100;
		uint16_t total_len = 0;
		total_len = fill_tx_buffer((uint8_t *)UI_char_data, sizeof(UI_chars_t));
		
		// uint32_t delay =0;
		// __HAL_TIM_SetCounter(&IMU_INIT_TIM_HANDLE,0);
		// __HAL_TIM_ENABLE(&IMU_INIT_TIM_HANDLE);
		// while((delay = __HAL_TIM_GetCounter(&IMU_INIT_TIM_HANDLE))<100 * 1000);
		// __HAL_TIM_DISABLE(&IMU_INIT_TIM_HANDLE);

		uart_send_async(&UI_UART_HANDLE, tx_buffer, total_len);
	}

}

void send_5(UI_figure_5_t *UI_graphic5_data)
{
	UI_graphic5_data->UI_header.data_cmd_id = 0x103;
	UI_graphic5_data->UI_header.sender_ID = game_robot_status.robot_id;
	UI_graphic5_data->UI_header.receiver_ID = game_robot_status.robot_id + 0x100;

	send_referee_single(&UI_UART_HANDLE, 0x0301, (uint8_t *)UI_graphic5_data, sizeof(UI_figure_5_t));
}

uint16_t fill_tx_buffer(uint8_t *p_data, uint16_t len)
{
	memset(tx_buffer, 0, 128);

	uint16_t total_size;
	frame_header_t p_header;

	p_header.sof = 0xA5;
	p_header.data_len = len;
	p_header.seq = 0;
	p_header.crc8 = Get_CRC8_Check_Sum((uint8_t *)&p_header, sizeof(frame_header_t) - 1, 0xff);
	memcpy(&tx_buffer, &p_header, sizeof(frame_header_t));

	// uint16_t cmd_id = 0x0301;
	// memcpy(&tx_buffer[sizeof(frame_header_t)], (uint8_t*)&cmd_id, sizeof(uint16_t));
	*(uint16_t *)&tx_buffer[sizeof(frame_header_t)] = 0x0301;

	memcpy(&tx_buffer[sizeof(frame_header_t) + sizeof(uint16_t)], p_data, len);

	total_size = sizeof(frame_header_t) + sizeof(uint16_t) + len + sizeof(uint16_t);
	Append_CRC16_Check_Sum(tx_buffer, total_size);
	return total_size;
}
void update_send_index(void){
	static uint8_t cnt = 2;
	// 0:static graphic -- 1:static char//////2:char -- 3:data1 -- 4:data2 -- 5:data3 
	if(send_static_graphic_flag || send_static_char_flag) // 抢占式
		if(send_static_char_flag)send_type = SEND_S_CHAR;
		if(send_static_graphic_flag)send_type = SEND_S_GRAPHIC;
	else{
		send_type = cnt;
		cnt++;
		if(cnt > 5)cnt = 2;
		// send_type = 2 + hrng->Instance.DR%(5-2+1); // 2-5循环
	}
}
