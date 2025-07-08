#include "drv_ui.h"
#include HAL_INCLUDE
#include "Base/graphic_conf.h"

#include "Base/referee.h"
#include "Base/drv_uart.h"
#include "algorithm/crc.h"

#include <math.h>
#include <string.h>

#if USE_UI_OLD
extern UART_HandleTypeDef huart6;
#define UI_HUART huart6

static uint8_t ui_txbuf[128];

UI_figure_1_t fl;
UI_figure_char_t char_data;
UI_figure_7_t UI_7_dat1;
UI_figure_7_t UI_7_dat2;
UI_figure_7_t UI_7_dat3;


void Static_UI(uint8_t rank, UI_graphic_type_t type, UI_color_type_t color, 
uint16_t start_x, uint16_t start_y, uint16_t details_a, uint16_t details_b, uint16_t details_c, uint16_t details_d, uint16_t details_e, uint8_t width)
{
	UI_figure_1_t buffer;
	buffer.figures.figure_name[0] = 0x30;
	buffer.figures.figure_name[1] = 0x23;
	buffer.figures.figure_name[2] = 0x22 + rank;

	buffer.figures.operate_type = OP_TYPE_ADD;
	buffer.figures.figure_type = type;
	buffer.figures.layer = DYNAMIC_LAYER;
	buffer.figures.color = color;
	buffer.figures.details_a = details_a;
	buffer.figures.details_b = details_b;
	buffer.figures.width = width;

	buffer.figures.start_x = start_x;
	buffer.figures.start_y = start_y;

	buffer.figures.details_c = details_c;
	buffer.figures.details_d = details_d;
	buffer.figures.details_e = details_e;

	if (rank < 7)
		memcpy(&UI_7_dat1.figures._[rank], &buffer.figures, sizeof(UI_figure_1_t));
	else if (7 <= rank && rank < 14)
		memcpy(&UI_7_dat2.figures._[rank - 7], &buffer.figures, sizeof(UI_figure_1_t));
	else if (14 <= rank && rank < 21)
		memcpy(&UI_7_dat3.figures._[rank - 14], &buffer.figures, sizeof(UI_figure_1_t));
}
// 防刷
void show(uint8_t rank)
{
	uint8_t appear = 1;
	UI_figure_1_t buffer;

	if (rank < 7)
		memcpy(&buffer.figures, &UI_7_dat1.figures._[rank], sizeof(UI_figure_1_t));

	if (appear == 1)
		buffer.figures.operate_type = OP_TYPE_ADD;
	else if (!appear)
		buffer.figures.operate_type = OP_TYPE_DEL;
	else if (appear == 2)
		buffer.figures.operate_type = OP_TYPE_MOD;

	if (rank < 7)
		memcpy(&UI_7_dat1.figures._[rank], &buffer.figures, sizeof(UI_figure_1_t));
}
void modify(uint8_t rank, UI_graphic_op_type_t appear, UI_color_type_t color, 
uint16_t start_x, uint16_t start_y, uint16_t details_a, uint16_t details_b, uint16_t details_c, uint16_t details_d, uint16_t details_e, uint8_t width)
{ 
	UI_figure_1_t buffer;

	if (rank < 7)
		memcpy(&buffer.figures, &UI_7_dat1.figures._[rank], sizeof(UI_figure_1_t));

	if (appear == OP_TYPE_ADD)
		buffer.figures.operate_type = OP_TYPE_ADD;
	else if (!appear)
		buffer.figures.operate_type = OP_TYPE_DEL;
	else if (appear == OP_TYPE_DEL)
		buffer.figures.operate_type = OP_TYPE_MOD;

	if (appear == OP_TYPE_MOD) 
	{
		buffer.figures.color = color;
		buffer.figures.details_a = details_a;
		buffer.figures.details_b = details_b;
		buffer.figures.width = width;

		buffer.figures.start_x = start_x;
		buffer.figures.start_y = start_y;

		buffer.figures.details_c = details_c;
		buffer.figures.details_d = details_d;
		buffer.figures.details_e = details_e;
	}

	if (rank < 7)
		memcpy(&UI_7_dat1.figures._[rank], &buffer.figures, sizeof(UI_figure_1_t));
}

void send_7_old(UI_figure_7_t layer_data7)
{
	layer_data7.UI_header.data_cmd_id = 0x104;
	layer_data7.UI_header.sender_ID = game_robot_status.robot_id;
	layer_data7.UI_header.receiver_ID = game_robot_status.robot_id + 0x100;

	HAL_Delay(10);
	send_referee_single(&UI_HUART, 0x0301, (uint8_t *)&layer_data7, sizeof(layer_data7));
}

void send_5_old(UI_figure_5_t layer_data)
{
	layer_data.UI_header.data_cmd_id = 0x103;
	layer_data.UI_header.sender_ID = game_robot_status.robot_id;
	layer_data.UI_header.receiver_ID = game_robot_status.robot_id + 0x100;

	HAL_Delay(10);
	send_referee_single(&UI_HUART, 0x0301, (uint8_t *)&layer_data, sizeof(layer_data));
}

void char_data_init(void)
{
	char_data.UI_header.data_cmd_id = 0x110;
	char_data.UI_header.sender_ID = game_robot_status.robot_id;
	char_data.UI_header.receiver_ID = game_robot_status.robot_id + 0x100;

	char_data.char_figure.config.figure_name[0] = 0x20;
	char_data.char_figure.config.figure_name[1] = 0x22;
	char_data.char_figure.config.figure_name[2] = 0x20;

	char_data.char_figure.config.operate_type = OP_TYPE_ADD;
	char_data.char_figure.config.figure_type = (UI_graphic_type_t)CHAR; // 0-line 2- circle 7-char
	char_data.char_figure.config.layer = 3;
	char_data.char_figure.config.color = (UI_color_type_t)COLOR_GREEN; // 2/6-green,1-yellow,0-main,3-orange
	char_data.char_figure.config.details_a = 20;
	char_data.char_figure.config.details_b = 20;
	char_data.char_figure.config.width = 2;
	char_data.char_figure.config.details_c = 0;
	char_data.char_figure.config.details_d = 0;
	char_data.char_figure.config.details_e = 0; // 350 +300
}

// void send_char(uint16_t x,uint16_t y,uint8_t width,uint8_t size,uint8_t size_of,uint8_t str)
void send_char(UI_graphic_op_type_t appear, uint16_t x, uint16_t y, UI_color_type_t color, uint8_t size, uint8_t width, char *str)
{
	uint16_t total_len = 0;

	char_data.char_figure.config.color = color;

	if (appear)
		char_data.char_figure.config.operate_type = OP_TYPE_ADD;
	else
		char_data.char_figure.config.operate_type = OP_TYPE_DEL;

	char_data.char_figure.config.figure_name[2] = 0x21 + x / 15 + y / 13;
	char_data.char_figure.config.start_x = x;
	char_data.char_figure.config.start_y = y;
	char_data.char_figure.config.width = width;
	char_data.char_figure.config.details_a = size;
	char_data.char_figure.config.details_b = strlen(str);
	memset(char_data.char_figure.data, 0, strlen(str));
	memcpy(char_data.char_figure.data, str, strlen(str));
	total_len = fill_tx_buffer((uint8_t *)&char_data, sizeof(char_data));
	uart_send_async(&UI_HUART, ui_txbuf, total_len);
}

void float_data_init(void)
{
	uint16_t total_len = 0;
	fl.UI_header.data_cmd_id = 0x101; // �����ַ�
	fl.UI_header.sender_ID = game_robot_status.robot_id;
	fl.UI_header.receiver_ID = game_robot_status.robot_id + 0x100;
	fl.figures.details_d = 0;
	fl.figures.details_e = 0;
	fl.figures.operate_type = OP_TYPE_ADD;
	fl.figures.figure_type = 5; // 0-line 2- circle 7-char
	fl.figures.layer = DYNAMIC_LAYER;
	total_len = fill_tx_buffer((uint8_t *)&fl, sizeof(fl));
	uart_send_async(&UI_HUART, ui_txbuf, total_len);
}
void send_float(UI_graphic_op_type_t appear, uint16_t x, uint16_t y, UI_color_type_t color, uint8_t size, uint8_t width, float fl_num)
{

	uint16_t total_len = 0;
	int32_t buf;
	fl.figures.figure_name[0] = 0x13;
	fl.figures.figure_name[1] = 0x09 + x / 30;
	fl.figures.figure_name[2] = 0x12 + y / 40;

	if (appear == 0)
		fl.figures.operate_type = OP_TYPE_DEL;
	else if (appear == 1)
		fl.figures.operate_type = OP_TYPE_ADD;
	else if (appear == 2)
		fl.figures.operate_type = OP_TYPE_MOD;

	fl.figures.color = color; // 2/6-green,1-yellow,0-main,3-orange
	fl.figures.details_a = size;
	fl.figures.details_b = 5;
	fl.figures.width = width;

	fl.figures.start_x = x;
	fl.figures.start_y = y;

	// 2/6-green,1-yellow,0-main,3-orange
	buf = fl_num * 10.0f;
	fl.figures.details_c = buf;
	total_len = fill_tx_buffer((uint8_t *)&fl, sizeof(fl));
	uart_send_async(&UI_HUART, ui_txbuf, total_len);
}

void int_data_init(void)
{
	uint16_t total_len = 0;
	fl.UI_header.data_cmd_id = 0x101; 
	fl.UI_header.sender_ID = game_robot_status.robot_id;
	fl.UI_header.receiver_ID = game_robot_status.robot_id + 0x100;
	fl.figures.details_d = 0;
	fl.figures.details_e = 0;
	fl.figures.operate_type = OP_TYPE_ADD;
	fl.figures.figure_type = 6; // 0-line 2- circle 7-char
	fl.figures.layer = DYNAMIC_LAYER;
	total_len = fill_tx_buffer((uint8_t *)&fl, sizeof(fl));
	uart_send_async(&UI_HUART, ui_txbuf, total_len);
}

void send_int(UI_graphic_op_type_t appear, uint16_t x, uint16_t y, UI_color_type_t color, uint8_t size, uint8_t width, int32_t int_num)
{

	uint16_t total_len = 0;

	fl.figures.figure_name[0] = 0x13;
	fl.figures.figure_name[1] = 0x09 + x / 30;
	fl.figures.figure_name[2] = 0x12 + y / 40 + x / 30;

	if (appear == 0)
		fl.figures.operate_type = OP_TYPE_DEL;
	else if (appear == 1)
		fl.figures.operate_type = OP_TYPE_ADD;
	else if (appear == 2)
		fl.figures.operate_type = OP_TYPE_MOD;

	fl.figures.color = color; // 2/6-green,1-yellow,0-main,3-orange
	fl.figures.details_a = size;
	fl.figures.details_b = 0;
	fl.figures.width = width;

	fl.figures.start_x = x;
	fl.figures.start_y = y;

	// 2/6-green,1-yellow,0-main,3-orange
	fl.figures.details_c = int_num;
	total_len = fill_tx_buffer((uint8_t *)&fl, sizeof(fl));
	uart_send_async(&UI_HUART, ui_txbuf, total_len);
}
#endif
