#ifndef _DRV_UI_H
#define _DRV_UI_H

#include "drv_conf.h"
#include "stdint.h"
#include "stdbool.h"
#include "Base/graphic_conf.h"
#include "Base/referee_conf.h"

#define USE_UI_OLD 1

#define CHASSIS_STATE 2
#define GIMBAL_STATE 6
#define SHOOT_STATE 10
#define IMU_STATE 14
#define FRIC_STATE 18
#define NUC_STATE 22
#define SHELL_STATE 24

// 可设置UI元素数量
/**
 * @note 浮点数和整型属于graphic 字符为char 框架设计可以设置n个动态图形
 * @attention 静态图形框架还没写，后面写了可以发更多
 */
// 规定 0-20为动态图 21-41为静态图 42-51 为动态字符 52-61为静态字符

#define DYNAMIC_GRAPHIC_NUM 21
#define STATIC_GRAPHIC_NUM 21

#define DYNAMIC_CHAR_NUM 10
#define STATIC_CHAR_NUM 10

#define GRAPHIC_NUM DYNAMIC_GRAPHIC_NUM + STATIC_GRAPHIC_NUM
#define CHAR_NUM DYNAMIC_CHAR_NUM + STATIC_CHAR_NUM

#define UI_ELEMENT_NUM CHAR_NUM + GRAPHIC_NUM

// 屏幕尺寸
#define SCREEP_WIDTH  1920
#define SCREEP_HEIGHT 1080

// 准心坐标
#define BASIC_CENTER_X 960
#define BASIC_CENTER_Y 540

typedef __packed struct
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
} UI_header_t;

typedef __packed struct
{
	UI_header_t UI_header;
	interaction_layer_delete_t UI_delete_command;
} UI_delete_command_t;
typedef __packed struct
{
	UI_header_t UI_header;
	interaction_figure_t figures;
} UI_figure_1_t;
typedef __packed struct
{
	UI_header_t UI_header;
	interaction_figure_2_t figures;
} UI_figure_2_t;

typedef __packed struct
{
	UI_header_t UI_header;
	interaction_figure_3_t figures;
} UI_figure_5_t;

typedef __packed struct
{
	UI_header_t UI_header;
	interaction_figure_4_t figures;
} UI_figure_7_t;
typedef __packed struct
{
	// graphic_data_struct_t grapic_data_struct;
	UI_header_t UI_header;
	interaction_figure_char_t char_figure;
} UI_figure_char_t;

typedef struct{
    UI_figure_char_t _[CHAR_NUM];
}UI_chars_t;

typedef enum
{
	COLOR_MAIN_RB = 0,
	COLOR_YELLOW = 1,
	COLOR_GREEN = 2,
	COLOR_ORANGE = 3,
	COLOR_PURPLE = 4,
	COLOR_PINK = 5,
	COLOR_CYAN = 6,
	COLOR_BLACK = 7,
	COLOR_WHITE = 8
}UI_color_type_t;

typedef enum
{
	OP_TYPE_NOP = 0,
	OP_TYPE_ADD = 1,
	OP_TYPE_MOD = 2,
	OP_TYPE_DEL = 3
}UI_graphic_op_type_t;

typedef enum
{
	LAYER_OP_TYPE_NOP = 0,
	LAYER_OP_TYPE_DEL = 1,
	LAYER_OP_TYPE_DEL_ALL = 2
}UI_layer_op_type_t;

typedef enum
{
	STATIC_LAYER = 1,
	DYNAMIC_LAYER = 2,
}UI_layer_type_t;

typedef enum
{
	LINE = 0,
	SQUARE = 1,
	CIRCLE = 2,
	ELLIPSE = 3,
	ARC = 4,
	FL_NUM = 5,
	INT_NUM = 6,
	CHAR = 7
}UI_graphic_type_t;

#define	SEND_S_GRAPHIC 0
#define SEND_S_CHAR 1
#define SEND_CHAR 2
#define SEND_DATA1 3
#define SEND_DATA2 4
#define SEND_DATA3 5

uint16_t fill_tx_buffer(uint8_t *p_data, uint16_t len);

void UI_create(void);
bool UI_delete(uint16_t *graphic_data);
bool UI_modify_graphic(uint16_t *graphic_data);
bool UI_modify_char(Char *char_data);

void update_send_index(void);
void show(uint8_t rank); // shit 要铲掉的
void UI_send(void);
void send_5(UI_figure_5_t *UI_graphic5_data);

// 老版本UI
void send_7_old(UI_figure_7_t layer_data7);
void send_5_old(UI_figure_5_t layer_data);
void char_data_init(void);
void send_char(UI_graphic_op_type_t appear, uint16_t x, uint16_t y, UI_color_type_t color, uint8_t size, uint8_t width, char *str);
void float_data_init(void);
void send_float(UI_graphic_op_type_t appear, uint16_t x, uint16_t y, UI_color_type_t color, uint8_t size, uint8_t width, float fl_num);
void int_data_init(void);
void send_int(UI_graphic_op_type_t appear, uint16_t x, uint16_t y, UI_color_type_t color, uint8_t size, uint8_t width, int32_t int_num);
void show(uint8_t rank);
void modify(uint8_t rank, UI_graphic_op_type_t appear, UI_color_type_t color, 
uint16_t start_x, uint16_t start_y, uint16_t details_a, uint16_t details_b, uint16_t details_c, uint16_t details_d, uint16_t details_e, uint8_t width);
void Static_UI(uint8_t rank, UI_graphic_type_t type, UI_color_type_t color, 
uint16_t start_x, uint16_t start_y, uint16_t details_a, uint16_t details_b, uint16_t details_c, uint16_t details_d, uint16_t details_e, uint8_t width);


#endif
