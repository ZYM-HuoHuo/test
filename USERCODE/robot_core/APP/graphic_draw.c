/**
 * @file graphic_draw.c
*
 * @brief UI应用层
 *
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#include "stm32f4xx_hal.h"
#include "graphic_draw.h"

#include "Base/referee.h"
#include "Base/drv_uart.h"
#include "Base/drv_ui.h"
#include "Base/graphic_conf.h"
#include "algorithm/crc.h"

#include "algorithm/imu_fusion.h"
#include "algorithm/util.h"

#include <math.h>
#include <string.h>
#include "behaviour.h"
#include "vision.h"

#define WIDTH 30
#define BAR 16

int cap_len_ui = 630;
int HP_len_ui = 284;

uint16_t UI_count;

#if CLEAN_FLAG == 1
extern float max_buffer_c, max_chassis_power;
extern power_parameter_t power_parameter;
#else
float max_buffer_c, max_chassis_power;
struct power_parameter_t
{
	float buffer_c;
} power_parameter;
#endif

char fric_text[30] = "FRIC";
char spin_text[30] = "SPIN";
char state_text[30] = "CVR X\nNOBUF\nSMALLBUF\nBIGBUF";

extern UI_figure_7_t dynamic_layer_data1;
extern UI_figure_7_t dynamic_layer_data2;
extern UI_figure_7_t dynamic_layer_data3;

extern UI_figure_7_t UI_7_dat1;
extern UI_figure_7_t UI_7_dat2;
extern UI_figure_7_t UI_7_dat3;

extern ext_game_robot_HP_t game_robot_HP;
extern ext_bullet_remaining_t bullet_remaining;
uint16_t robot_HP_data_RED[7];
uint16_t robot_HP_data_BLUE[7];
uint16_t RED_outpost;
uint16_t BLUE_outpost;
uint16_t robot_HP_max_RED[7];
uint16_t robot_HP_max_BLUE[7];
uint16_t show_robot_HP;
uint16_t show_robot_HP_max;


// 第二准心
#define SECOND_TARGET_X 910
#define SECOND_TARGET_Y 470

#define CHASSIS_YAW_UI_BIAS 90
#define CHASSIS_PITCH_CENTER_X BASIC_CENTER_X
#define CHASSIS_PITCH_CENTER_Y BASIC_CENTER_Y
// #define CHASSIS_CENTER_X 235
// #define CHASSIS_CENTER_Y 750
#define CHASSIS_PITCH_RADIUS 163
#define CHASSIS_CIRCLE_X 235
#define CHASSIS_CIRCLE_Y 750
#define CHASSIS_RADIUS 40
#define CHASSIS_DIRC_RAIUS 80
#define SECOND_TARGET_RAIUS 30

Line cap_V = {.rank=1,.type=LINE,.layer=DYNAMIC_LAYER,.color=COLOR_CYAN,.x1=645,.y1=200,.x2=1270,.y2=200,.width=1};
Line fric_tip = {.rank=2,.type=LINE,.layer=DYNAMIC_LAYER,.color=COLOR_MAIN_RB,.x1=900,.y1=490,.x2=1020,.y2=590,.width=9};

void input_HP(void);

void UI_init(void)
{
	// UI_create();
	#if USE_UI_OLD
	// 铅垂线
	Static_UI(11, LINE, COLOR_WHITE, 960,110,0,0,0,960,534,2);
	// 瞄准刻度线
	Static_UI(12, LINE, COLOR_WHITE, 850,300,0,0,0,1070,300,2);
	Static_UI(13, LINE, COLOR_WHITE, 915,350,0,0,0,1005,350,2);
	Static_UI(14, LINE, COLOR_WHITE, 940,400,0,0,0,980,400,2);
	Static_UI(15, LINE, COLOR_WHITE, 950,425,0,0,0,970,425,2);
	Static_UI(17, LINE, COLOR_WHITE, 950,450,0,0,0,970,450,2);
	#endif
}

void UI_update(void) // 此函数在裸机中调用会严重影响单发精度
{ 
	#if USE_UI_OLD
	static uint16_t UI_count;
	UI_count++;
	if (UI_count % 16 == 0){
		send_7_old(UI_7_dat2); // 勿动
		send_7_old(UI_7_dat3); // 勿动
	}
	if (UI_count == 33){ 
		// 所有要变化（MOD）东西的ADD来一遍 防止被刷掉了
		send_7_old(UI_7_dat1); // 勿动
	}
	#else
	UI_modify_graphic((uint16_t*)&cap_V);
	UI_modify_graphic((uint16_t*)&fric_tip);
	UI_send();
	#endif
}

void input_HP(void)
{
	uint8_t i;
	memcpy(robot_HP_data_RED, (void *)&(game_robot_HP.red_1_robot_HP), sizeof(uint16_t) * 7);
	memcpy(robot_HP_data_BLUE, (void *)(&(game_robot_HP.red_1_robot_HP) + 8), sizeof(uint16_t) * 7);
	RED_outpost = game_robot_HP.red_outpost_HP;
	BLUE_outpost = game_robot_HP.blue_outpost_HP;

	// 用一直更新的方法知道大概的血量上限
	for (i = 0; i < 7; i++)
	{
		robot_HP_max_RED[i] = (robot_HP_max_RED[i] < robot_HP_data_RED[i]) ? robot_HP_data_RED[i] : robot_HP_max_RED[i];
		robot_HP_max_BLUE[i] = (robot_HP_max_BLUE[i] < robot_HP_data_BLUE[i]) ? robot_HP_data_BLUE[i] : robot_HP_max_BLUE[i];
	}

	if (vision_ctrl_data.target)
	{
		if (vision_ctrl_data.id_num == 1 || vision_ctrl_data.id_num == 3 || vision_ctrl_data.id_num == 4 || vision_ctrl_data.id_num == 5)
		{
			i = vision_ctrl_data.id_num;

			if (game_robot_status.robot_id > 100)
			{ // 判断己方颜色 大于一百为蓝
				show_robot_HP = robot_HP_data_RED[i - 1];
				show_robot_HP_max = robot_HP_max_RED[i - 1];
			}
			else
			{
				show_robot_HP = robot_HP_data_BLUE[i - 1];
				show_robot_HP_max = robot_HP_max_BLUE[i - 1];
			}
		}
		else if (vision_ctrl_data.id_num == 6)
		{
			if (game_robot_status.robot_id > 100)
				show_robot_HP = robot_HP_data_RED[5];
			else
				show_robot_HP = robot_HP_data_BLUE[5]; // 哨兵
			show_robot_HP_max = 1000;
		}
		else if (vision_ctrl_data.id_num == 0)
		{
			if (game_robot_status.robot_id > 100)
				show_robot_HP = RED_outpost;
			else
				show_robot_HP = BLUE_outpost;
			show_robot_HP_max = 1500; // outpost max HP
		}
		else
		{
			show_robot_HP = 0;
			show_robot_HP_max = 100;
		}
	}
	else
	{
		show_robot_HP = 0;
		show_robot_HP_max = 100;
	}
}

Line get_line_pos(uint16_t center_x, uint16_t center_y, uint16_t radius, float theta) // theta为rad
{
	Line tempLine;
	tempLine.x1 = center_x - cosf(theta) * radius;
	tempLine.y1 = center_y - sinf(theta) * radius;
	tempLine.x2 = center_x + cosf(theta) * radius;
	tempLine.y2 = center_y + sinf(theta) * radius;
	return tempLine;
}
