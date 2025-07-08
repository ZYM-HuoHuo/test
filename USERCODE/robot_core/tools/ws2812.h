#ifndef _WS2812_H_
#define _WS2812_H_

#include "main.h"
#define LED_NUM 10

#define RESET_BIT 1
// 0和1码对应的比较值
#define CODE1 134
#define CODE0 67
#define WS2812_INTI_MAX_CNT 50

#define LIGHT_DEGREE 0.015f // 给定亮度 (%)
typedef enum{
	WS2812_OFF,

	WS2812_NORMAL,		//正常
	WS2812_ABNORMAL,	//异常

	WS2812_LEVEL1,		//表示程度
	WS2812_LEVEL2,		
	WS2812_LEVEL3,

	WS2812_SP1,			//特殊1
	WS2812_SP2,			//特殊2
	WS2812_SP3,			//特殊3
}ws2812_state_t;
typedef struct{
	uint8_t R;
	uint8_t G;
	uint8_t B;	
}Color;
typedef struct
{
	union{
		Color rgb;
		uint32_t rgb_24bit;
	}color;
	float degree_percent; // 0-100%
	uint8_t pos; // 从左往右 位置即是id
	ws2812_state_t state;
}ws2812_t;
typedef union{
	struct{
		ws2812_t chassis_l;
		ws2812_t gimbal_l;
		ws2812_t shooter_l;
		ws2812_t rc_l;
		ws2812_t vision_l;
		ws2812_t vision_target_l;
		ws2812_t vision_hz_l;
		ws2812_t vision_fitting_l;
		ws2812_t referee_l;
		ws2812_t reserve1_l; // 未安排作用
	};
	ws2812_t _[LED_NUM];
}ws2812s_t;

extern uint16_t ws2812_init_cnt;

ws2812s_t *get_ws2812s_ptr(void);

void ws2812_update(void);

void ws2812_init(void);
void ws2812_error(void);
void ws2812_loop(void);

void ws2812_setSingleColor(uint8_t pos, Color color);
void ws2812_rainbow(uint8_t wait);
void ws2812_colorWipe(Color c, uint8_t wait);
void ws2812_rainbowCycle(uint8_t wait);
void ws2812_theaterChase(Color c, uint8_t wait);
void ws2812_theaterChaseRainbow(uint8_t wait);
#endif
