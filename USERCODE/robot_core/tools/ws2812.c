/**
 * @file ws2812.c
*
 * @brief ws2812灯珠驱动
 *
 *
 * @copyright SCNU-PIONEER (c) 2023-2024
 *
 */
#include "app/robot.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

#include "usbd_cdc_if.h"
#include "tools/buzzer.h"
#include "tools/ws2812.h"
#include "tools/vofa.h"

#include "algorithm/crc.h"
#include "algorithm/filter.h"
#include "algorithm/util.h"

#include "drv_conf.h"
#include "Base/drv_uart.h"
#include "Devices/MOTOR/motor_headers.h"

#include "OS/drv_freertos/include/FreeRTOS.h"
#include "OS/drv_freertos/include/task.h"
#include "OS/drv_freertos/include/self_delay.h"

#include "app/behaviour.h"
#include "app/imu_bmi088.h"

extern TIM_HandleTypeDef WS2812_TIM_HANDLE;
extern uint32_t now_timestamp;
extern RAM_PERSIST uint8_t vt_rc_rx_lost;

uint8_t enable_rainbow_flag = 0;
uint16_t ws2812_init_cnt = 0;

// 颜色预设，顺序为{RED,GREEN,BLUE}
const Color BRIGHT_RED = {231, 37, 32}, DULL_RED = {200, 22, 29};
const Color THIN_RED = {242, 155, 135}, RED = {216, 33, 13};

const Color THICK_PINK = {240, 145, 145}, THIN_PINK = {250, 219, 217};
const Color ROSE_PINK = {213, 89, 111}, PINK = {214, 0, 111};

const Color FLASH_GREEN = {217, 228, 128}, SPRING_GREEN = {105, 178, 115};
const Color WITHERED_GREEN = {175, 186, 127}, GERM_GREEN = {0, 161, 86};

const Color SAKS_BLUE = {78, 140, 168}, WATER_BLUE = {113, 199, 213};
const Color BLACK_BLUR = {4, 22, 58}, MOUSE_BLUE = {90, 118, 174};

const Color FRESH_YELLOW = {255, 234, 0}, MOON_YELLOW = {255, 237, 97};
const Color GOLDEN_YELLOW = {219, 180, 0}, FLOWER_YELLOW = {255, 236, 148};

const Color GRAY = {129, 129, 129};
const Color SILVER = {211, 211, 212};
const Color BLACK = {0};
const Color WHITE = {2, 2, 4};

ws2812s_t ws2812s;
// 发送数组
uint32_t pixel_buffer[LED_NUM + RESET_BIT][24]__attribute__((at(0x20008190)));
/*----------------------------------↓驱动↓----------------------------------------*/
ws2812s_t *get_ws2812s_ptr(void){
	return &ws2812s;
}
// 将三原色单独数据合并为24位数据
uint32_t rgb24_forward(uint8_t red, uint8_t green, uint8_t blue){
    return green << 16 | red << 8 | blue;
}
// 从24位数据中提取三原色分量
Color rgb24_reverse(uint32_t color24){
    return (Color){(color24 >> 8) & 0xFF,(color24 >> 16) & 0xFF,color24 & 0xFF};
}
// 轮转中间层函数
uint32_t ws2812_wheel(uint8_t wheelPos){
    wheelPos = 255 - wheelPos;
    if (wheelPos < 85){
        return rgb24_forward(255 - wheelPos * 3, 0, wheelPos * 3);
    }
    if (wheelPos < 170){
        wheelPos -= 85;
        return rgb24_forward(0, wheelPos * 3, 255 - wheelPos * 3);
    }
    wheelPos -= 170;
    return rgb24_forward(wheelPos * 3, 255 - wheelPos * 3, 0);
}
// 设置颜色
void ws2812_setColor(ws2812_t *ws2812)
{
	switch(ws2812->state){
		case WS2812_OFF:
			ws2812->color.rgb = BLACK;break;
		case WS2812_NORMAL:
			ws2812->color.rgb = GERM_GREEN;break;
		case WS2812_ABNORMAL:
			ws2812->color.rgb = FRESH_YELLOW;break;
		case WS2812_LEVEL1:
			ws2812->color.rgb = GRAY;break;
		case WS2812_LEVEL2:
			ws2812->color.rgb = FRESH_YELLOW;break;
		case WS2812_LEVEL3:
			ws2812->color.rgb = GERM_GREEN;break;
		case WS2812_SP1:
			ws2812->color.rgb = ROSE_PINK;break;
		case WS2812_SP2:
			ws2812->color.rgb = SAKS_BLUE;break;
		case WS2812_SP3:
			ws2812->color.rgb = DULL_RED;break;
	}
	uint8_t i;
	uint8_t id = ws2812->pos;
	float ratio = ws2812->degree_percent;
	Color tmp_color = (Color){ws2812->color.rgb.R * ratio, ws2812->color.rgb.G * ratio, ws2812->color.rgb.B * ratio};
	if (id > LED_NUM)
		return;
	for (i = 0; i < 8; i++)
		pixel_buffer[id][i] = ((tmp_color.G & (1 << (7 - i))) ? CODE1 : CODE0);
	for (i = 8; i < 16; i++)
		pixel_buffer[id][i] = ((tmp_color.R & (1 << (15 - i))) ? CODE1 : CODE0);
	for (i = 16; i < 24; i++)
		pixel_buffer[id][i] = ((tmp_color.B & (1 << (23 - i))) ? CODE1 : CODE0);
}

// 存储用于发送一个灯颜色后的等待码
void ws2812_reset(){
	for (uint8_t i = 0; i < 24; i++)
		pixel_buffer[LED_NUM][i] = 0;
}

// 发送函数
void ws2812_update(){
	ws2812_reset();
	#if USE_WS2812
	HAL_TIM_PWM_Start_DMA(&WS2812_TIM_HANDLE, WS2812_TIM_CHANNAL, (uint32_t *)pixel_buffer, (LED_NUM + 1) * 24);
	#endif
}
/*----------------------------------↑驱动↑----------------------------------------*/
// 自定义某灯色
void ws2812_setSingleColor(uint8_t pos, Color color){
	Color tmp_color;
	for (uint8_t i = 0; i < LED_NUM; i++){
		float ratio = ws2812s._[i].degree_percent;
		if(ws2812s._[i].pos == i){
			tmp_color = (Color){ws2812s._[i].color.rgb.R * ratio, ws2812s._[i].color.rgb.G * ratio, ws2812s._[i].color.rgb.B * ratio};
		}
		uint8_t j;
		uint8_t id = ws2812s._[i].pos;
		if (id > LED_NUM)
			return;
		for (j = 0; j < 8; j++)
			pixel_buffer[id][j] = ((tmp_color.G & (1 << (7 - j))) ? CODE1 : CODE0);
		for (j = 8; j < 16; j++)
			pixel_buffer[id][j] = ((tmp_color.R & (1 << (15 - j))) ? CODE1 : CODE0);
		for (j = 16; j < 24; j++)
			pixel_buffer[id][j] = ((tmp_color.B & (1 << (23 - j))) ? CODE1 : CODE0);
		break;
	}
}
// RAINBOW!
void ws2812_rainbow(uint8_t wait){
    uint16_t i, j;
    for (j = 0; j < 256; j++){
        for (i = 0; i < LED_NUM; i++){
            ws2812_setSingleColor(i, rgb24_reverse(ws2812_wheel((i + j) & 255)));
        }
        ws2812_update();
		#if OS_ENABLE
		delay_ms(wait);
		#else
        delay_xms(wait);
		#endif
    }
}
// 单色轮转亮
void ws2812_colorWipe(Color c, uint8_t wait){
    for (uint16_t i = 0; i < LED_NUM; i++){
        ws2812_setSingleColor(i, c);
        ws2812_update();
        #if OS_ENABLE
		delay_ms(wait);
		#else
        delay_xms(wait);
		#endif
    }
}
// 整体rainbow而非单个灯珠rainbow (比较耗时)
void ws2812_rainbowCycle(uint8_t wait){
    uint16_t i, j;
    for (j = 0; j < 256 * 5; j++){ // 5 cycles of all colors on wheel
        for (i = 0; i < LED_NUM; i++){
            ws2812_setSingleColor(i, rgb24_reverse(ws2812_wheel(((i * 256 / LED_NUM) + j) & 255)));
        }
        ws2812_update();
        #if OS_ENABLE
		delay_ms(wait);
		#else
        delay_xms(wait);
		#endif
    }
}
// Theatre-style crawling lights.
void ws2812_theaterChase(Color c, uint8_t wait){
    for (int j = 0; j < 10; j++){ // do 10 cycles of chasing
        for (int q = 0; q < 3; q++){
            for (uint16_t i = 0; i < LED_NUM; i = i + 3){
                ws2812_setSingleColor(i + q, c); // turn every third pixel on
            }
            ws2812_update();
			#if OS_ENABLE
			delay_ms(wait);
			#else
			delay_xms(wait);
			#endif
            for (uint16_t i = 0; i < LED_NUM; i = i + 3){
                ws2812_setSingleColor(i + q, (Color){0,0,0}); // turn every third pixel off
            }
        }
    }
}
// Theatre-style crawling lights with rainbow effect
void ws2812_theaterChaseRainbow(uint8_t wait){
    for (int j = 0; j < 256; j++){ // cycle all 256 colors in the wheel
        for (int q = 0; q < 3; q++){
            for (uint16_t i = 0; i < LED_NUM; i = i + 3){
                ws2812_setSingleColor(i + q, rgb24_reverse(ws2812_wheel((i + j) % 255))); // turn every third pixel on
            }
            ws2812_update();
			#if OS_ENABLE
			delay_ms(wait);
			#else
			delay_xms(wait);
			#endif
            for (uint16_t i = 0; i < LED_NUM; i = i + 3){
                ws2812_setSingleColor(i + q, (Color){0,0,0}); // turn every third pixel off
            }
        }
    }
}

// 初始化
void ws2812_init(void)
{
	ws2812s.chassis_l.pos = 0;
	ws2812s.gimbal_l.pos = 1;
	ws2812s.rc_l.pos = 2;
	ws2812s.vision_l.pos = 3;
	ws2812s.vision_fitting_l.pos = 4;
	ws2812s.vision_hz_l.pos = 5;
	ws2812s.vision_target_l.pos = 6;
	ws2812s.reserve1_l.pos = 7;
	ws2812s.referee_l.pos = 8;
	ws2812s.shooter_l.pos = 9;

	for (uint8_t i = 0; i < LED_NUM; i++)
	{
		ws2812s._[i].degree_percent = LIGHT_DEGREE;
		if (i % 2)
			ws2812s._[i].state = WS2812_SP1;
		else
			ws2812s._[i].state = WS2812_SP2;
		ws2812_setColor(&ws2812s._[i]);
	}
	ws2812_update();
}
// 程序错误指示灯
void ws2812_error(void){
	for(uint8_t i = 0;i<LED_NUM;i++){
		ws2812s._[i].state = WS2812_SP3;
		ws2812_setColor(&ws2812s._[i]);
	}
	ws2812_update();
}
void ws2812_loop(void){
	for(uint8_t i = 0;i<LED_NUM;i++){
		ws2812_setColor(&ws2812s._[i]);
	}
	ws2812_update();
}
