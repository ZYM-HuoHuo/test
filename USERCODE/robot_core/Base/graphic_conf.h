/**
 * @file graphic_conf.h
*
 * @brief 通用图形配置
 *
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#ifndef _GRAPHIC_CONF_H
#define _GRAPHIC_CONF_H

#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"

typedef struct{
    // 两点确定一条直线
    uint16_t type; 
    uint16_t layer; 
    uint16_t color;
    uint16_t reserve1; 
    uint16_t reserve2; 
    uint16_t width; 
    uint16_t x1; 
    uint16_t y1; 
    uint16_t reserve3; 
    uint16_t x2; 
    uint16_t y2;
    uint16_t rank;
}Line;
typedef struct{
    // 对角顶点坐标
    uint16_t type; 
    uint16_t layer; 
    uint16_t color;
    uint16_t reserve1; 
    uint16_t reserve2; 
    uint16_t width; 
    uint16_t x1; 
    uint16_t y1; 
    uint16_t reserve3; 
    uint16_t x2; 
    uint16_t y2;
    uint16_t rank;
}Square;
typedef struct{
    // 圆心坐标 半径
    uint16_t type; 
    uint16_t layer; 
    uint16_t color;
    uint16_t reserve1; 
    uint16_t reserve2; 
    uint16_t width; 
    uint16_t x; 
    uint16_t y; 
    uint16_t r; 
    uint16_t reserve3; 
    uint16_t reserve4;
    uint16_t rank;
}Circle;
typedef __packed struct{
    // 圆心坐标 长短半轴
    uint16_t type; 
    uint16_t layer; 
    uint16_t color;
    uint16_t reserve1;
    uint16_t reserve2;
    uint16_t width; 
    uint16_t x; 
    uint16_t y; 
    uint16_t reserve3; 
    uint16_t a;  
    uint16_t b;
    uint16_t rank;
}Ellipse;
typedef struct{
    // 圆心坐标 长短半轴 始终角度
    uint16_t type; 
    uint16_t layer; 
    uint16_t color;
    uint16_t ang1;    // 起始角度
    uint16_t ang2;    // 终止角度
    uint16_t width; 
    uint16_t x; 
    uint16_t y; 
    uint16_t reserve1; 
    uint16_t a;  
    uint16_t b;
    uint16_t rank;
}Arc;
typedef struct{
    uint16_t type; 
    uint16_t layer; 
    uint16_t color;
    uint16_t size;
    uint16_t reserve1;
    uint16_t width; 
    uint16_t x; 
    uint16_t y; 
    uint16_t a1; 
    uint16_t a2;  
    uint16_t a3;
    uint16_t rank;
}Float;
typedef struct{
    uint16_t type; 
    uint16_t layer; 
    uint16_t color;
    uint16_t size;
    uint16_t reserve1;
    uint16_t width; 
    uint16_t x; 
    uint16_t y; 
    uint16_t a1; 
    uint16_t a2;  
    uint16_t a3;
    uint16_t rank;
}Int;


typedef struct{
    uint16_t type; 
    uint16_t layer; 
    uint16_t color;
    uint16_t size;
    uint16_t char_rank;
}Char_Config;
typedef struct{
    char str[30];
}Char_Text;
typedef struct{
    Char_Config config; 
    Char_Text text;
}Char;


#endif
