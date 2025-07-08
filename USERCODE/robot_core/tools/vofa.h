#ifndef VOFA_H
#define VOFA_H
#include "drv_conf.h"
#include HAL_INCLUDE
#ifdef __USART_H__
#include "usart.h"
#else
#include "main.h"
#endif
#ifdef __cplusplus
extern "C" {
#endif

void update_vofa(void);

#ifdef __cplusplus
}
#endif

#endif
