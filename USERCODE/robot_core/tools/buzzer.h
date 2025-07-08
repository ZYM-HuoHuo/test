#ifndef _BUZZER_H
#define _BUZZER_H

#include "drv_conf.h"

void buzzer_proc(void);
void beep(uint32_t cycle, uint32_t cnt, uint32_t ARR);
void beep_start_show(void);

#endif
