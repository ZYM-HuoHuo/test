#include "buzzer.h"
#include HAL_INCLUDE

extern TIM_HandleTypeDef BEEP_TIM_HANDLE;

uint32_t beep_cycle, beep_cnt, beep_cnt_loaded;
static uint32_t beep_psc = 0;

void beep(uint32_t cycle, uint32_t cnt, uint32_t ARR){
	BEEP_TIM_HANDLE.Instance->ARR = ARR;
	beep_cycle = 2*cycle; 
	beep_cnt_loaded = cnt;
	beep_cnt = 0;
}

uint8_t beep_start_show_cnt = 0; // 置于detectloop中
bool beep_start_show_enable = false;

void buzzer_proc(void){
	if(beep_start_show_enable){
		if(beep_start_show_cnt < 2){
			beep(1, 4, 600);
		}
		else if(beep_start_show_cnt < 7){
			beep(1, 1, 400);
		}
		else{
			beep_start_show_enable = false;
		}
		beep_start_show_cnt++;
	}
	beep_psc = ((beep_cycle+1) % 2)*125;
	if(beep_cycle > 0){
		if(beep_cnt > 0) beep_cnt--; else{
			__HAL_TIM_SetCompare(&BEEP_TIM_HANDLE, BEEP_TIM_CHANNAL, beep_psc);
			beep_cnt = beep_cnt_loaded;
			beep_cycle--;
		}	
	}
}
void beep_start_show(void){
	beep_start_show_enable = true;
}
