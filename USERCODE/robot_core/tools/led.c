#include "led.h"

#include "main.h"

float real_s = 0;
uint8_t pin_index = 0;
GPIO_PinState pinstate[3] = {GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET};
void RGB_loop(uint32_t tim_freq, float s)
{
    // freq为所在定时器频率
	  // s表示几秒换颜色
    real_s = (float)tim_freq / 1.f;
    if(real_s > s){
			if (pin_index++ == 3) pin_index = 0;
			for(uint8_t i = 0; i < 3; i++){
				if(i != pin_index)
					pinstate[i] = GPIO_PIN_RESET;
				else pinstate[pin_index] = GPIO_PIN_SET;
			}
			real_s = 0;
    }
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, pinstate[0]);
    HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, pinstate[1]);
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, pinstate[2]);
}
