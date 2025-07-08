/**
 * @file bus_detect.c
*
 * @brief 检测模块状态(未完成
 *
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#include "bus_detect.h"

// #include "dwt.h"
#include "Devices/MOTOR/motor_headers.h"
#include "Base/rc.h"
#include "Base/referee.h"

#include "tools/buzzer.h"
#include "tools/ws2812.h"
#include "tools/asr.h"

#define POWER_UP_DELAY 8
#define MODULE_RX_LOST_MAX 100

#define HZ_QUALITY_RANK1 1.f / 70.f
#define HZ_QUALITY_RANK2 1.f / 35.f

#if CHASSIS_TYPE != OMNI && CHASSIS_TYPE != MECANUM
#error("The Robot type is not supported!")
#endif
#if !defined(MOTOR_CAN_ENABLE) || MOTOR_CAN_ENABLE == 0
#error("MOTOR DISABLE!")
#endif
#if !BOARDS_MODE && MACHINE_TYPE == CHASSIS_SLAVE
#error("You can't be a slave with only one board!")
#endif
#if !READY_FOR_BATTLE
#warning("No Kill Protect!")
#endif
#if !USE_REFEREE
#warning("Unlimited Power!")
#endif
#if !USE_GIMBAL_RESET || !USE_CHASSIS_RESET
#warning("Disable Soft Launching!")
#endif

extern uint32_t now_timestamp;
extern RAM_PERSIST uint8_t dt7_rc_rx_lost;
extern RAM_PERSIST uint8_t vt_rc_rx_lost;

void detect_loop(void)
{
	#if MACHINE_TYPE == GIMBAL_MASTER
	motors_t *motors = get_motors_ptr();
	vision_ctrl_t *vision = get_vision_ctrl_data_ptr();
	robot_t *robot = get_robot_ptr();

	ws2812s_t *ws2812s = get_ws2812s_ptr();
	// 这里周期性检测非触发性异常 建议搭配ws2812和asr食用;
	float hz_quality = 1 / (now_timestamp - vision->r_timestamp);
	// asr_control_enable_cmd(); // debug
	// 反馈云台在线情况
	if (is_motors_offline(GIMBAL_MOTORS)){
		asr_struct_offline_cmd(GIMBAL_MOTORS);
		ws2812s->gimbal_l.state = WS2812_ABNORMAL;
	} 
	else ws2812s->gimbal_l.state = WS2812_NORMAL;

	// 反馈底盘在线情况
	if (is_motors_offline(CHASSIS_MOTORS)){
		asr_struct_offline_cmd(CHASSIS_MOTORS);
		ws2812s->chassis_l.state = WS2812_ABNORMAL;
	}
	else ws2812s->chassis_l.state = WS2812_NORMAL;

	// 反馈发射机构在线情况
	if (is_motors_offline(SHOOTER_MOTORS)){
		asr_struct_offline_cmd(SHOOTER_MOTORS);
		ws2812s->shooter_l.state = WS2812_ABNORMAL;
	} 
	else ws2812s->shooter_l.state = WS2812_NORMAL;

	// 反馈裁判系统在线情况
	if (game_robot_status.shooter_barrel_heat_limit == 0){
		asr_referee_offline_cmd();
		ws2812s->referee_l.state = WS2812_ABNORMAL;
	}
	else ws2812s->referee_l.state = WS2812_NORMAL;

	// 反馈视觉是否检测到目标情况
	if (vision->header)ws2812s->vision_l.state = WS2812_NORMAL;
	else ws2812s->vision_l.state = WS2812_ABNORMAL;

	// 反馈遥控指令情况
	if (dt7_rc_rx_lost == RC_RX_LOST_MAX && vt_rc_rx_lost == VT_RC_RX_MAX_LOST)
		ws2812s->rc_l.state = WS2812_LEVEL1;
	else if(vt_rc_rx_lost == VT_RC_RX_MAX_LOST) ws2812s->rc_l.state = WS2812_LEVEL2;
	else if(dt7_rc_rx_lost == RC_RX_LOST_MAX) ws2812s->rc_l.state = WS2812_LEVEL3;

	// 反馈视觉目标
	if (vision->target == 0) ws2812s->vision_target_l.state = WS2812_LEVEL1;
	else if (vision->target == 1) // armor
		ws2812s->vision_target_l.state = WS2812_SP1;
	else if (vision->target == 2) // buff
		ws2812s->vision_target_l.state = WS2812_SP2;

	// 反馈上下位机数据交互品质
	if (hz_quality <= HZ_QUALITY_RANK1) ws2812s->vision_hz_l.state = WS2812_LEVEL1;
	else if (hz_quality > HZ_QUALITY_RANK1 && hz_quality < HZ_QUALITY_RANK2) ws2812s->vision_hz_l.state = WS2812_LEVEL2;
	else ws2812s->vision_hz_l.state = WS2812_LEVEL3;
	
	// 反馈状态观测器运行情况
	if (vision->vz <= 0.01f) ws2812s->vision_fitting_l.state = WS2812_NORMAL;
	else ws2812s->vision_fitting_l.state = WS2812_ABNORMAL;

	// // 反馈电机情况
	// for(uint8_t i = 0;i< NUM_OF_ALL_MOTOR;i++){
	// 	if(motors->_[i].INFO.error_set[0] == MotorError_OverHeat)
	// 	asr_motor_heat_cmd(i);
	// }

	// 刷新灯珠
	if (ws2812_init_cnt < WS2812_INTI_MAX_CNT){
		ws2812_init_cnt++;
	}
	else
		ws2812_loop();

	asr_cmd_send_loop(TIM3_IT_FREQ/2);	
	#endif
}
void disable_all_it(void)
{
	__disable_irq();
	for (int i = 0; i < 8; i++)
	{
		NVIC->ICER[i] = (uint32_t)(0xFFFFFFFF); // disable all interrupts
		NVIC->ICPR[i] = (uint32_t)(0xFFFFFFFF); // clear pendingbits
	}
	__DSB();
	__ISB();
	__enable_irq();
}

void mpu_nx_protect(void)
{
	MPU_Region_InitTypeDef MPU_InitStruct;

	HAL_MPU_Disable();

	MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	MPU_InitStruct.BaseAddress = 0x20000000;
	MPU_InitStruct.Size = MPU_REGION_SIZE_512MB;
	MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
	MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
	MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
	MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
	MPU_InitStruct.Number = MPU_REGION_NUMBER0;
	MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
	MPU_InitStruct.SubRegionDisable = 0x00;
	MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);

	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}
// 函数预设给超电保留
float update_data_refresh_freq(rfreq_t *s_rf, float data_curr)
{
	if (data_curr != s_rf->data_last)
	{
		s_rf->data_last = data_curr;
		uint32_t delta_tick = (HAL_GetTick() - s_rf->tick_start);
		s_rf->refresh_rate = (delta_tick == 0 ? -1 : 1000.f / delta_tick);
		s_rf->tick_start = HAL_GetTick();
	}
	else
		return -2.f;
	return s_rf->refresh_rate;
}
// void dwt_write_watch(uint32_t addr, uint32_t size) {
//     if(size > 0x800) return;
//   /*
//   const size_t num_comparators = (A_DWT->CTRL>>28) & 0xF;
//   if (comp_id > num_comparators) return;
//   dwt_config_t *config = &(A_DWT->COMP_CONFIG[comp_id]);
//   config->COMP = addr;
//   config->MASK = size;
//   // set list since this will enable the comparator
//   config->FUNCTION = 0x6; //0x5 for read, 0x6 for write
//   */
//     DWT->FUNCTION0 = 0x0;
//     SET_BIT(CoreDebug->DEMCR,CoreDebug_DEMCR_TRCENA_Msk);
// 	//CoreDebug_DEMCR_DWTENA_Msk
//     SET_BIT(DWT->CTRL,0x01);
//     DWT->COMP0 = addr;
//     DWT->MASK0 = size;
//     DWT->FUNCTION0 = 0x6; //0x5 for read, 0x6 for write
//     __DSB();
// }

// void disable_dwt(void){
// 	DWT->FUNCTION0 = 0x0;
// 	CLEAR_BIT(CoreDebug->DEMCR,CoreDebug_DEMCR_TRCENA_Msk);
// 	__DSB();
// }

// FUNCTION        Evaluation Performed        Event Generated
// 0b0000 (0x0)    None                        Comparator is Disabled
// 0b0100 (0x4)    Instruction Fetch           PC watchpoint debug event
// 0b0101 (0x5)    Read/Load Data Access       Watchpoint debug event
// 0b0110 (0x6)    Write/Store Data Access     Watchpoint debug event
// 0b0111 (0x7)    Read or Write Data Access   Watchpoint debug event

// #define SECT_START(_name_)                _name_##$$Base
// #define SECT_END(_name_)                  _name_##$$Limit
// extern const uint32_t SECT_START(STACK);  //end of the stack
// //extern uint32_t __initial_sp;
// //#define MAX_STACK_SIZE 0x400
// //sp_addr = (uint32_t)(&__initial_sp);

// void enable_stack_protect(uint32_t detect_size){
//     //setup an area in end of stack(low address) in DWT, it will send a debug event when written,
//     //the halt eventally lead to a watchdog reset
//     //a scatter load after reset may trigger DWT, please put stack segement into non-init segment
//     uint32_t addr =  (uint32_t)(&SECT_START(STACK)) + detect_size;
//     dwt_write_watch(addr,detect_size);
// }

// MPU Manage
// void HAL_MPU_ConfigRegion  ( MPU_Region_InitTypeDef *  MPU_Init )
// void HAL_MPU_Enable  ( uint32_t  MPU_Control )

// Manual Reset
//__set_FAULTMASK(1); // ??3�ڧ�?-????????-???
// NVIC_SystemReset(); // ?�ҧ�????
