/**
 * @file robot.c
 * @brief 机器人进程函数
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#include "robot.h"

extern UART_HandleTypeDef huart6;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern RNG_HandleTypeDef hrng;
extern TIM_HandleTypeDef M_TIM_HANDLE;
extern TIM_HandleTypeDef H_TIM_HANDLE;
extern TIM_HandleTypeDef IMU_TIM_HANDLE;
extern TIM_HandleTypeDef BEEP_TIM_HANDLE;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim13;


robot_t robot;
robot_state_t expected_state,current_state;

imu_data_t imu_data;

rc_ctrl_t *rc_recv;

uint32_t now_timestamp = 0;

int8_t imu_rslt;
uint16_t imu_init_cnt = 0;
int8_t imu_init(void){
  int8_t _rslt = 0;
  uint32_t rcc_csr = RCC->CSR;
  if ((rcc_csr & (RCC_CSR_PORRSTF | RCC_CSR_BORRSTF))){
        bmi08x_delay_us(50000, 0); // 50ms for POR/BOR startup
    }
    do{
        bmi08x_delay_us(3000, 0); // 3ms startup
        _rslt = init_bmi08x();
        if (_rslt != BMI08X_OK)
        {
            imu_init_cnt++;
            // beep(5, 10);
            bmi08x_delay_us(5000, 0);
        }
        if (imu_init_cnt > 5)
        {
            // imu init faile, reset all
            __set_FAULTMASK(1);
            NVIC_SystemReset();
            while (1);
        }
    } while (_rslt != BMI08X_OK);
    imu_data.raw_data.a_fullscale = ACCEL_RANGE_G * 9.7833f;
    imu_data.raw_data.g_fullscale = GYRO_RANGE_DPS;
    enable_bmi08x_interrupt();
    return _rslt;
}

/**
 * @brief 机器人上电初始化
 * @note 初始框架配置 不初始stm32化外设相关
 * @retval None
 */
void robot_init(void){
    __disable_irq();
  
    __HAL_RCC_CLEAR_RESET_FLAGS();

    rc_recv_dma_init();
		#if USE_VT_RC_UART
    vt_rc_recv_dma_init();
		#endif
    referee_recv_dma_init();
		// old_can_user_init(&hcan1);
		// old_can_user_init(&hcan2);
    // magazine_switch_init();
    behaviour_init(&robot);

    #if (BOARDS_MODE && MACHINE_TYPE == GIMBAL_MASTER) || !BOARDS_MODE
    shooter_init();
    gimbal_init();
    #endif

    motor_test_init();
	
		#if BOARDS_MODE
		board_interact_init();
		#endif

    UI_init();
		
		ws2812_init();
    asr_cmd_init();
		
    imu_rslt = imu_init();
		
    __enable_irq();

    beep_start_show();
}

/**
 * @brief 机器人自检函数 另外还负责给电机发0断控
 * @note 主要检测电机 CAN IMU RC等
 * @retval None
 */
void diag_slow_time_cycle(void)
{
  motor_switch(robot.robot_control_flag);
  magazine_ctrl_loop(robot.magazine_switch_flag);
  #if BOARDS_MODE
  update_boards_interact_status();
    #if MACHINE_TYPE == CHASSIS_SLAVE
    update_motor_status(CHASSIS_MOTORS);
    #elif MACHINE_TYPE == GIMBAL_MASTER
    update_motor_status(GIMBAL_MOTORS);
    update_motor_status(SHOOTER_MOTORS);
    #endif
  #else
  update_motor_status(DEFAULT);
  #endif
  detect_loop();
  if ((HAL_CAN_GetError(&hcan1) & HAL_CAN_ERROR_BOF))
  {
    CAN1->MCR |= CAN_MCR_INRQ;
    HAL_CAN_Start(&hcan1);
  }
  if ((HAL_CAN_GetError(&hcan2) & HAL_CAN_ERROR_BOF))
  {
    CAN2->MCR |= CAN_MCR_INRQ;
    HAL_CAN_Start(&hcan2);
  }

  inc_rc_rx_lost();
  inc_referee_rx_lost();
  inc_vision_rx_lost();
  buzzer_proc();
}

/**
 * @brief 机器人中速进程 200Hz
 * @retval None
 */
void robot_m_tim_cycle(void){
    rc_recv = get_rc_data_ptr();
    // behaviour_ctrl_loop(rc_recv,&robot);

    motor_test_loop(rc_recv,&robot);

    // chassis_ctrl_loop(rc_recv,&robot);    

    update_rc_last_key(robot.rc_sc);
}

/**
 * @brief 机器人高速进程 500Hz
 * @retval None
 */
void robot_h_tim_cycle(void){
    rc_recv = get_rc_data_ptr();
    behaviour_ctrl_loop(rc_recv,&robot);

    #if (BOARDS_MODE && MACHINE_TYPE == GIMBAL_MASTER) || !BOARDS_MODE
    gimbal_ctrl_loop(rc_recv,&robot);
    #endif
    #if (BOARDS_MODE && MACHINE_TYPE == GIMBAL_MASTER) || !BOARDS_MODE
    shooter_ctrl_loop(rc_recv,&robot);
    #endif
    
    // 视觉要用
    now_timestamp += 1000.f / (uint32_t)H_TIM_FREQ; // 单位ms

    update_rc_last_key(robot.rc_sc);
    #if BOARDS_MODE
    board_interact_send();
    #endif
}

/**
 * @brief 机器人bmi08x特供进程 1250Hz
 * @retval None
 */
void robot_imu_tim_cycle(void)
{ 
  imu_rslt = get_imu_att(&imu_data.fp_data, &imu_data.eular, &imu_data.temperature);
  if (imu_rslt == BMI08X_OK)
  {
    //	if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && rslt ==
    // BMI08X_OK){ send if usb is connected
    send_vision_request(imu_data.eular.roll * (2.0f * PI / IMU_RANGE),
						            imu_data.eular.pitch * (2.0f * PI / IMU_RANGE),
                        imu_data.eular.yaw * (2.0f * PI / IMU_RANGE));
  }
}

/**
 * @brief 机器人无指定时序进程 ??Hz
 * @retval None
 */
void robot_main_loop(void)
{
  // 需要控制运行速度可以在这里加丐版延时
  // uint32_t tick_start = HAL_GetTick();
  // while (HAL_GetTick() - tick_start < 1000 / (uint32_t)MIN_MAIN_LOOP_FREQ);
  // update_vofa();
}

/**
 * @brief 阻塞延时
 */
uint32_t robot_delay_us_tick =0;
extern TIM_HandleTypeDef DELAY_TIM_HANDLE;
void robot_delay_us(uint32_t us){
  __HAL_TIM_SetCounter(&DELAY_TIM_HANDLE,0);
	__HAL_TIM_ENABLE(&DELAY_TIM_HANDLE);
	
	while((robot_delay_us_tick = __HAL_TIM_GetCounter(&DELAY_TIM_HANDLE))<us);
	
	__HAL_TIM_DISABLE(&DELAY_TIM_HANDLE);
}
