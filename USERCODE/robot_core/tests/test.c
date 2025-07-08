/**
 * @file test.c
*
 * @brief 测试电机的抽象机构
 *
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#include "test.h"

#include "drv_conf.h"
#include "Base/referee.h"
#include "Devices/MOTOR/motor_headers.h"

#include "algorithm/filter.h"
#include "algorithm/imu_fusion.h"
#include "algorithm/util.h"

#include "APP/behaviour.h"
#include "APP/bus_detect.h"
#include "APP/gimbal.h"
#include "APP/pid_lpf_param.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

const pid_struct_t test_motor_pid_init_value = {
    .kp = 0.0f,
	.ki = 0.0f,     
	.kd = 0.0f,      
	.i_max = C620_CURR_DATA_MAX/50,
	.out_max = C620_CURR_DATA_MAX,
	.k_deadband = 500
};
LPF_t test_motor_lpf = {
    .fc = M_TIM_FREQ,
    .ts = 1,
};
pid_struct_t test_motor_pid;

float motor_output = 0;

void motor_test_init(void){
    #if USE_TEST_FILE
    motors_t *motors = get_motors_ptr();
    memcpy(&test_motor_pid, &test_motor_pid_init_value, sizeof(pid_struct_t));
    //  CAN1
    //  ID  rxId    txId
    //  7   0x207   0x1FF
    RM_QUAD_motor_init(&motors->test,motor_model_RM_QUAD_M3508,
                  &hcan1,4,M_TIM_FREQ,&test_motor_lpf);
		can_user_init(
      &TEST_MOTORS_HCAN,
      (uint32_t[]){rx_stdid_of(motors->test),
                   0,0,
                   0,
                   },1);
    #endif
    ;
}
HAL_StatusTypeDef motor_test_loop(rc_ctrl_t *rc_recv, robot_t *robot){
    #if USE_TEST_FILE
    motors_t *motors = get_motors_ptr(); 
    extern float rocker_ry;
    // if(robot->robot_control_flag == DISABLE)
    //     motor_output = 0;
    // else 
    //     motor_output = s_curve(2.0f,rocker_ry);
    LOAD_MOTOR_TFF(motors->test,motor_output);
    return set_all_motor_output(TEST_MOTORS);
    #else
    return HAL_OK;
		#endif
}
