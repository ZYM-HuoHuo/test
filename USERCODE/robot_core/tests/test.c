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

/*pid结构体*/
pid_struct_t fric_pid[2];
pid_struct_t dial_rpm_pid; // 连发对速度PID
pid_struct_t dial_ang_pid; // 单发对角度PID

/*固定速度设置*/
float set_fric_rpm = FRIC_TEST_RPM; // 摩擦轮设定(rpm)
float set_dial_rpm = DIAL_RPM;      // 推弹速度设定(rpm)
float exp_dial_rpm = 0;             // DT7拨轮给的期望



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
  memcpy(&dial_rpm_pid, &dial_rpm_pid_init_val, sizeof(pid_struct_t));
  memcpy(&dial_ang_pid, &dial_ang_pid_init_val, sizeof(pid_struct_t));

    //  3   0x203   0x200
  RM_QUAD_motor_register(&motors->dial, motor_model_RM_QUAD_M2006, QUAD_CURR,
                         COM_CAN, (uint32_t *)&SHOOTER_MOTORS_HCAN, 3, RR_M2006,
                         &dial_lpf_init_val, M_SHOOTER);


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
