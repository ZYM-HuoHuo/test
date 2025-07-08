/**
 * @file motor_conf_sample.h
 *
 *
 * @brief 电机库主要配置示例文件
 * @note 根据下面的修改指引 创建副本并改名为motor_conf.h于根目录
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#ifndef MOTOR_CONF_H_
#define MOTOR_CONF_H_

#include "./motor_def.h"
#include "drv_conf.h"
#ifndef HAL_INCLUDE
#error "HAL_INCLUDE is not defined"
#else
#include HAL_INCLUDE // 本宏来自 drv_conf:stm32xxxx_hal.h
#endif
#ifndef CAN_INCLUDE
#error "CAN_INCLUDE is not defined"
#else
#include CAN_INCLUDE // 本宏来自 drv_conf:fdcan.h
#endif               // ! CAN_INCLUDE

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_CONF_FILE "motor_conf.h"

#if 0
 __  __  ___ _____ ___  ____        ____ ___  _   _ _____ 
|  \/  |/ _ \_   _/ _ \|  _ \      / ___/ _ \| \ | |  ___|
| |\/| | | | || || | | | |_) |    | |  | | | |  \| | |_   
| |  | | |_| || || |_| |  _ <     | |__| |_| | |\  |  _|  
|_|  |_|\___/ |_| \___/|_| \_\     \____\___/|_| \_|_|    

 _____ ___  _     _     _____        _____ _   _  ____ 
|  ___/ _ \| |   | |   / _ \ \      / /_ _| \ | |/ ___|
| |_ | | | | |   | |  | | | \ \ /\ / / | ||  \| | |  _ 
|  _|| |_| | |___| |__| |_| |\ V  V /  | || |\  | |_| |
|_|   \___/|_____|_____\___/  \_/\_/  |___|_| \_|\____|

#endif
#if 0
#define WHEELS_NUM           2 // 轮毂电机数量

/*--------------------------------*/
#define GIMBAL_MOTORS_HCAN   hcan1
#define GIMBAL_MOTORS_NUM    2
#define GIMBAL_MOTORS_INDEX  (1 - 1) // init_index:0
/*--------------------------------*/
#define SHOOTER_MOTORS_HCAN  hcan2
#define FRIC_MOTORS_HCAN     SHOOTER_MOTORS_HCAN
#define DIAL_MOTORS_HCAN     SHOOTER_MOTORS_HCAN
#define SHOOTER_MOTORS_NUM   3
#define SHOOTER_MOTORS_INDEX (GIMBAL_MOTORS_INDEX + GIMBAL_MOTORS_NUM) // init_index:2
#define FRIC_MOTORS_NUM      2
#define FRIC_MOTORS_INDEX    SHOOTER_MOTORS_INDEX // init_index:2
#define DIAL_MOTORS_NUM      1
#define DIAL_MOTORS_INDEX    (FRIC_MOTORS_INDEX + FRIC_MOTORS_NUM) // init_index:4
/*--------------------------------*/
#define WHEEL_MOTORS_HCAN    hcan2
#define HIP_MOTORS_HCAN      hcan1
#define CHASSIS_MOTORS_NUM   6
#define CHASSIS_MOTORS_INDEX (SHOOTER_MOTORS_INDEX + SHOOTER_MOTORS_NUM) // init_index:5
#define WHEELS_MOTORS_INDEX  CHASSIS_MOTORS_INDEX
#define SPECIAL_MOTORS_INDEX CHASSIS_MOTORS_INDEX + WHEELS_NUM
/*--------------------------------*/
#define ARM_MOTORS_HCAN      hcan1
#define ARM_MOTORS_NUM       0
#define ARM_MOTORS_INDEX     (CHASSIS_MOTORS_INDEX + CHASSIS_MOTORS_NUM)
/*--------------------------------*/
#define TEST_MOTORS_HCAN     hcan1
#if USE_TEST_FILE
#define TEST_MOTORS_NUM 1
#else
#define TEST_MOTORS_NUM 0
#endif
#define TEST_MOTORS_INDEX    (ARM_MOTORS_INDEX + ARM_MOTORS_NUM) // init_index:9
/*--------------------------------*/
#define NUM_OF_RM_QUAD_MOTOR 1

#define NUM_OF_LK_MOTOR      1
#define NUM_OF_LK_QUAD_MOTOR 1

#define NUM_OF_DM_MOTOR      1
#define NUM_OF_DM_QUAD_MOTOR 1

#define NUM_OF_UT_MOTOR      0

#define NUM_OF_ALL_MOTOR                                                               \
    (NUM_OF_RM_QUAD_MOTOR + NUM_OF_LK_MOTOR + NUM_OF_LK_QUAD_MOTOR + NUM_OF_DM_MOTOR + \
     NUM_OF_DM_QUAD_MOTOR + NUM_OF_UT_MOTOR)


typedef union {
   struct {
    //INDEX 1-1
    motor_t pitch;    // pitch电机
    motor_t yaw;      // yaw

    //INDEX 3-1
    motor_t R_fric;   // 右摩擦轮
    motor_t L_fric;   // 左摩擦轮
    motor_t dial;     // 拨弹盘

    //INDEX 6-1
    motor_t R_wheel;  // 右轮
    motor_t L_wheel;  // 左轮
    
    motor_t LF_hip;  // 左前方髋关节电机
    motor_t RF_hip;  // 右前方髋关节电机
    motor_t LB_hip;  // 左后方髋关节电机
    motor_t RB_hip;  // 右后方髋关节电机

#if USE_TEST_FILE
    motor_t test; // 测试用电机
#endif
  };
  motor_t _[NUM_OF_ALL_MOTOR]; // 方便遍历用
  // 需要在上面写好不同类型的电机数量 否则NUM_OF_ALL_MOTOR将不会正常工作
  // 不存在的电机要注释掉
} motors_t;
#endif
#if 0
    _    ____   _____     _______ 
   / \  | __ ) / _ \ \   / / ____|
  / _ \ |  _ \| | | \ \ / /|  _|  
 / ___ \| |_) | |_| |\ V / | |___ 
/_/   \_\____/ \___/  \_/  |_____|

 __  __  ___ _____ ___  ____        ____ ___  _   _ _____ 
|  \/  |/ _ \_   _/ _ \|  _ \      / ___/ _ \| \ | |  ___|
| |\/| | | | || || | | | |_) |    | |  | | | |  \| | |_   
| |  | | |_| || || |_| |  _ <     | |__| |_| | |\  |  _|  
|_|  |_|\___/ |_| \___/|_| \_\     \____\___/|_| \_|_|
#endif

#if USE_TEST_FILE
#warning "Test Motor is enabled!"
#endif
#if NUM_OF_ALL_MOTOR != GIMBAL_MOTORS_NUM + SHOOTER_MOTORS_NUM + CHASSIS_MOTORS_NUM + \
                                ARM_MOTORS_NUM + TEST_MOTORS_NUM
#error "NUM_OF_ALL_MOTOR is not correct!"
// 此项报错 则说明机器人 应启用电机个数 与 实际存在电机数量 不匹配
#endif

/*-----------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif
