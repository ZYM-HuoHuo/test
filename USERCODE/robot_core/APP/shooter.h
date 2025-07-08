#ifndef SHOOTER_H
#define SHOOTER_H

#include "drv_conf.h"
#include HAL_INCLUDE
#include "behaviour.h"
#include "Base/rc.h"
#include "Devices/MOTOR/motor_headers.h"

#if 0
#define GUUUUUUUUUUUUUUUUUUUUUUUUUUUU
#define shooter_init shooter42_init
#define shooter_ctrl_loop(_rc_,_probot_) shooter42_ctrl_loop(_rc_,_probot_)

#define DIAL_RR (1.f)

#define STUCK_FORWARD -1

#define DIAL_RPM 135.f // 135 20Hz
#define DIAL_TEST_RPM 135.f

#define FRIC_RPM 6350.f
#define FRIC_TEST_RPM 2000.f

#define SHOOT_SPEED_DEC 0.90f

#define ONE_SHOOT_ANG -0.77338f //调参

#define MIN_SHOOT_HEAT 30
#define MIN_ONE_SHOOT_HEAT 90

#else 
#define shooter_init() shooter17_init()
#define shooter_ctrl_loop(_rc_,_probot_) shooter17_ctrl_loop(_rc_,_probot_)

#define DIAL_RR (2.25f/1.f)

#define C_FRIC_WHEEL (0.06f * PI)
#define C_PUSHER_WHEEL (0.08f * PI)

#define DEFAULT_SHOOT_SPEED 15.0f
#define MAX_SHOOT_SPEED 30.0f

#define STUCK_MAX_TIME_CNT 10 // 600//堵转时间
#define STUCK_SPEED 5
#define STUCK_RESTORE_RPM -500
#define STUCK_REVERSE_TIME_CNT 60
#define STUCK_CURRENT 5.0f // 安培
#define STUCK_FORWARD -1
#define STUCK_REVERSE 1

#define S_CURVE_DIAL_ACC 0.5f

#define DIAL_RPM 135.f // 135 20Hz
#define DIAL_TEST_RPM 135.f

#define FRIC_RPM 6350.f // 不带减速箱的rpm
#define FRIC_TEST_RPM 4000.f

#define ONE_SHOOT_ANG (0.029187724f-1.35870194f) //调参 

#define MIN_SHOOT_HEAT 30
#define MIN_ONE_SHOOT_HEAT 90

void anti_stuck_output(shooter_tag_t tag, float curr, float cur_rpm, float exp_rpm);

#endif

void shooter42_init(void);
HAL_StatusTypeDef shooter42_ctrl_loop(rc_ctrl_t* rc_recv,robot_t *robot);

void shooter17_init(void);
HAL_StatusTypeDef shooter17_ctrl_loop(rc_ctrl_t* rc_recv,robot_t *robot);

uint8_t get_single_shooting_flag(void);
uint8_t *get_heat_enable_flag_ptr(void);

#endif
