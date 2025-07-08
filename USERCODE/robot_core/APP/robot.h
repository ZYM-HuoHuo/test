#ifndef _ROBOT_H
#define _ROBOT_H

#include "drv_conf.h"
#include "main.h"
#include "usbd_def.h"

#include "Base/rc.h"
#include "Base/referee.h"
#include "Base/ext_imu.h"
#include "Base/drv_can.h"

#include "Devices/MOTOR/motor_headers.h"

#include "arm_math.h"

#include "algorithm/pid.h"
#include "algorithm/pid_ladrc.h"
#include "algorithm/pid_leso.h"
#include "algorithm/filter.h"

#include "tools/ws2812.h"
#include "tools/flash.h"
#include "tools/vofa.h"
#include "tools/asr.h"
#include "tools/buzzer.h"
#include "tools/led.h"

#include "APP/behaviour.h"
#include "APP/gimbal.h"
#include "APP/shooter.h"
#include "APP/vision.h"
#include "APP/bus_detect.h"
#include "APP/imu_bmi088.h"
#include "APP/graphic_draw.h"

#include "tests/test.h"

// typedef struct{
//     void (*init)(void);
//     void (*control)(void);
// }chassis_t;
// typedef struct{
//     void (*init)(void);
//     void (*control)(void);
// }gimbal_t;
// typedef struct{
//     void (*init)(void);
//     void (*control)(void);
// }shooter_t;

void robot_init(void);
robot_t *get_robot_ptr(void);

void diag_slow_time_cycle(void);
void robot_m_tim_cycle(void);
void robot_h_tim_cycle(void);
void robot_imu_tim_cycle(void);
void robot_main_loop(void);
void robot_delay_us(uint32_t us);

#endif  //_ROBOT_TASK_H
