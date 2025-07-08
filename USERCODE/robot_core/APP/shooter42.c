/**
 * @file shooter42.c
*
 * @brief 42mm发射机构中间层+底层
 *
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#include "shooter.h"

#include "Devices/MOTOR/motor_headers.h"
#include "drv_conf.h"
#include "Base/referee.h"
#include "algorithm/pid.h"
#include "algorithm/filter.h"
#include "algorithm/KF.h"
#include "bus_detect.h"
#include "vision.h"
#include "pid_lpf_param.h"
#include "fire_ctrl.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void shooter42_init(void){
    ;
}
HAL_StatusTypeDef shooter42_ctrl_loop(rc_ctrl_t *rc_recv, robot_t *robot){
    return HAL_OK;
}

