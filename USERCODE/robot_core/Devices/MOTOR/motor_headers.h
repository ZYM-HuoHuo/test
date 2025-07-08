/**
 * @file motor_headers.h
 * @brief Motor headers
 *
 * @note 已收录电机品牌：RM(RoboMaster)、LK(瓴控)、DM(达妙)、UT(宇树)
 * @note 部分参数需要配合drv_conf.h存在
 * @details 目前收录电机支持一拖四控制 MIT控制 位置速度控制 速度控制
 * @attention motor.h文件中有多配置需要用户自定义
 *
 * @version 1.2.8.3
 *
 *
 * @version Version 0.1 - Initial Release
 * @author zzq
 * @date 2024/03
 *
 * @version Version 1.2.8.3 - Feature Update
 * @author MYUIN & yjy
 * @date 2024/12/07
 * - tp-> https://scnu-pioneer.coding.net/p/ec/d/MOTOR/git
 *
 * @copyright (c) 2024-2025 SCNU-PIONEER, MYUIN. All rights reserved.
 */

#ifndef MOTOR_HEADERS_H_
#define MOTOR_HEADERS_H_

#include "./motor_conf.h"
#include "./motor_def.h"


#ifndef MOTOR_CONF_FILE
#error "motor_conf.h not exists, please add motor_conf.h to your project orz"
#endif // !MOTOR_CONF_FILE

#include "./motor_ctrl.h"
#include "./motor_lib/dev_DM.h"
#include "./motor_lib/dev_LK.h"
#include "./motor_lib/dev_RM.h"
#include "./motor_lib/dev_UT.h"
#include "./motor_utils/motor_math.h"


// 调用电机库只需在文件中include本头文件即可

#endif // !MOTOR_HEADERS_H_
