#ifndef _TEST_H
#define _TEST_H
#include "Base/rc.h"
#include "APP/robot.h"

void motor_test_init(void);
HAL_StatusTypeDef motor_test_loop(rc_ctrl_t *rc_recv, robot_t *robot);

#endif // !_TEST_H
