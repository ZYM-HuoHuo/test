/**
 * @file shooter17.c
 *
 * @brief 17mm发射机构中间层+底层
 *
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#include "shooter.h"

#include "Base/bsp_dwt.h"
#include "Base/referee.h"
#include "Devices/MOTOR/motor_headers.h"
#include "algorithm/KF.h"
#include "algorithm/filter.h"
#include "algorithm/pid.h"
#include "bus_detect.h"
#include "drv_conf.h"
#include "fire_ctrl.h"
#include "pid_lpf_param.h"
#include "tools/asr.h"
#include "vision.h"


#include <math.h>
#include <stdlib.h>
#include <string.h>

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern uint8_t suggest_fire;
extern ext_game_robot_status_t game_robot_status;
extern ext_power_heat_data_t power_heat_data;

/*发射机构计时*/
uint32_t shooter_dwt_cnt = 0;
float shooter_dt = 1.f / SHOOTER_TIM_FREQ;

/*单发 控制目标*/
uint16_t single_shoot_wait_cnt = 0;
uint16_t single_shoot_wait_cnt_max = 100; // 单发角度稳定等待时间
float one_shoot_ang = ONE_SHOOT_ANG;
float delta_dial_ang = 0;
float fix_dial_ang = 0;
uint8_t single_shooting_flag = FINISH;
float cur_dial_rpm = 0, cur_dial_ang = 0;

/*热量允许标志*/
uint8_t heat_enable_flag = 0;

/*防堵转*/
int8_t stuck_dir = STUCK_FORWARD;
RAM_PERSIST uint16_t dial_stuck_cnt;

/*pid结构体*/
pid_struct_t fric_pid[2];
pid_struct_t dial_rpm_pid; // 连发对速度PID
pid_struct_t dial_ang_pid; // 单发对角度PID

/*固定速度设置*/
float set_fric_rpm = FRIC_TEST_RPM; // 摩擦轮设定(rpm)
float set_dial_rpm = DIAL_RPM;      // 推弹速度设定(rpm)
float exp_dial_rpm = 0;             // DT7拨轮给的期望

/*电机输出编码值*/
float fric_output[2] = {0};
float dial_output = 0;

/*输入计数值*/
RAM_PERSIST float dial_cnt = 0;

/**
 * @brief 发射机构初始化
 * @retval None
 */
void shooter17_init(void) {
  motors_t *motors = get_motors_ptr();

  memcpy(&fric_pid[LEFT], &shoot_left_fric_pid_init_val, sizeof(pid_struct_t));
  memcpy(&fric_pid[RIGHT], &shoot_right_fric_pid_init_val,
         sizeof(pid_struct_t));

  memcpy(&dial_rpm_pid, &dial_rpm_pid_init_val, sizeof(pid_struct_t));
  memcpy(&dial_ang_pid, &dial_ang_pid_init_val, sizeof(pid_struct_t));

  //  CAN2
  //  ID  rxId    txId
  //  1   0x201   0x200
  RM_QUAD_motor_register(&motors->L_fric, motor_model_RM_QUAD_M3508, QUAD_CURR,
                         COM_CAN, (uint32_t *)&SHOOTER_MOTORS_HCAN, 1,
                         RR_BARE_M3508, &L_fric_lpf_init_val, M_SHOOTER);
  //  2   0x202   0x200
  RM_QUAD_motor_register(&motors->R_fric, motor_model_RM_QUAD_M3508, QUAD_CURR,
                         COM_CAN, (uint32_t *)&SHOOTER_MOTORS_HCAN, 2,
                         RR_BARE_M3508, &R_fric_lpf_init_val, M_SHOOTER);
  //  3   0x203   0x200
  RM_QUAD_motor_register(&motors->dial, motor_model_RM_QUAD_M2006, QUAD_CURR,
                         COM_CAN, (uint32_t *)&SHOOTER_MOTORS_HCAN, 3, RR_M2006,
                         &dial_lpf_init_val, M_SHOOTER);
  // 发射机构电机 放在fifo1
  can_user_init(&SHOOTER_MOTORS_HCAN,
                (uint32_t[]){
                    rx_stdid_of(motors->L_fric),
                    rx_stdid_of(motors->R_fric),
                    rx_stdid_of(motors->dial),
                    0,
                },
                1);
  dial_stuck_cnt = 0;
}

float cur_fric_rpm[2] = {0};
/**
 * @brief 17mm弹丸发射机构控制函数
 * @note 不存在base_shooter.c 这个函数本身即是base
 * @note 扬掉了原来的set_multi_shoot_output 避免非正常使用导致的混乱
 * @param rc_recv 遥控指令结构体指针
 * @param robot 机器人结构体指针
 * @return HAL_StatusTypeDef
 *          HAL_StatusTypeDef:
 *          @arg HAL_OK 进程正常
 *          @arg HAL_ERROR 机构电机掉线
 */
HAL_StatusTypeDef shooter17_ctrl_loop(rc_ctrl_t *rc_recv, robot_t *robot) {
  shooter_dt = DWT_GetDeltaT(&shooter_dwt_cnt);
  if (!shooter_dt)
    shooter_dt = 1.f / SHOOTER_TIM_FREQ;
  motors_t *motors = get_motors_ptr();

  cur_fric_rpm[LEFT] =
      fir_filter_5(motors->L_fric.real.rpm * RR_of(motors->L_fric),
                   SHOOT_LEFT_FILTER2_RPM_CH);
  cur_fric_rpm[RIGHT] =
      fir_filter_5(motors->R_fric.real.rpm * RR_of(motors->R_fric),
                   SHOOT_RIGHT_FILTER2_RPM_CH);

  cur_dial_rpm = motors->dial.real.rpm / DIAL_RR;
  cur_dial_ang = motors->dial.real.abs_angle;

  if (robot->robot_control_flag == DISABLE) {
    pid_reset(&fric_pid[LEFT]);
    pid_reset(&fric_pid[RIGHT]);
    pid_reset(&dial_rpm_pid);
    pid_reset(&dial_ang_pid);
    memset(fric_output, 0, sizeof(fric_output));
    dial_output = 0;
    exp_dial_rpm = 0;
    // return shutdown_all_motor();
  } else {
    rc_ctrl_t *dt7_data = get_dt7_data_ptr();
    if (dt7_data) // 防wheel编码器损坏为0
      dial_cnt +=
          CLAMP((float)(dt7_data->wheel - 1024) * T_ACC_CNT / 660.0f - dial_cnt,
                S_CURVE_DIAL_ACC);
#if USE_DT7_WHEEL
    if (robot->use_keyboard_flag)
      exp_dial_rpm = DIAL_RPM;
    else
      exp_dial_rpm = fabsf(s_curve(DIAL_TEST_RPM, dial_cnt));
#else
    exp_dial_rpm = DIAL_RPM;
#endif

    if (is_motors_offline(SHOOTER_MOTORS))
      return HAL_ERROR;

    int16_t remaining_heat = game_robot_status.shooter_barrel_heat_limit -
                             power_heat_data.shooter_id1_17mm_cooling_heat;
    if (remaining_heat > MIN_SHOOT_HEAT ||
        game_robot_status.shooter_barrel_heat_limit == 65535 ||
        game_robot_status.shooter_barrel_heat_limit == 0)
      heat_enable_flag = ENABLE;
    else
      heat_enable_flag = DISABLE;
    float zero_number = 0;
    if (robot->use_fric_flag == ENABLE){
			float fric_rpm = -FRIC_RPM; 
      slope_following(&fric_rpm, &set_fric_rpm, 1);
    
    }else {
      // set_fric_rpm = 0;
      slope_following(&zero_number, &set_fric_rpm, 2); // 此值决定停转速度
    }

    fric_output[LEFT] =
        pid_calc(&fric_pid[LEFT], set_fric_rpm, cur_fric_rpm[LEFT]) * Kn_M3508 *
        C620_CURR_MAX / C620_CURR_DATA_MAX;
    fric_output[RIGHT] =
        pid_calc(&fric_pid[RIGHT], -set_fric_rpm, cur_fric_rpm[RIGHT]) *
        Kn_M3508 * C620_CURR_MAX / C620_CURR_DATA_MAX;
    static bool lock_single_shoot = true;

    switch (robot->shooter_tag) {
    case S_STOP_MODE:
      single_shoot_wait_cnt = 0;
      lock_single_shoot = true;
      single_shooting_flag = FINISH;
      fric_output[LEFT] = fric_output[RIGHT] = 0;
      set_dial_rpm = 0;
      break;
    case S_MULTIPLE_MODE:
      if (robot->fire_flag == ENABLE)
        set_dial_rpm = exp_dial_rpm;
      else
        set_dial_rpm = 0;
      break;
    case S_RESTRICT_MODE:
      if (robot->fire_flag == ENABLE && suggest_fire && heat_enable_flag)
        set_dial_rpm = exp_dial_rpm;
      else
        set_dial_rpm = 0;
      break;
    case S_SINGLE_MODE:
      // 这里可以再研究一下 写得更精炼些
      if (robot->fire_flag == ENABLE && suggest_fire && heat_enable_flag &&
          lock_single_shoot) {
        single_shooting_flag = UNFINISH;
        fix_dial_ang = cur_dial_ang;
        lock_single_shoot = false; // 防止重复触发
      }
      if (single_shoot_wait_cnt > single_shoot_wait_cnt_max) {
        single_shoot_wait_cnt = 0;
        single_shooting_flag = FINISH;
        lock_single_shoot = true;
      }
      if (single_shooting_flag == UNFINISH) {
        delta_dial_ang = cur_dial_ang - fix_dial_ang;
        if (fabsf(delta_dial_ang - one_shoot_ang) < 0.01f)
          single_shoot_wait_cnt++;
        else
          single_shoot_wait_cnt =
              single_shoot_wait_cnt < 1 ? 0 : single_shoot_wait_cnt--;
      } else
        delta_dial_ang = one_shoot_ang;
      dial_output =
          pid_calc(&dial_ang_pid, one_shoot_ang, delta_dial_ang); // 输出torque
      break;
    default:
      break;
    }
    anti_stuck_output(robot->shooter_tag, motors->dial.real.current,
                      cur_dial_rpm, set_dial_rpm);

    // memset(fric_output,0,sizeof(fric_output));  // debug
    // memset(&dial_output,0,sizeof(float));       // debug

    LOAD_MOTOR_TFF(motors->L_fric, fric_output[LEFT]);
    LOAD_MOTOR_TFF(motors->R_fric, fric_output[RIGHT]);
    LOAD_MOTOR_TFF(motors->dial, dial_output);

        motors->L_fric.AUX.EN = 1;/////////////////////
        motors->R_fric.AUX.EN = 0;///////////////
        motors->dial.AUX.EN = true;//////////////////
        
    return set_all_motor_output(SHOOTER_MOTORS);
  }
  return HAL_OK;
}

/**
 * @brief 防堵转处理函数
 * @brief 还可以再优化 目前对单发不启用
 * @retval None
 */
void anti_stuck_output(shooter_tag_t tag, float curr, float cur_rpm,
                       float exp_rpm) {
  float output = 0;
  if (fabsf(exp_rpm) > STUCK_SPEED) {
    if (fabsf(cur_rpm) < STUCK_SPEED && fabsf(curr) > STUCK_CURRENT) {
      // 检测到 电流过大
      if (dial_stuck_cnt < STUCK_MAX_TIME_CNT)
        // 检测到电流过大一段时间->>>>堵转
        dial_stuck_cnt++;
      else {
        asr_shooter_stuck_cmd();
        // 反转不了就正转 正转不了就反转 直到恢复顺畅 dial_stuck_cnt 变回0
        dial_stuck_cnt = STUCK_REVERSE_TIME_CNT + STUCK_MAX_TIME_CNT;
        if (stuck_dir == STUCK_FORWARD) // 前时刻为正转
          stuck_dir = STUCK_REVERSE;
        else if (stuck_dir == STUCK_REVERSE)
          stuck_dir = STUCK_FORWARD;
      }
    } else if (dial_stuck_cnt <= STUCK_MAX_TIME_CNT) {
      // 表示恢复顺畅
      dial_stuck_cnt = 0;
      stuck_dir = STUCK_FORWARD;
    }
    if (dial_stuck_cnt > STUCK_MAX_TIME_CNT) {
      exp_rpm = stuck_dir * STUCK_RESTORE_RPM;
      // STUCK_REVERSE_RPM * sign(sinf((pusher_stuck_cnt - STUCK_MAX_TIME_CNT) /
      // STUCK_REVERSE_TIME_CNT * (3 * PI))); TODO:平滑过渡消除阶跃
      dial_stuck_cnt--;
    }
    output = pid_calc(&dial_rpm_pid, -exp_rpm, cur_rpm);
  } else
    output = pid_calc(&dial_rpm_pid, 0, cur_rpm);
  // 非单发模式就传值
  if (tag != S_SINGLE_MODE)
    dial_output = output;
}

/**
 * @brief 访问单发是否完成标志位
 * @retval uint8_t
 *         single_shooting_flag
 *         @arg FINISH 单发完成
 *         @arg UNFINISH 单发未完成
 */
uint8_t get_single_shooting_flag(void) { return single_shooting_flag; }
/**
 * @brief 获取热量是否允许发射标志位指针
 */
uint8_t *get_heat_enable_flag_ptr(void) { return &heat_enable_flag; }
