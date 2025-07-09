/**
 * @file gimbal.c
*
 * @brief 云台中间层
 * @version 0.2
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 * 
 * 调云台前须知:
 * 要求yaw轴顺时针转时 yaw_rad逐渐减小
 * 要求pitch轴顺时针转时(即低头时) pitch_rad逐渐增大
 * 因此反装电机时计算rad需要给abs_angle加负号 否则将影响底盘换系! pitch同理
 * 电机正反装会影响get_minor_arc的结果 这里要求减正数得到小角度,减负数得到大角度,反了的话就改get_minor_arc两角顺序
 * 框架欧拉角规定 抬头pitch减小	顺时针yaw减小 车头向前顺时针roll增大
 */

#include "gimbal.h"

#include "drv_conf.h"
#include "Base/ext_imu.h"
#include "Devices/MOTOR/motor_headers.h"
#include "Base/base_gimbal.h"
#include "Base/bsp_dwt.h"

#include "algorithm/pid.h"
#include "algorithm/filter.h"
#include "algorithm/KF.h"
#include "algorithm/imu_fusion.h"

#include "bus_detect.h"
#include "pid_lpf_param.h"
#include "behaviour.h"
#include "vision.h"

#include <stdlib.h>
#include <math.h>
#include <string.h>

#define GIMBAL_USE_DM_MOTOR 1
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;


extern vision_ctrl_t vision;
extern float chassis_pitch;
extern uint8_t need_change_yaw_center;

/*云台机构计时*/
uint32_t gimbal_dwt_cnt = 0;
float gimbal_dt = 1.f/GIMBAL_TIM_FREQ;

/*云台角限制变值*/
float pitch_rad_max = PITCH_REL_RAD_MAX;
float pitch_rad_min = PITCH_REL_RAD_MIN;
float pitch_imu_max = PITCH_REL_RAD_MAX;
float pitch_imu_min = PITCH_REL_RAD_MIN;
float yaw_rad_max = YAW_REL_RAD_MAX;
float yaw_rad_min = YAW_REL_RAD_MIN;
float yaw_imu_max = YAW_REL_RAD_MAX;
float yaw_imu_min = YAW_REL_RAD_MIN;
float yaw_res_cpst = YAW_RES_CPST;
float pitch_res_cpst =
    PITCH_RES_CPST; // 本质是一个死区，用于补偿pitch响应不完美导致的卡顿
// 全名pitch_response_compensation 与pitch响应 pitch前馈 imu稳定性有关

/*pitch和yaw轴0值*/
float pitch_rad_center = 0;
float yaw_rad_center = 0;

// 上电摆好角度再上控 缓启动成功即可确认本次启动的零角度
// 本次缓启动被称为"真·缓启动"
uint8_t is_confirm_0degree = 0; 

/*pitch和yaw在车体坐标系的角度 以各自center作为零点 弧度制*/
float pitch_rad = 0; // 要求pitch轴顺时针转时 pitch_rad逐渐增大
float yaw_rad = 0;   // 要求yaw轴顺时针转时 yaw_rad逐渐减小
float roll_rad = 0;  // 暂时用不上

/*自瞄相关*/
float echo_kp = 0.3f; // 相应补偿
uint8_t change_yaw_enable = 0;
float e_yaw_prev = 0; // e前缀仅代表娱乐性瞄人相关
float e_yaw = 0;

/*云台电机系 IMU系标志*/
gimbal_ctrl_type_t gimbal_ctrl_type = USE_MOTOR;
/**
 * @brief 云台选择控制坐标系
 */
static gimbal_ctrl_type_t
select_gimbal_ctrl_type(robot_t *robot, gimbal_reseting_state_t reset_state) {
  // 选择控制模式
  motors_t *motors = get_motors_ptr();
  robot_state_t *cur_state = robot->current_state;
  if (reset_state == RESETED) {
#if 1
    cur_state->pitch_ang = robot->imu_data->eular.pitch;
    cur_state->roll_ang = robot->imu_data->eular.roll;
    cur_state->yaw_ang = robot->imu_data->eular.yaw;
    return USE_IMU;
#else
    // 测试电机系常驻效果 注意极性要对应yaw_rad pitch_rad一致
    cur_state->pitch_ang = motors->pitch.real.abs_angle / PITCH_RR;
    cur_state->roll_ang = robot->imu_data->eular.roll / IMU_RANGE * 2*PI;
    cur_state->yaw_ang = -motors->yaw.real.abs_angle / YAW_RR;
    return USE_MOTOR;
#endif
    } else if(reset_state == RESETING){
     cur_state->pitch_ang = pitch_rad;
     cur_state->roll_ang = robot->imu_data->eular.roll / IMU_RANGE * 2*PI;
     cur_state->yaw_ang = yaw_rad;
     return USE_MOTOR;
    } else
    return USE_MOTOR;
}
/*云台缓启动标志*/
gimbal_reseting_state_t gimbal_reset_state  = G_UNRESET;
uint8_t gimbal_reset_flag = UNFINISH;
uint16_t gimbal_launch_time_cnt = 0; // 观测此变量来判断缓启动是否超时

uint8_t debug_cnt = 0;
/**
 * @brief 云台缓启动解算函数
 * @attention 复位的第一次缓启动结束的位置决定了机器人0点 故要保证摆好方向再上电启动
 * @note is_confirm_0degeree 决定了缓启动是以电机中心编码值作为参考 还是以云台rad中心作为参考 会被need_change_yaw_center抢占
 * @param exp_state 机器人期望状态指针
 * @param pmotors 电机集指针
 * @retval gimbal_reset_state 返回缓启动状态标识
 */
float cur_delta_ang_pitch = 0, cur_delta_ang_yaw = 0;
float exp_delta_ang_pitch = 0, exp_delta_ang_yaw = 0;
gimbal_reseting_state_t gimbal_smooth_reseting(robot_state_t *exp_state,motors_t *pmotors){
    if(is_motors_offline(GIMBAL_MOTORS))
        return RESETED;
    float soft_start_deadband = 0;
    motor_t *pitch_pmotor = &pmotors->pitch;
    motor_t *yaw_pmotor = &pmotors->yaw;
    if(need_change_yaw_center)is_confirm_0degree = 0;

    if(gimbal_reset_state == RESETING){
       if(is_confirm_0degree){ // 已确认零点
            exp_delta_ang_yaw = get_minor_arc(range_map(yaw_rad_center,0,2*PI), exp_state->yaw_ang, 2*PI);
            cur_delta_ang_yaw = get_minor_arc(range_map(yaw_rad_center,0,2*PI), yaw_rad, 2*PI);
            exp_delta_ang_pitch = get_minor_arc(pitch_rad_center, exp_state->pitch_ang, 2*PI);
            cur_delta_ang_pitch = get_minor_arc(pitch_rad_center, pitch_rad, 2*PI);
            exp_state->pitch_ang += CLAMP(exp_delta_ang_pitch, SOFT_START_SPEED_RAD);
            //exp_state->pitch_ang += 0;
            exp_state->yaw_ang += CLAMP(exp_delta_ang_yaw, SOFT_START_SPEED_RAD);
            soft_start_deadband = SOFT_START_DEADBAND_RAD;
        }
        else{
            if(need_change_yaw_center)
                exp_delta_ang_yaw = get_major_arc(GIMBAL_YAW_MOTOR_CENTER, exp_state->yaw_ang, span_of(*yaw_pmotor));
            else
                exp_delta_ang_yaw = get_minor_arc(GIMBAL_YAW_MOTOR_CENTER, exp_state->yaw_ang, span_of(*yaw_pmotor));
            cur_delta_ang_yaw = get_minor_arc(GIMBAL_YAW_MOTOR_CENTER, yaw_pmotor->real.raw_scale, span_of(*yaw_pmotor));
            exp_delta_ang_pitch = get_minor_arc(GIMBAL_PITCH_MOTOR_CENTER, exp_state->pitch_ang, span_of(*pitch_pmotor));
            cur_delta_ang_pitch = get_minor_arc(GIMBAL_PITCH_MOTOR_CENTER, pitch_pmotor->real.raw_scale, span_of(*pitch_pmotor));
             exp_state->pitch_ang += CLAMP(exp_delta_ang_pitch, SOFT_START_SPEED_MOTOR);
           // exp_state->pitch_ang += 0;
            exp_state->yaw_ang += CLAMP(exp_delta_ang_yaw, SOFT_START_SPEED_MOTOR);
            soft_start_deadband = SOFT_START_DEADBAND_MOTOR;
        }
        if(gimbal_launch_time_cnt < SOFT_START_MAX_CNT){
            gimbal_launch_time_cnt++;
            if (fabsf(cur_delta_ang_pitch) <= soft_start_deadband && 
                fabsf(cur_delta_ang_yaw) <= soft_start_deadband){
                
                if(!is_confirm_0degree){// 设置电机0点
                
                    set_motor_zero_angle(yaw_pmotor,yaw_pmotor->real.raw_scale);  
                    pitch_pmotor->AUX.xout.dat += pitch_pmotor->real.abs_angle;  
                    set_motor_zero_angle(pitch_pmotor,pitch_pmotor->real.raw_scale);
                    if(!pitch_pmotor->AUX.xout.read_flag){
                       // read_single_DM_motor_xout(pitch_pmotor,COM_CAN)
						;
                        while(!pitch_pmotor->AUX.xout.read_flag){
                           // read_single_DM_motor_xout(pitch_pmotor,COM_CAN);
                            robot_delay_us(5000);
                        }
                    }

                    set_motor_zero_angle(yaw_pmotor,range_map(yaw_pmotor->real.raw_scale,0,span_of(*yaw_pmotor)));
                    yaw_rad_center = range_map(yaw_pmotor->real.rel_angle,0,2*PI);
                    pitch_rad_center = range_map(pitch_pmotor->real.rel_angle,0,2*PI); // 时序理想的情况为0
                }
                need_change_yaw_center = 0;
                is_confirm_0degree = 1;
                asr_init_finish_cmd();
                return RESETED; // 缓启动完成
            }
            else
                return RESETED; // 缓启动未完成 还在缓启动中
        }
        else{
            pitch_pmotor->AUX.xout.dat += pitch_pmotor->real.abs_angle;  
            set_motor_zero_angle(pitch_pmotor,pitch_pmotor->real.raw_scale);
            asr_gimbal_launch_error_cmd();// todo
            need_change_yaw_center = 0;
            is_confirm_0degree = 0;
            return RESETED; // 缓启动超时 可能是电机被卡住
        }
    }
    else if(gimbal_reset_state == G_UNRESET || need_change_yaw_center){
        gimbal_launch_time_cnt = 0;
        // 共量程开始
        if(is_confirm_0degree){
            exp_state->pitch_ang = pitch_rad;
            exp_state->yaw_ang = yaw_rad;
        }
        else{
            exp_state->pitch_ang = (float)pitch_pmotor->real.raw_scale;
            exp_state->yaw_ang = (float)yaw_pmotor->real.raw_scale;
        }
        return RESETED;
    }
    else /*(gimbal_reset_state == RESETED)*/
		return RESETED;

}

/*快速掉头相关设置*/
static uint8_t turnable_flag = DISABLE; //状态限位
static uint8_t turn_cnt = 0; //延时限位

/*pid结构体*/
pid_struct_t pitch_pid[2], yaw_pid[2];
pid_struct_t pitch_imu_pid[2], yaw_imu_pid[2];
pid_struct_t pitch_imu_vision_pid[2], yaw_imu_vision_pid[2];

/*观测器参数结构体*/
leso_para_t pitch_imu_leso, yaw_imu_leso;

/*一维卡尔曼滤波结构体*/
one_vec_kf_t kalman_pitch_filter, kalman_yaw_filter;

/*输入计数值*/
extern RAM_PERSIST float rocker_lx;
extern RAM_PERSIST float rocker_ly; 
/**
 * @brief 云台状态清零函数
 */
static void gimbal_state_clear(robot_t *robot) {
  for (int i = 0; i < 2; i++) {
    pid_reset(&pitch_pid[i]);
    pid_reset(&yaw_pid[i]);
    pid_reset(&pitch_imu_pid[i]);
    pid_reset(&yaw_imu_pid[i]);
    pid_reset(&pitch_imu_vision_pid[i]);
    pid_reset(&yaw_imu_vision_pid[i]);
  }

  robot->expected_state->pitch_ang = robot->current_state->pitch_ang;
  robot->expected_state->yaw_ang = robot->current_state->yaw_ang;

  gimbal_reset_state = G_UNRESET;
  gimbal_reset_flag = UNFINISH;

  gimbal_launch_time_cnt = 0;
}
/**
 * @brief 云台机构初始化函数
 * @retval None
 */
void gimbal_init(void) {
  motors_t *motors = get_motors_ptr();

  memcpy(&pitch_pid[ANG_LOOP], &pitch_pid_ang_loop, sizeof(pid_struct_t));
  memcpy(&pitch_pid[RPM_LOOP], &pitch_pid_rpm_loop, sizeof(pid_struct_t));

  memcpy(&yaw_pid[ANG_LOOP], &yaw_pid_ang_loop, sizeof(pid_struct_t));
  memcpy(&yaw_pid[RPM_LOOP], &yaw_pid_rpm_loop, sizeof(pid_struct_t));

  memcpy(&pitch_imu_pid[ANG_LOOP], &pitch_imu_pid_ang_loop,
         sizeof(pid_struct_t));
  memcpy(&pitch_imu_pid[RPM_LOOP], &pitch_imu_pid_rpm_loop,
         sizeof(pid_struct_t));

  memcpy(&yaw_imu_pid[ANG_LOOP], &yaw_imu_pid_ang_loop, sizeof(pid_struct_t));
  memcpy(&yaw_imu_pid[RPM_LOOP], &yaw_imu_pid_rpm_loop, sizeof(pid_struct_t));

  memcpy(&yaw_imu_vision_pid[ANG_LOOP], &yaw_imu_pid_ang_vision_loop,
         sizeof(pid_struct_t));
  memcpy(&yaw_imu_vision_pid[RPM_LOOP], &yaw_imu_pid_rpm_vision_loop,
         sizeof(pid_struct_t));

  leso_6020_init(&pitch_imu_leso, 0.2f, 5.0f, 1.5f);
  pitch_imu_leso.af_z1 = 0.05f;
  leso_6020_init(&yaw_imu_leso, 0.2f, 5.0f, 1.5f);
  yaw_imu_leso.af_z1 = 0.05f;

  kalman1_init(&kalman_pitch_filter, 0, 0.1, 60);
  kalman1_init(&kalman_yaw_filter, 0, 0.1, 60);
#if (BOARDS_MODE && MACHINE_TYPE == GIMBAL_MASTER) || !BOARDS_MODE
  //  CAN1
  //  ID  rxId    txId
  //  1   0xA0+1  0x0A+1
//注册电机
 /// DM_motor_register(&motors->pitch,motor_model_DM_DM4310,PDESVDES,
    ///                COM_CAN, (uint32_t *)&GIMBAL_MOTORS_HCAN, 1, RR_DM4310,
     ///               &pitch_lpf_init_val, M_GIMBAL);

  //  2   0x206   0x1FF
  RM_QUAD_motor_register(&motors->yaw, motor_model_RM_QUAD_GM6020, QUAD_CURR,
                         COM_CAN, (uint32_t *)&GIMBAL_MOTORS_HCAN, 3, RR_GM6020,
                         &yaw_lpf_init_val, M_GIMBAL);

  // 云台电机 放在fifo1
  can_user_init(
      &GIMBAL_MOTORS_HCAN,
      (uint32_t[]){rx_stdid_of(motors->pitch),
                   rx_stdid_of(motors->yaw),
                   0,
                   0
                   },1);

#if NUM_OF_DM_MOTOR && GIMBAL_USE_DM_MOTOR
  // 读取多圈角度 DM特供
  robot_delay_us(50000);
  clear_DM_motor_error(&GIMBAL_MOTORS_HCAN);
  robot_delay_us(50000);
  enable_DM_motor_output(&GIMBAL_MOTORS_HCAN);
  __enable_irq();
  if (!motors->pitch.AUX.xout.read_flag){
    read_single_DM_motor_xout(&motors->pitch,COM_CAN);
    while(!motors->pitch.AUX.xout.read_flag){
    read_single_DM_motor_xout(&motors->pitch,COM_CAN);
      robot_delay_us(5000);
    }
  }
  __disable_irq();
#endif
#endif

}
/**
 * @brief 云台控制函数
 * @param rc_recv 遥控器指令结构体指针
 * @param robot 机器人结构体指针
 * @retval None
 */
float min_debug = 0;
float max_debug = 0;
void gimbal_ctrl_loop(rc_ctrl_t *rc_recv, robot_t *robot){
    gimbal_dt = DWT_GetDeltaT(&gimbal_dwt_cnt);
    if(!gimbal_dt) gimbal_dt = 1.f/GIMBAL_TIM_FREQ;
    motors_t *motors = get_motors_ptr();
    robot_state_t *exp_state = robot->expected_state,
                  *cur_state = robot->current_state;
                  // IMU系控制时 cur_state随imu更新
                  // 电机系控制时 cur_state随电机相对弧度更新
    vision_ctrl_t *vision = get_vision_ctrl_data_ptr();
    float vision_pitch,vision_yaw;
    
    // 反装电机时计算rad需要给abs_angle加负号 否则将影响底盘换系! pitch同理
    pitch_rad = motors->pitch.real.abs_angle / PITCH_RR;
    yaw_rad = -motors->yaw.real.abs_angle / YAW_RR;
    pitch_rad = range_map(pitch_rad, 0, 2 * PI);
    if(pitch_rad < 0)pitch_rad += 2*PI;
    yaw_rad = range_map(yaw_rad, 0, 2 * PI);
    if(yaw_rad < 0)yaw_rad += 2*PI;

    gimbal_ctrl_type = select_gimbal_ctrl_type(robot, gimbal_reset_state);

    // 输入值映射到机体期望
    static float assemble_bias_pitch = 0.0f, assemble_bias_yaw = 0.f, vision_spin_bias = (10.0f); 
    if(gimbal_ctrl_type == USE_IMU){
        // 先做映射 再限角度
        // 抬头IMU PITCH减小 
        // !!!(电机正反装会影响get_minor_arc的结果 这里要求减正数得到小角度,减负数得到大角度,反了的话就改get_minor_arc两角顺序)!!!
        pitch_imu_min = range_map(cur_state->pitch_ang - get_minor_arc(pitch_rad,PITCH_REL_RAD_MIN,2*PI)/(2.f*PI)*IMU_RANGE,-IMU_RANGE/2,IMU_RANGE/2);
        pitch_imu_max = range_map(cur_state->pitch_ang - get_minor_arc(pitch_rad,PITCH_REL_RAD_MAX,2*PI)/(2.f*PI)*IMU_RANGE,-IMU_RANGE/2,IMU_RANGE/2);
        yaw_imu_min = range_map(cur_state->yaw_ang + get_minor_arc(YAW_REL_RAD_MIN,yaw_rad,2*PI)/(2.f*PI)*IMU_RANGE,-IMU_RANGE/2,IMU_RANGE/2);
        yaw_imu_max = range_map(cur_state->yaw_ang + get_minor_arc(YAW_REL_RAD_MAX,yaw_rad,2*PI)/(2.f*PI)*IMU_RANGE,-IMU_RANGE/2,IMU_RANGE/2);

        min_debug = get_minor_arc(cur_state->pitch_ang,yaw_imu_min,IMU_RANGE);
        max_debug = get_minor_arc(cur_state->pitch_ang,yaw_imu_max,IMU_RANGE);
        if((get_minor_arc(cur_state->pitch_ang,pitch_imu_min,IMU_RANGE) <= pitch_res_cpst/(2*PI) * IMU_RANGE && rocker_lx < 0.0f) || 
            (get_minor_arc(cur_state->pitch_ang,pitch_imu_max,IMU_RANGE) >= -pitch_res_cpst/(2*PI) * IMU_RANGE && rocker_lx > 0.0f))
            rocker_lx = 0; // 已经越界了还要更越界则取消本次操作
        else exp_state->pitch_ang += rocker_lx;
        if((get_minor_arc(cur_state->yaw_ang,yaw_imu_min,IMU_RANGE) >= -yaw_res_cpst/(2*PI) * IMU_RANGE && rocker_ly > 0.0f) || 
            ((get_minor_arc(cur_state->yaw_ang,yaw_imu_max,IMU_RANGE)) <= yaw_res_cpst/(2*PI) * IMU_RANGE && rocker_ly < 0.0f))
            rocker_ly = 0; // 已经越界了还要更越界则取消本次操作
        else exp_state->yaw_ang += rocker_ly;
#if 1 // 瞄车
       if(robot->use_vision_flag && vision->target){
           if(!is_vision_offline() &&
            vision_ctrl(&vision_pitch, &vision_yaw, span_of(motors->pitch),
                      span_of(motors->yaw)) ==
             VISION_OK) {//视觉锁定优先，控制云台
                   exp_state->pitch_ang = -(vision_pitch + assemble_bias_pitch);
                   exp_state->yaw_ang = vision_yaw + assemble_bias_yaw;        
               if(fabsf(vision->v_yaw) > 2.5f){//部分车自瞄敌方小陀螺时会偏高
                   exp_state->pitch_ang += vision_spin_bias;
               }
               if(robot->chassis_tag == C_SPIN_MODE){//自身小陀螺自瞄yaw修正
                   exp_state->yaw_ang += cur_state->w_z * SPINNING_GIMBAL_REVISE_K;
               }
           }
       }
#else // 瞄人
				if(fabsf(vision->yaw) < 45.f && vision->yaw){
							e_yaw = echo_kp * ((-vision->yaw) + 22.5f);
                if(e_yaw_prev != e_yaw ){
                    change_yaw_enable = 1;
                }
                else 
                    change_yaw_enable = 0;
                
                if(change_yaw_enable){
                    exp_state->yaw_ang += deg2rad(e_yaw)/2.f/PI * IMU_RANGE;
                    range_map(exp_state->yaw_ang, -4096, 4096);
                    change_yaw_enable = 0;
                }

                if(fabsf(get_minor_arc(exp_state->yaw_ang, cur_state->yaw_ang, IMU_RANGE)) < SOFT_START_DEADBAND_MOTOR){
                    change_yaw_enable = 1;
                }
                
                e_yaw_prev = e_yaw;
            }
            else e_yaw_prev = e_yaw = 0;
#endif
    }
    else if(gimbal_ctrl_type == USE_MOTOR){
        pitch_rad_min = motors->pitch.real.abs_angle/PITCH_RR - get_minor_arc(pitch_rad,PITCH_REL_RAD_MIN,2*PI);
        pitch_rad_max = motors->pitch.real.abs_angle/PITCH_RR - get_minor_arc(pitch_rad,PITCH_REL_RAD_MAX,2*PI);
        if ((motors->pitch.real.abs_angle/PITCH_RR <= pitch_rad_min + pitch_res_cpst && rocker_lx < 0.0f) || 
            (motors->pitch.real.abs_angle/PITCH_RR >= pitch_rad_max - pitch_res_cpst && rocker_lx > 0.0f))
            rocker_lx = 0; // 已经越界了还要更越界则取消本次操作
        else exp_state->pitch_ang += rocker_lx/IMU_RANGE*2*PI;
        yaw_rad_min = motors->yaw.real.abs_angle/YAW_RR - get_minor_arc(yaw_rad,YAW_REL_RAD_MIN,2*PI);
        yaw_rad_max = motors->yaw.real.abs_angle/YAW_RR - get_minor_arc(yaw_rad,YAW_REL_RAD_MAX,2*PI);
        if ((motors->yaw.real.abs_angle/YAW_RR <= yaw_rad_min + yaw_res_cpst && rocker_ly < 0.0f) || 
            (motors->yaw.real.abs_angle/YAW_RR >= yaw_rad_max - yaw_res_cpst && rocker_ly > 0.0f))
            rocker_ly = 0; // 已经越界了还要更越界则取消本次操作
        else exp_state->yaw_ang += rocker_ly/IMU_RANGE*2*PI;
    }
    
    // 根据tag变换期望pitch和yaw 被动切换tag(behaviour.c中主动切换)
    switch(robot->gimbal_tag){
        case G_STOP_MODE:
            gimbal_state_clear(robot);
            // shutdown_all_motor();
        break;
        case G_LAUNCH_MODE:
        // LIMIT_MIN_MAX(exp_state->pitch_ang, pitch_motor_min, pitch_motor_max);
        if(gimbal_reset_state != RESETED || need_change_yaw_center){
            update_tag(robot,NULL,G_LAUNCH_MODE,NULL);
            gimbal_reset_state = gimbal_smooth_reseting(exp_state,motors);
            if (gimbal_reset_state == RESETED){
                // 共量程开始
                gimbal_ctrl_type = select_gimbal_ctrl_type(robot, gimbal_reset_state);
                exp_state->pitch_ang = cur_state->pitch_ang;
                exp_state->yaw_ang = cur_state->yaw_ang;
                asr_control_enable_cmd();
                // update_tag(robot,NULL,G_DEFAULT_MODE,NULL);
            }
        }
        break;
        case G_MANUAL_MODE:
        if(gimbal_ctrl_type == USE_IMU){
            exp_state->yaw_ang = get_minor_arc(exp_state->yaw_ang, 0, IMU_RANGE); // 使电机响应不过来的时候不会误判(比如快速掉头)
            // IMU_RANGE 制
            LIMIT_MIN_MAX(exp_state->pitch_ang,pitch_imu_min,pitch_imu_max);
            LIMIT_MIN_MAX_ARC(exp_state->yaw_ang,yaw_imu_min,yaw_imu_max,IMU_RANGE);  // pitch轴不可能相对大地旋转 而非全向运动的yaw轴会
        }
        else if(gimbal_ctrl_type == USE_MOTOR){ 
            exp_state->yaw_ang = get_minor_arc(exp_state->yaw_ang, 0, 2*PI);
            LIMIT_MIN_MAX(exp_state->pitch_ang,pitch_rad_min,pitch_rad_max);
            LIMIT_MIN_MAX_ARC(exp_state->yaw_ang,yaw_rad_min,yaw_rad_max,2*PI);
        }
        break;
        case G_PATROL_MODE:
        exp_state->yaw_ang = cur_state->yaw_ang;
        exp_state->pitch_ang = cur_state->pitch_ang;
        if (gimbal_ctrl_type == USE_IMU){ // TODO:只是简单模拟了一个 未来可以搞贝塞尔曲线或者三角函数消除阶跃的影响
            static float patrol_pitch_dir = 1.f;
            static float patrol_yaw_dir = 1.f;
            if(vision->target){
                exp_state->pitch_ang = -(vision_pitch + assemble_bias_pitch);
                exp_state->yaw_ang = vision_yaw + assemble_bias_yaw;        
                if(fabsf(vision->v_yaw) > 2.5f){//部分车自瞄敌方小陀螺时会偏高
                    exp_state->pitch_ang += vision_spin_bias;
                }
                if(robot->chassis_tag == C_SPIN_MODE){//自身小陀螺自瞄yaw修正
                    exp_state->yaw_ang += cur_state->w_z * SPINNING_GIMBAL_REVISE_K;
                }
            }
            else{
                // 余弦变化
                if(get_minor_arc(cur_state->pitch_ang,pitch_imu_min,IMU_RANGE) <= pitch_res_cpst/(2*PI) * IMU_RANGE)
                    patrol_pitch_dir = 1 * (1+cosf(get_minor_arc(cur_state->pitch_ang,-pitch_imu_max,IMU_RANGE)/IMU_RANGE*2.f*PI));
                else if(get_minor_arc(cur_state->pitch_ang,pitch_imu_max,IMU_RANGE) >= -pitch_res_cpst/(2*PI) * IMU_RANGE) 
                    patrol_pitch_dir = -1 * (1+cosf(get_minor_arc(cur_state->pitch_ang,pitch_imu_min,IMU_RANGE)/IMU_RANGE*2.f*PI));
                if(get_minor_arc(cur_state->yaw_ang,yaw_imu_min,IMU_RANGE) >= -yaw_res_cpst/(2*PI) * IMU_RANGE)
                    patrol_yaw_dir = -1 * (1+cosf(get_minor_arc(cur_state->yaw_ang,yaw_imu_max,IMU_RANGE)/IMU_RANGE*2.f*PI));
                else if(get_minor_arc(cur_state->yaw_ang,yaw_imu_max,IMU_RANGE) <= yaw_res_cpst/(2*PI) * IMU_RANGE) 
                    patrol_yaw_dir = 1 * (1+cosf(get_minor_arc(cur_state->yaw_ang,yaw_imu_min,IMU_RANGE)/IMU_RANGE*2.f*PI));
                exp_state->yaw_ang += patrol_yaw_dir * PATROL_YAW_SPEED;
                exp_state->pitch_ang += patrol_pitch_dir * PATROL_PITCH_SPEED;
                // LIMIT_MIN_MAX(exp_state->pitch_ang,pitch_imu_min,pitch_imu_max); // 倒悬云台需要取反
                LIMIT_MIN_MAX_ARC(exp_state->yaw_ang,yaw_imu_min,yaw_imu_max,IMU_RANGE);  // pitch轴不可能相对大地旋转 而非全向运动的yaw轴会
            }
        }
        break;
        case G_QUICKTURN_MODE:
        turn_cnt++;
        if(turn_cnt == 20 && turnable_flag == ENABLE){
            if(gimbal_ctrl_type == USE_IMU){
                //exp_state->pitch_ang = get_minor_arc(pitch_rad,pitch_rad_center,2*PI)/(2*PI)*IMU_RANGE + cur_state->pitch_ang; 
                exp_state->yaw_ang += IMU_RANGE / 2;
            }
            else if(gimbal_ctrl_type == USE_MOTOR){
                exp_state->pitch_ang = pitch_rad_center; 
                exp_state->yaw_ang += PI;
            }
            turnable_flag = DISABLE;
        }
        if(turn_cnt >= 100){
            turn_cnt = 0;
            turnable_flag = ENABLE;
            update_tag(robot,NULL,G_DEFAULT_MODE,NULL);
        }
        break;
        default:break;
    } 
    
    #if (BOARDS_MODE && MACHINE_TYPE == GIMBAL_MASTER) || !BOARDS_MODE
        if(gimbal_ctrl_type == USE_MOTOR)// 电机系控云台 exp_state与电机共量程
        gimbal_motor_ctrl(motors,robot,pitch_pid,yaw_pid); 
        else if(gimbal_ctrl_type == USE_IMU)// IMU系控云台 exp_state与imu共量程
        gimbal_imu_ctrl(motors,robot,
                        pitch_imu_pid,yaw_imu_pid,
                        pitch_imu_vision_pid,yaw_imu_vision_pid,
                        &pitch_imu_leso,&yaw_imu_leso,
                        &kalman_pitch_filter,&kalman_yaw_filter); 
        else;
    #endif
}
/**
 * @brief 判断云台是否缓启动完成函数
 * @retval gimbal_reset_flag
 * @arg FINISH:1
 * @arg UNFINISH:0
 */
uint8_t is_gimbal_reseted(void){
    #if USE_GIMBAL_RESET
    if(gimbal_reset_state == RESETED)
    gimbal_reset_flag = FINISH;
    else gimbal_reset_flag = UNFINISH;
    #else
    gimbal_reset_flag = FINISH;
    #endif
    return gimbal_reset_flag;
}
uint8_t *get_gimbal_reset_flag_ptr(void){
    is_gimbal_reseted();
    return &gimbal_reset_flag;
} 
uint8_t *get_gimbal_ctrl_type_ptr(void){
    return &gimbal_ctrl_type;
}
