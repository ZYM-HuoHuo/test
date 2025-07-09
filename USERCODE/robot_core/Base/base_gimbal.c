/**
 * @file base_gimbal.c
*
 * @brief 云台底层
 *
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#include "base_gimbal.h"

#include "Base/ext_imu.h"
#include "Devices/MOTOR/motor_headers.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern gimbal_ctrl_type_t gimbal_ctrl_type;

float ref_pitch_p = 0, ref_yaw_p = 0;
float pitch_ref_tff = 0,yaw_ref_tff = 0;
float vision_yaw_output,vision_pitch_output;

float delta_ang_pitch,delta_ang_yaw;
float yaw_torque_revise = 106.f;

float pitch_imu_rpm = 0;
float yaw_imu_rpm = 0;

// pitch前馈电压 单位V
float pitch_feedback = 0; 
// pitch力平衡时的角度 单位编码值
int16_t pitch_balance_pos = PITCH_BAL_POS; // TODO:对于头部重量会变化的头 建议搞成自适应的
// 云台重力对应的电压
float gimbal_weight = GIMBAL_WEIGHT;
// 云台运动最大速度 单位rad/s
float gimbal_vel_max = 100.f;

/**
 * @brief 云台IMU系控制
 * @attention 电压控+leso控yaw轴GM6020 电流控(或位置速度控)pitch轴DM4310
 */
float pitch_output_debug = 0; // debug
HAL_StatusTypeDef gimbal_imu_ctrl(
                motors_t *motors,
                robot_t *robot,
                pid_struct_t pitch_imu_pid[2], pid_struct_t yaw_imu_pid[2],
                pid_struct_t pitch_vision_pid[2], pid_struct_t yaw_vision_pid[2],
                leso_para_t *pitch_imu_leso, leso_para_t *yaw_imu_leso,
                one_vec_kf_t *kalman_pitch_filter, one_vec_kf_t *kalman_yaw_filter){
    HAL_StatusTypeDef rslt = HAL_OK;
    motor_data_t *pitch_motor_pdata = &motors->pitch.real;
    motor_data_t *yaw_motor_pdata = &motors->yaw.real;
    robot_state_t *exp_state = robot->expected_state;
    robot_state_t *cur_state = robot->current_state;
    imu_data_t *imu = robot->imu_data;
    if(imu == NULL) {return HAL_ERROR;}

    delta_ang_pitch = get_minor_arc(exp_state->pitch_ang,cur_state->pitch_ang,IMU_RANGE);
    delta_ang_yaw = get_minor_arc(exp_state->yaw_ang,cur_state->yaw_ang,IMU_RANGE);

    kalman1_filter(kalman_pitch_filter,pitch_motor_pdata->current + pitch_motor_pdata->rpm/Kn_RPM_GM6020*(GM6020_VOLT_DATA_MAX/GM6020_VOLT_RATED));
	kalman1_filter(kalman_yaw_filter,yaw_motor_pdata->current + yaw_motor_pdata->rpm/Kn_RPM_GM6020*(GM6020_VOLT_DATA_MAX/GM6020_VOLT_RATED));
    
    // fp_data.g单位为rad/s
    pitch_imu_rpm = imu->fp_data.gy*(60.f/(2.f*PI));
    yaw_imu_rpm = imu->fp_data.gz*(60.f/(2.f*PI))*cosf(imu->eular.pitch/IMU_RANGE*(2.f*PI)); // 余弦映射到机体坐标系
    
    // 调前馈的时候 将pid_imu的输出给0
    pitch_feedback = sinf(motors->pitch.real.abs_angle / PITCH_RR - pitch_balance_pos) * gimbal_weight;
    // 期望电压
    pitch_ref_tff = CLAMP(pid_dual_loop(pitch_imu_pid,-delta_ang_pitch,pitch_imu_rpm) + pitch_feedback, DM4310_CURR_MAX);
    yaw_ref_tff = CLAMP(pid_leso_dualloop(yaw_imu_pid,yaw_imu_leso, -delta_ang_yaw, 
                    -yaw_imu_rpm,kalman_yaw_filter->x), GM6020_VOLT_MAX);

    ref_pitch_p = motors->pitch.AUX.xout.dat + pitch_motor_pdata->abs_angle + delta_ang_pitch/IMU_RANGE*(2*PI);
    ref_yaw_p = motors->yaw.AUX.xout.dat + yaw_motor_pdata->abs_angle + delta_ang_yaw/IMU_RANGE*(2*PI);
    
    // pitch_ref_tff = pitch_output_debug;   // debug
    // yaw_ref_tff = 0;     // debug
    if(is_motors_offline(GIMBAL_MOTORS)){
        shutdown_motors_of_structure(GIMBAL_MOTORS);
        return HAL_ERROR;
    }
    if(robot->robot_control_flag){
        if(motors->pitch.work_mode == MIT_TT || motors->pitch.work_mode == QUAD_CURR)
            LOAD_MOTOR_LPFTFF(motors->pitch, pitch_ref_tff);
        else{
            LOAD_MOTOR_PDES(motors->pitch, ref_pitch_p);
            LOAD_MOTOR_VDES(motors->pitch, gimbal_vel_max);
        }
        if(motors->yaw.work_mode == MIT_TT || motors->yaw.work_mode == QUAD_CURR)
            LOAD_MOTOR_LPFTFF(motors->yaw, yaw_ref_tff);
        else{
            LOAD_MOTOR_PDES(motors->yaw, ref_yaw_p);
            LOAD_MOTOR_VDES(motors->yaw, gimbal_vel_max);
        }
        motors->pitch.AUX.EN = true;
        rslt |= set_all_motor_output(GIMBAL_MOTORS);
    }
    else{
        if(motors->pitch.work_mode == MIT_TT || motors->pitch.work_mode == QUAD_CURR)
            LOAD_MOTOR_TFF(motors->pitch, 0);
        else
            LOAD_MOTOR_VDES(motors->pitch, 0);
        if(motors->yaw.work_mode == MIT_TT || motors->yaw.work_mode == QUAD_CURR)
            LOAD_MOTOR_TFF(motors->yaw, 0);
        else
            LOAD_MOTOR_VDES(motors->yaw, 0);
        motors->pitch.AUX.EN = false;
        rslt |=  HAL_OK;
    }
    return rslt;
}

/**
 * @brief 云台电机系控制
 * @note 无人机不能借助轮毂电机来闭环 需要新函数 -->get_gimbal_angle_for_air
 * @attention 电压控yaw轴GM6020 电流控pitch轴DM4310
 * @attention 真·缓启动下cur量为raw_scale   缓启动下cur量为rad  缓启动完成下cur量为abs_angle/RR
 */
float cur_yaw_angle = 0; // ang_rel
float cur_pitch_angle = 0;
HAL_StatusTypeDef gimbal_motor_ctrl(
                motors_t* motors,
                robot_t *robot,
                pid_struct_t pitch_pid[2],pid_struct_t yaw_pid[2] ){
    HAL_StatusTypeDef rslt = HAL_OK;
    motor_data_t *pitch_motor_pdata = &motors->pitch.real;
    motor_data_t *yaw_motor_pdata = &motors->yaw.real;
    robot_state_t *exp_state = robot->expected_state;
    robot_state_t *cur_state = robot->current_state;

    int16_t pitch_span = span_of(motors->pitch);
    int16_t yaw_span = span_of(motors->yaw);

    if(is_gimbal_reseted()){ // 用abs_angle/RR
        delta_ang_pitch = get_minor_arc(exp_state->pitch_ang,cur_state->pitch_ang,2*PI)/(2*PI)*pitch_span;
        delta_ang_yaw = get_minor_arc(exp_state->yaw_ang,cur_state->yaw_ang,2*PI)/(2*PI)*yaw_span;
    }
    else{
        if(is_confirm_0degree){ // 用相对rad
            delta_ang_pitch = get_minor_arc(exp_state->pitch_ang,cur_state->pitch_ang,2*PI)/(2*PI)*pitch_span;
            delta_ang_yaw = get_minor_arc(exp_state->yaw_ang,cur_state->yaw_ang,2*PI)/(2*PI)*yaw_span;
        }
        else{ // 用raw_scale
            delta_ang_pitch = get_minor_arc(exp_state->pitch_ang,pitch_motor_pdata->raw_scale,pitch_span);
            delta_ang_yaw = -get_minor_arc(exp_state->yaw_ang,get_gimbal_angle_for_air(motors,&motors->yaw),yaw_span);
        }
    }
    // float pitch_current_rpm = iir_filter_2(pitch_motor_pdata->rpm, GIMBAL_PITCH_FILTER2_RPM_CH);
    float pitch_current_rpm = pitch_motor_pdata->rpm;
    float yaw_current_rpm = -iir_filter_2(yaw_motor_pdata->rpm, GIMBAL_YAW_FILTER2_RPM_CH);

    // 期望电压
    pitch_ref_tff = CLAMP(pid_dual_loop(pitch_pid, -delta_ang_pitch, pitch_current_rpm),DM4310_CURR_MAX);
    yaw_ref_tff = -CLAMP(pid_dual_loop(yaw_pid, -delta_ang_yaw, yaw_current_rpm),GM6020_VOLT_MAX);

    ref_pitch_p = motors->pitch.AUX.xout.dat + pitch_motor_pdata->abs_angle + delta_ang_pitch/pitch_span*(2*PI);
    ref_yaw_p = motors->yaw.AUX.xout.dat + yaw_motor_pdata->abs_angle + delta_ang_yaw/yaw_span*(2*PI);

    // pitch_ref_tff = pitch_output_debug;   // debug
    // yaw_ref_tff = 0;     // debug
    if(is_motors_offline(GIMBAL_MOTORS)){
        shutdown_motors_of_structure(GIMBAL_MOTORS);
        return HAL_ERROR;
    }
    if(robot->robot_control_flag){
        if(motors->pitch.work_mode == MIT_TT || motors->pitch.work_mode == QUAD_CURR)
            LOAD_MOTOR_LPFTFF(motors->pitch, pitch_ref_tff);
        else{
            LOAD_MOTOR_PDES(motors->pitch, ref_pitch_p);
            LOAD_MOTOR_VDES(motors->pitch, gimbal_vel_max);
        }
        if(motors->yaw.work_mode == MIT_TT || motors->yaw.work_mode == QUAD_CURR)
            LOAD_MOTOR_LPFTFF(motors->yaw, yaw_ref_tff);
        else{
            LOAD_MOTOR_PDES(motors->yaw, ref_yaw_p);
            LOAD_MOTOR_VDES(motors->yaw, gimbal_vel_max);
        }
        motors->pitch.AUX.EN =0 ;
        motors->yaw.AUX.EN = true;//////////////////////////////////////////////////////

        rslt |= set_all_motor_output(GIMBAL_MOTORS);
    }


    else{
        if(motors->pitch.work_mode == MIT_TT || motors->pitch.work_mode == QUAD_CURR)
            LOAD_MOTOR_TFF(motors->pitch, 0);
        else
            LOAD_MOTOR_VDES(motors->pitch, 0);
        if(motors->yaw.work_mode == MIT_TT || motors->yaw.work_mode == QUAD_CURR)
            LOAD_MOTOR_TFF(motors->yaw, 0);
        else
            LOAD_MOTOR_VDES(motors->yaw, 0);
        motors->pitch.AUX.EN = false;
        rslt |=  HAL_OK;
    }
    return rslt;
}

/**
 * @brief 获取底盘链接yaw部分转动导致的云台角度变化
 * @note 此函数设置的目的是使在缓启动时考虑上底盘被移动产生的偏置
* @note 但显然无人机并没有底盘
 */
float get_gimbal_angle_for_air(motors_t *motors, motor_t *gimbal_pmotor){
    static float z_angle = 0;
    z_angle = gimbal_pmotor->real.raw_scale;
    return z_angle;
}

