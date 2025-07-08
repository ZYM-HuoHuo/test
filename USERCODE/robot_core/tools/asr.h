#ifndef _ASR_H
#define _ASR_H

#include "drv_conf.h"
#include HAL_INCLUDE
#include "app/robot.h"

#define NUM_OF_ASR_CMD 29
typedef struct{
    bool ready; //表示就绪
    char cmd_val; // 命令字符值
}asr_cmd_t;
typedef union{
    struct{
        asr_cmd_t ASR_INIT_FINISH;
        // ↑ 1

        asr_cmd_t ASR_CHASSIS_OFFLINE;
        asr_cmd_t ASR_GIMBAL_OFFLINE;
        asr_cmd_t ASR_SHOOTER_OFFLINE;
        // ↑ 3

        asr_cmd_t ASR_MOTOR0_HEAT;
        asr_cmd_t ASR_MOTOR1_HEAT;
        asr_cmd_t ASR_MOTOR2_HEAT;
        asr_cmd_t ASR_MOTOR3_HEAT;
        asr_cmd_t ASR_MOTOR4_HEAT;
        asr_cmd_t ASR_MOTOR5_HEAT;
        asr_cmd_t ASR_MOTOR6_HEAT;
        asr_cmd_t ASR_MOTOR7_HEAT;
        // ↑ 8

        asr_cmd_t ASR_RC_OFFLINE;
        asr_cmd_t ASR_REFEREE_OFFLINE;
        asr_cmd_t ASR_POWER_OVER;
        asr_cmd_t ASR_CAP_EXHAUSTED;
        asr_cmd_t ASR_VISION_ERROR;
        asr_cmd_t ASR_GIMBAL_LAUNCH_ERROR;
        asr_cmd_t ASR_SHOOTER_STUCK;
        // ↑ 7

        asr_cmd_t ASR_CONTROL_ENABLE;
        asr_cmd_t ASR_CONTROL_DISABLE;
        asr_cmd_t ASR_SHOOTER_START;
        asr_cmd_t ASR_SHOOTER_STOP;
        // ↑ 4

        asr_cmd_t ASR_RUN;
        asr_cmd_t ASR_HURT;
        asr_cmd_t ASR_KILLED;
        asr_cmd_t ASR_REBORN;
        // ↑ 4

        asr_cmd_t ASR_FIRE;
        asr_cmd_t ASR_SUPERCAP_ENABLE;
        // ↑ 2
    };
    asr_cmd_t _[NUM_OF_ASR_CMD];
}asr_cmds_t;

void asr_cmd_init(void);

uint8_t *get_asr_control_flag(void);

asr_cmds_t *get_asr_cmds(void);

void asr_cmd_load(asr_cmd_t *asr_cmd);
void asr_cmd_send_loop(uint16_t FREQ);

///< func
void asr_init_finish_cmd(void);

void asr_struct_offline_cmd(motors_type_t motors_type);

void asr_motor_heat_cmd(uint8_t motor_id);

void asr_rc_offline_cmd(void);
void asr_referee_offline_cmd(void);
void asr_power_over_cmd(void);
void asr_cap_exhausted_cmd(void);
void asr_vision_error_cmd(void);
void asr_gimbal_launch_error_cmd(void);
void asr_shooter_stuck_cmd(void);

void asr_control_enable_cmd(void);
void asr_control_disable_cmd(void);
void asr_shooter_start_cmd(void);
void asr_shooter_stop_cmd(void);

void asr_run_cmd(void);
void asr_hurt_cmd(void);
void asr_killed_cmd(void);
void asr_reborn_cmd(void);

void asr_fire_cmd(void);
void asr_supercap_enable_cmd(void);

#endif 
