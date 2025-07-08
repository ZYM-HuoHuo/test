/**
 * @file asr.c
*
 * @brief ASR处理接口
 *
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#include "asr.h"
#include "i2c.h"

#define ASR_DEVADDRESS 0x10

#define CONTROL_DAT 1
#define ASR_I2C 0
#define ASR_UART 1

#if USE_ASR
#if ASR_I2C
extern I2C_HandleTypeDef hi2c2;
#define ASR_SEND_CMD(cmd) HAL_I2C_Master_Transmit(&hi2c2, ASR_DEVADDRESS, (uint8_t *)&cmd, 1, 0x10)
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c){
    if(hi2c == &hi2c2){
        if(recv_dat == CONTROL_DAT)
        asr_control_flag = recv_dat;
    }
}
#elif ASR_UART
extern UART_HandleTypeDef huart1;
#define ASR_SEND_CMD(cmd)                                   \
    char ch[1] = {cmd};                                     \
    HAL_UART_Transmit(&huart1, (uint8_t *)ch, 1, 0x10);
#endif
#else
#define ASR_SEND_CMD(cmd) ;
#endif

uint8_t recv_dat = 0;
uint8_t asr_control_flag = DISABLE;


uint8_t *get_asr_control_flag(void){
    HAL_I2C_Master_Receive_IT(&hi2c2, ASR_DEVADDRESS, &recv_dat, 1);
    return &asr_control_flag;
}

asr_cmds_t asr_cmds;
asr_cmds_t *get_asr_cmds(void){
    return &asr_cmds;
}
asr_cmd_t *ready_msg[NUM_OF_ASR_CMD];
uint8_t ready_msg_counting = 0; // 记录有效的缓冲区内容数
void asr_cmd_init(void){
    for(uint8_t i = 0; i < NUM_OF_ASR_CMD; i++) {
        asr_cmds._[i].cmd_val = (char)('1'+ i);
        asr_cmds._[i].ready = false;  
        ready_msg[i] = NULL;  
    }
    ready_msg_counting = 0;
}

void asr_cmd_load(asr_cmd_t *asr_cmd){
    asr_cmd_t **asr_cmdp = NULL;
    if(asr_cmd->ready == false){
        asr_cmd->ready = true;
        uint8_t cheak_counting = ready_msg_counting + 1;
        if(cheak_counting < NUM_OF_ASR_CMD){
            asr_cmdp = &ready_msg[ready_msg_counting];
            *asr_cmdp = asr_cmd;
            ready_msg_counting++;
        }
        else return ;
    }
}
/**
 * @brief 放在10Hz定时器中 3秒发一次
 */
uint8_t asr_debug_val = 0;
uint16_t loop_cnt = 0;
void asr_cmd_send_loop(uint16_t FREQ){
		// ASR_SEND_CMD(asr_debug_val)
    static bool uncontrol_restrict_for_asr = false;
    robot_t *robot = get_robot_ptr();
    if(robot->robot_control_flag == ENABLE){
        uncontrol_restrict_for_asr = true;
        if(loop_cnt++ == FREQ*3){
            loop_cnt = 0;
						ASR_SEND_CMD(ready_msg[0]->cmd_val);
						ready_msg[0]->ready = false;
						if(ready_msg_counting > 0)ready_msg_counting--;
						for(uint8_t i = 0; i < ready_msg_counting; i++){
								ready_msg[i] = ready_msg[i+1];
						}
						ready_msg[ready_msg_counting] = NULL;
        }
    }
    else{
        loop_cnt = 0;
        for(uint8_t i = 0; i < NUM_OF_ASR_CMD; i++) {
            asr_cmds._[i].ready = false;    
        }
        ready_msg_counting = 0;
        memset(ready_msg, 0, sizeof(asr_cmds._));
        if(uncontrol_restrict_for_asr){
            ASR_SEND_CMD(asr_cmds.ASR_CONTROL_DISABLE.cmd_val);
        }
        uncontrol_restrict_for_asr = false;
    }
}


/*具体的发送函数*/
void asr_init_finish_cmd(void){asr_cmd_load(&asr_cmds.ASR_INIT_FINISH);}
void asr_struct_offline_cmd(motors_type_t motors_type){
    uint8_t index = asr_cmds.ASR_CHASSIS_OFFLINE.cmd_val - '1' + motors_type;
    asr_cmd_load(&asr_cmds._[index]);
}
void asr_motor_heat_cmd(uint8_t motor_id){
    // if(motor_id < NUM_OF_ALL_MOTOR){
    //     uint8_t index = asr_cmds.ASR_CHASSIS_OFFLINE.cmd_val - '1' + motor_id;
    //     asr_cmd_load(&asr_cmds._[index]);
    // }
    // 废弃
    ;
}
void asr_rc_offline_cmd(void){asr_cmd_load(&asr_cmds.ASR_RC_OFFLINE);}
void asr_referee_offline_cmd(void){asr_cmd_load(&asr_cmds.ASR_REFEREE_OFFLINE);}
void asr_power_over_cmd(void){asr_cmd_load(&asr_cmds.ASR_POWER_OVER);}
void asr_cap_exhausted_cmd(void){asr_cmd_load(&asr_cmds.ASR_CAP_EXHAUSTED);}
void asr_vision_error_cmd(void){asr_cmd_load(&asr_cmds.ASR_VISION_ERROR);} // 未调用
void asr_gimbal_launch_error_cmd(void){asr_cmd_load(&asr_cmds.ASR_GIMBAL_LAUNCH_ERROR);}
void asr_shooter_stuck_cmd(void){asr_cmd_load(&asr_cmds.ASR_SHOOTER_STUCK);}

void asr_control_enable_cmd(void){asr_cmd_load(&asr_cmds.ASR_CONTROL_ENABLE);}
void asr_control_disable_cmd(void){asr_cmd_load(&asr_cmds.ASR_CONTROL_DISABLE);}
void asr_shooter_start_cmd(void){asr_cmd_load(&asr_cmds.ASR_SHOOTER_START);}
void asr_shooter_stop_cmd(void){asr_cmd_load(&asr_cmds.ASR_SHOOTER_STOP);}

void asr_run_cmd(void){asr_cmd_load(&asr_cmds.ASR_RUN);}
void asr_hurt_cmd(void){asr_cmd_load(&asr_cmds.ASR_HURT);}
void asr_killed_cmd(void){asr_cmd_load(&asr_cmds.ASR_KILLED);}
void asr_reborn_cmd(void){asr_cmd_load(&asr_cmds.ASR_REBORN);}

void asr_fire_cmd(void){asr_cmd_load(&asr_cmds.ASR_FIRE);}
void asr_supercap_enable_cmd(void){asr_cmd_load(&asr_cmds.ASR_SUPERCAP_ENABLE);}

