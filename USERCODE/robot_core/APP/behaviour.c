/**
 * @file behaviour.c
 * @brief 状态机
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */

#include "behaviour.h"
#include "Devices/MOTOR/motor_headers.h"
#include "gimbal.h"
#include "shooter.h"

#include <string.h>

extern TIM_HandleTypeDef MAGAZINE_TIM_HANDLE;

extern uint8_t target_req_change;
extern uint8_t target_req_reset_tracker;

extern robot_t robot;
extern robot_state_t expected_state,current_state;
extern imu_data_t imu_data;

uint16_t magezine_on_pulse = 2500;
uint16_t magezine_off_pulse = 1415;
#define COVER_WORK_MAXTIM 250

uint8_t rc_sc;

RAM_PERSIST float rocker_rx = 0;
RAM_PERSIST float rocker_ry = 0; 
RAM_PERSIST float rocker_lx = 0;
RAM_PERSIST float rocker_ly = 0; 

chassis_tag_t C_DEFAULT_MODE = C_FOLLOW_MODE;
gimbal_tag_t G_DEFAULT_MODE = G_MANUAL_MODE;
int8_t spin_direction = 1;
uint8_t need_change_yaw_center = 0;

/**
 * @brief 机器人默认初始不使能 初始化所有行为和模式 默认为NORMAL
 * @param robot 机器人结构体指针
 * @retval None
 */
void behaviour_init(robot_t* robot)
{
    memset(&expected_state, 0, sizeof(robot_state_t));
    memset(&current_state, 0, sizeof(robot_state_t));
    robot->expected_state = &expected_state;
    robot->current_state = &current_state;

    robot->robot_control_flag = DISABLE;
    robot->robot_survival_flag = DISABLE;

    robot->robot_basic_mode = NORMAL_MODE;
    robot->rc_sc = DT7_CONTROLLER;

    robot->reset_flag = UNFINISH;

    robot->magazine_switch_flag = UNCONTROL;

    robot->chassis_tag = C_STOP_MODE;
    robot->gimbal_tag = G_STOP_MODE;
    robot->shooter_tag = S_STOP_MODE;

    robot->imu_data = &imu_data;
}

/**
 * @brief 更新机器人控制状态变量
 * @param rc_ctrl 控制指令结构体指针
 * @param robot 机器人结构体指针
 * @retval None
 */
void behaviour_ctrl_loop(rc_ctrl_t *rc_recv, robot_t* robot)
{
    asr_cmds_t *asr = get_asr_cmds();
    select_rc_sc(robot); // 选择控制源
    killed_protect(robot);  // 阵亡检测
    select_control_flag(rc_recv, robot); // 控制机器人上下控
    select_robot_basic_mode(rc_recv, robot); // 选择机器人基本模式

    if (rc_recv->keyboard.last_keycode != 0 && robot->rc_sc != BOARDS_CONTROLLER)
    robot->use_keyboard_flag = ENABLE; //按过键盘后就不能用遥控器的摇杆控了

    if(robot->robot_control_flag == ENABLE)
    {   
        if(robot->rc_sc == BOARDS_CONTROLLER){
            if(is_gimbal_reseted() /*&& is_chassis_reseted()*/){
                // update_tag(robot,C_DEFAULT_MODE,G_DEFAULT_MODE,NULL); //改为在机构中update避免抢占
                robot->reset_flag = FINISH;
            }
            else
            update_tag(robot,NULL,G_LAUNCH_MODE,NULL);
            ; // 若为板间控制（当前板为从机 则不主动改变behaviour内容）
        }
        else{
            if(robot->reset_flag == FINISH){ 
                if(robot->use_keyboard_flag == ENABLE) //使用键鼠
                    update_robot_status_for_keyboard(rc_recv,robot);
                else/*(robot->use_keyboard_flag == DISABLE*/ //使用遥控器
                    update_robot_status_for_remoteControl(rc_recv,robot);
            }
            else/*(robot->reset_flag == UNFINISH)*/{
                if(is_gimbal_reseted() ){
                    update_tag(robot,NULL,G_DEFAULT_MODE,NULL);
                    // if(is_chassis_reseted()){
                    //     update_tag(robot,C_DEFAULT_MODE,NULL,NULL);
                    //     robot->reset_flag = FINISH;
					// 					}
                    // else
                    //     update_tag(robot,C_LAUNCH_MODE,NULL,NULL);
                    robot->reset_flag = FINISH;
                }
                else
                update_tag(robot,NULL,G_LAUNCH_MODE,NULL);
            }
        }
    }
    else/*(robot->robot_control_flag == DISABLE)*/{
        robot->reset_flag = UNFINISH;
        robot->robot_basic_mode = NORMAL_MODE;

        robot->use_fric_flag = DISABLE;
        robot->use_supercap_flag = DISABLE;
        robot->use_vision_flag = DISABLE;

        robot->fire_flag = DISABLE;
        robot->rush_flag = DISABLE;

        // robot->magazine_switch_flag = UNCONTROL;
        if(rc_recv->rc.switch_left == RC_SW_MID)
        robot->magazine_switch_flag = MAG_ON;
        else robot->magazine_switch_flag = MAG_OFF;

        robot->chassis_tag = C_STOP_MODE;
        robot->gimbal_tag = G_STOP_MODE;
        robot->shooter_tag = S_STOP_MODE;

        need_change_yaw_center = 0;
				
		// shutdown_all_motor(); //为了减轻can负载 放在motor_switch关电机
    }
}

/**
 * @brief 电机失能和使能管理函数
 * @note 为减轻CAN总线负载才搞的这个玩意 专门放在频率不高的定时器里调用
 */
void motor_switch(uint8_t switch_flag){ 
    if(switch_flag == DISABLE){
        #if !USE_TEST_FILE
        shutdown_all_motor();
        #endif
    }
}

/**
 * 
 */
/**
 * @brief 补弹机构控制函数
 * @param s_flag 机构控制标志位
 *      @arg UNCONTROL:不控制(有bug)
 *      @arg MAG_OFF:关弹
 *      @arg MAG_ON:开弹
 * @retval None
 */
void magazine_ctrl_loop(enum _magazine_switch_flag s_flag){
    static uint8_t cover_control_time = COVER_WORK_MAXTIM;
    if(s_flag == MAG_OFF){ 
		if(cover_control_time)
		{
			cover_control_time--;
			if(cover_control_time < 110){
				__HAL_TIM_SetCompare(&MAGAZINE_TIM_HANDLE, MAGAZINE_TIM_CHANNAL, magezine_off_pulse);
			}else if(cover_control_time < 160){
				__HAL_TIM_SetCompare(&MAGAZINE_TIM_HANDLE, MAGAZINE_TIM_CHANNAL, magezine_on_pulse);
			}else{
				__HAL_TIM_SetCompare(&MAGAZINE_TIM_HANDLE, MAGAZINE_TIM_CHANNAL, magezine_off_pulse);
			}
		}
    }
    else if(s_flag == MAG_ON){
        __HAL_TIM_SetCompare(&MAGAZINE_TIM_HANDLE, MAGAZINE_TIM_CHANNAL, magezine_on_pulse);
        cover_control_time = COVER_WORK_MAXTIM;
    }
    else{
        __HAL_TIM_SetCompare(&MAGAZINE_TIM_HANDLE, MAGAZINE_TIM_CHANNAL, NULL);
        cover_control_time = COVER_WORK_MAXTIM;
    }
}

/**
 * @brief 阵亡自动断控函数
 * @param robot 机器人结构体指针
 * @retval None
 */
void killed_protect(robot_t *robot){
    static bool alive_state = false;
    static bool alive_state_prev = false;
    static float HP_prev = 0;
    // 受击检测
    if(game_robot_status.current_HP < HP_prev)
        asr_hurt_cmd();
    HP_prev = game_robot_status.current_HP;
    // 阵亡检测
    
    alive_state = (game_robot_status.current_HP>0)?true:false;
	#if READY_FOR_BATTLE
    if (alive_state){
        robot->robot_survival_flag = 1;
        robot->use_supercap_flag = 1;
    }
    else{
        robot->robot_survival_flag = 0;
    }
	#else
	robot->robot_survival_flag = 1;
	#endif
    alive_state_prev = alive_state;
    if(alive_state_prev != alive_state){
        if(alive_state)
            asr_reborn_cmd();
        else
            asr_killed_cmd();
    }
}
/**
 * @brief 选择控制源函数
 * @param robot 机器人结构体指针
 * @retval None
 */
void select_rc_sc(robot_t *robot){
    if(MACHINE_TYPE == CHASSIS_SLAVE){
        robot->rc_sc = BOARDS_CONTROLLER;
    }
    else{
        if(is_rc_offline())
            robot->rc_sc = VT_CONTROLLER;
        else
            robot->rc_sc = DT7_CONTROLLER;
    }
    rc_sc = robot->rc_sc;
}
/**
 * @brief 处理机器人上下控
 */
void select_control_flag(rc_ctrl_t *rc_recv, robot_t *robot){
    if(robot->rc_sc == VT_CONTROLLER){ //控制源:图传链路
        if (is_key_pressed(KEY_A) && is_key_pressed(KEY_S) && is_key_pressed(KEY_Z))
        {
            if (robot->robot_survival_flag)
                robot->robot_control_flag = ENABLE;
            else
                robot->robot_control_flag = DISABLE;
            robot->use_keyboard_flag = ENABLE;
        }

        if (is_key_pressed(KEY_E) && is_key_pressed(KEY_C) && is_key_pressed(KEY_B))
        {
            robot->robot_control_flag = DISABLE;
            robot->use_keyboard_flag = DISABLE;
        }
    }
    else if(robot->rc_sc == DT7_CONTROLLER){ // 控制源:遥控器
        if (rc_recv->rc.switch_right == 0 || rc_recv->rc.switch_right == RC_SW_DOWN || is_rc_offline())
        { // 断控
            robot->robot_control_flag = DISABLE;
            robot->use_keyboard_flag = DISABLE;
        }
        else
        {
            if (robot->robot_survival_flag)
                robot->robot_control_flag = ENABLE;
            else
                robot->robot_control_flag = DISABLE;
			#if READY_FOR_BATTLE
            if (is_motors_offline(GIMBAL_MOTORS)){ //模拟云台被断电 防检录疯车
                robot->robot_control_flag = DISABLE;
            }
			#endif
        }
    }
    else if (robot->rc_sc == BOARDS_CONTROLLER){ // 控制源:多板控制
        ; // 进到这说明本机为从机 经由多板控制 上控由主机决定
        /*除chassisTag外其他机构tag和flag无效化*/
    }
    else{
        ;
    }
}
/**
 * @brief 选择机器人基础模式
 * @param robot 机器人结构体指针
 * @retval None
 */
void select_robot_basic_mode(rc_ctrl_t *rc_recv, robot_t *robot){
    /**
     * ↓↓↓处理机器人basic mode模式切换↓↓↓
     * 仅限遥控器处理
     */
    if(robot->rc_sc == DT7_CONTROLLER)
    {
        if (rc_recv->rc.switch_right == RC_SW_MID){
            robot->robot_basic_mode = NORMAL_MODE;
        }
        else if (rc_recv->rc.switch_right == RC_SW_UP){
            robot->robot_basic_mode = CHECK_MODE;
        }
    }
    else if(robot->rc_sc == VT_CONTROLLER){
        robot->robot_basic_mode = NORMAL_MODE;
    }
    else{ // 板间控制
        robot->robot_basic_mode = NORMAL_MODE;
    }
}
/**
 * @brief 控制源为键鼠的机器人控制变量更新
 * @param rc_recv 遥控指令结构体指针
 * @param robot 机器人结构体指针
 * @retval None
 */
uint8_t self_locking_cnt_max = 2;
uint8_t self_locking_cnt = DISABLE;
void update_robot_status_for_keyboard(rc_ctrl_t *rc_recv, robot_t *robot){
    // 获取输入值
    if (is_key_pressed(KEY_W)){rocker_rx += CLAMP((float)T_ACC_CNT - rocker_rx, S_CURVE_RX_ACC);}
    else if (is_key_pressed(KEY_S)){rocker_rx += CLAMP((float)-T_ACC_CNT - rocker_rx, S_CURVE_RX_ACC);}
    else{rocker_rx += CLAMP(-(rocker_rx), S_CURVE_RX_ACC);}
    if (is_key_pressed(KEY_D)){rocker_ry += CLAMP((float)T_ACC_CNT - rocker_ry, S_CURVE_RY_ACC);}
    else if (is_key_pressed(KEY_A)){rocker_ry += CLAMP((float)-T_ACC_CNT - rocker_ry, S_CURVE_RY_ACC);}
    else{rocker_ry += CLAMP(-rocker_ry, S_CURVE_RY_ACC);}
    rocker_lx = ((float)rc_recv->mouse.y * (GAIN_MOUSE_X));
    rocker_ly = -((float)rc_recv->mouse.x * (GAIN_MOUSE_Y));

    if(robot->robot_basic_mode == NORMAL_MODE){
        // *KEY G* 重启机器人
        static uint8_t key_g_keep_tim = 110;
        if (is_key_pressed(KEY_G)){
            key_g_keep_tim = 10;
        }
        if(key_g_keep_tim <= 10){
            key_g_keep_tim--;
            if(!key_g_keep_tim){
                key_g_keep_tim = 110;
                rc_recv->rc.switch_right = RC_SW_DOWN;
                robot->robot_control_flag = DISABLE;
                __disable_irq();
                NVIC_SystemReset();
            }
        }
        /*  ↓↓↓单按键控制 单个机器人状态 启停↓↓↓    */
        /*  因遥控器接收时序问题 触发并没有那么准确*/

        /* sample of SHIFT: [按住] 更新模式---[松开] 回到上一个模式 */
        if (!(is_key_last_pressed(KEY_SHIFT)) && (is_key_pressed(KEY_SHIFT))){
            update_tag(robot, C_SIDEWAYS_MODE, NULL, NULL); 
            //若需要更新相应的云台/发射tag 替换NULL-> XXX_MODE
            //并在下面的回溯函数update_tag 中 替换NULL->xxx_tag_last
        }
        else if ((is_key_last_pressed(KEY_SHIFT))&& !(is_key_pressed(KEY_SHIFT))){
            update_tag(robot, robot->chassis_tag_last, NULL, NULL);
        }
        
        /* sample of CTRL: [第一次按下] 切换开---[第二次按下] 切换关 */
        // if ((is_key_last_pressed(KEY_CTRL)) && !(is_key_pressed(KEY_CTRL)))
        // {
        //     static uint8_t upside_down_flag = 0;
        //     upside_down_flag = !upside_down_flag;
        //     if (upside_down_flag){
        //         update_tag(robot, C_CREEP_MODE, NULL, NULL);
        //     }
        //     else{
        //         if(robot->chassis_tag == C_CREEP_MODE /* || robot->gimbal_tag == G_XXX_MODE || robot->shooter_tag == S_XXX_MODE */){
        //             update_tag(robot, robot->chassis_tag_last, NULL, NULL);
        //         }
        //         else{
        //             update_tag(robot, C_CREEP_MODE, NULL, NULL);
        //             upside_down_flag = 1; 
        //         }// 这里判断是用于防止第一次按下ctrl后又去按别的模式 导致mode_last出现错误记录
        //     }
        // }
        /* 平衡步兵也许需要CTRL触发CREEP 其他车使用CTRL触发超电 简单的flag控制就行 */
        if ((is_key_last_pressed(KEY_CTRL)) && !(is_key_pressed(KEY_CTRL))){
            robot->use_supercap_flag = !robot->use_supercap_flag;
        }
        /*  ↑↑↑单按键控制 单个机器人姿态 启停↑↑↑    */

        /*  ↓↓↓多按键控制 单个机器人姿态 启停↓↓↓    */

        // *KEY X* 正反小陀螺开
        if (is_key_last_pressed(KEY_X) && !(is_key_pressed(KEY_X))){
            spin_direction = -spin_direction;
            update_tag(robot, C_SPIN_MODE, NULL, NULL);
        }
        // *KEY Q* 英雄狙击 默认同时开镜
        // if ((is_key_last_pressed(KEY_Q)) && !(is_key_pressed(KEY_Q)))
        // {
        //     static uint8_t upside_down_flag = 0;
        //     upside_down_flag = !upside_down_flag;
        //     if (upside_down_flag){
        //         update_tag(robot, C_SNIPE_MODE, NULL, NULL);
        //     }
        //     else{
        //         if(robot->chassis_tag == C_SNIPE_MODE /* || robot->gimbal_tag == G_XXX_MODE || robot->shooter_tag == S_XXX_MODE */){
        //             update_tag(robot, robot->chassis_tag_last, NULL, NULL);
        //         }
        //         else{
        //             update_tag(robot, C_SNIPE_MODE, NULL, NULL);
        //             upside_down_flag = 1; 
        //         }// 这里判断是用于防止第一次按下ctrl后又去按别的模式 导致mode_last出现错误记录
        //     }
        // }
        // *KEY Q* 底盘飞坡模式 同时允许冲刺 禁用快速掉头
        if (is_key_pressed(KEY_Q)){
            asr_supercap_enable_cmd();
            update_tag(robot, C_FLY_MODE, NULL, NULL);
            robot->use_supercap_flag = ENABLE;
        }
        // *KEY F* 按住开启冲刺 在飞坡模式下才能生效
        if (is_key_pressed(KEY_F)){
            if (robot->chassis_tag == C_FLY_MODE)
            robot->rush_flag = ENABLE;
            else robot->rush_flag = DISABLE;
        }
        else robot->rush_flag = DISABLE;
        // *KEY F* 点按快速掉头 在非飞坡模式下才能生效
        if (is_key_last_pressed(KEY_F) && !(is_key_pressed(KEY_F))){
            if (robot->chassis_tag != C_FLY_MODE){
                asr_run_cmd();
                update_tag(robot, NULL, G_QUICKTURN_MODE, NULL);
                // 完成掉头后会自动恢复到默认(人工)控制
                // 如有开镜 自动关镜  
            }
            
        }
        // *KEY R & MOUSE RIGHT* 切换自瞄目标
        if (rc_recv->mouse.press_right == MOUSE_PRESS && is_key_last_pressed(KEY_R) && !(is_key_pressed(KEY_R))){
            target_req_change = ENABLE;
        }
        else target_req_change = DISABLE;
        // *KEY R & !MOUSE RIGHT* 坦克模式
        if (rc_recv->mouse.press_right == MOUSE_NOT_PRESS && is_key_pressed(KEY_R)){
            update_tag(robot, C_TANK_MODE, NULL, NULL);
        }
        // *KEY V* 关摩擦轮
        if (is_key_pressed(KEY_V)){
            asr_shooter_stop_cmd();
            robot->use_fric_flag = DISABLE;
            update_tag(robot, NULL, NULL, S_STOP_MODE);
        }
        // *KEY B* 开摩擦轮 单发使能 打符
        if (is_key_pressed(KEY_B)){
            asr_shooter_start_cmd();
            robot->use_fric_flag = ENABLE;
            update_tag(robot, NULL, NULL, S_SINGLE_MODE);
        }
        // *KEY C* 开摩擦轮 连发使能 带火控
        if (is_key_pressed(KEY_C)){
            asr_shooter_start_cmd();
            robot->use_fric_flag = ENABLE;
            update_tag(robot, NULL, NULL, S_RESTRICT_MODE);
        }
        // *KEY E* 开摩擦轮 连发使能 不带火控
        if (is_key_pressed(KEY_E)){
            asr_shooter_start_cmd();
            robot->use_fric_flag = ENABLE;
            update_tag(robot, NULL, NULL, S_MULTIPLE_MODE);
        }
        // *KEY Z* 回到底盘默认状态
        if (is_key_pressed(KEY_Z)){
            update_tag(robot, C_DEFAULT_MODE, NULL, NULL);
        }
        // *KEY Z* & *KEY V* 开关弹舱盖
        if (is_key_pressed(KEY_Z) && is_key_pressed(KEY_V) &&
            is_key_last_pressed(KEY_Z && is_key_last_pressed(KEY_V))){
            if(robot->magazine_switch_flag == UNCONTROL)
            robot->magazine_switch_flag = MAG_ON;
            else
            robot->magazine_switch_flag = robot->magazine_switch_flag == MAG_ON?MAG_OFF:MAG_ON;
        }
        // *KEY Z* & *KEY R* 按住重置TRACKER
        if (is_key_pressed(KEY_Z) && is_key_pressed(KEY_R)){
            target_req_reset_tracker = ENABLE;    
        }
        else target_req_reset_tracker = DISABLE;
        // *KEY SHIFT* & *MOUSE LEFT* & *MOUSE RIGHT* 按住切换YAW CENTER
        if (is_key_pressed(KEY_V) && rc_recv->mouse.press_left && rc_recv->mouse.press_right){
            need_change_yaw_center = 1;
            robot->reset_flag = DISABLE;
            update_tag(robot,C_TANK_MODE,G_LAUNCH_MODE,NULL);
        }
        // *MOUSE LEFT* 摩擦轮启用时 左键按住开火 (单发下为点按)
        if(robot->use_fric_flag){
            if (robot->shooter_tag == S_SINGLE_MODE){
                // 这里为了增加灵敏度写了一大坨
                //static uint8_t self_locking_cnt = DISABLE; // 增加触发率的自锁限位
                if(rc_recv->mouse.press_left == MOUSE_PRESS && rc_recv->mouse.last_press_left == MOUSE_NOT_PRESS){
                    robot->fire_flag = ENABLE;
                    self_locking_cnt++; // 进入周期
                }
                // else robot->fire_flag = DISABLE;
                // if(self_locking_cnt < self_locking_cnt_max && rc_recv->mouse.press_left == MOUSE_PRESS && rc_recv->mouse.last_press_left == MOUSE_PRESS)
                //     robot->fire_flag = DISABLE;
                if(rc_recv->mouse.press_left == MOUSE_NOT_PRESS && rc_recv->mouse.last_press_left == MOUSE_PRESS){
                    if(self_locking_cnt > 0){
                        self_locking_cnt = DISABLE;
                    }
                    if(!(self_locking_cnt < self_locking_cnt_max)){
                        robot->fire_flag = ENABLE,self_locking_cnt = DISABLE;
                    }
                    self_locking_cnt = DISABLE; // 退出周期
                }
                // if(!(self_locking_cnt < self_locking_cnt_max) && (rc_recv->mouse.press_left == MOUSE_NOT_PRESS && rc_recv->mouse.last_press_left == MOUSE_PRESS))
                //     robot->fire_flag = ENABLE,self_locking_cnt = DISABLE;
                // if(rc_recv->mouse.press_left == MOUSE_NOT_PRESS && rc_recv->mouse.last_press_left == MOUSE_PRESS)
                //     self_locking_cnt = DISABLE; // 退出周期
            }
            else{
                if (rc_recv->mouse.press_left == MOUSE_PRESS){
                    robot->fire_flag = ENABLE;
                }
                else robot->fire_flag = DISABLE;
            }
        }
        else robot->fire_flag = DISABLE;
        // *MOUSE RIGHT* 右键按住自瞄
        if (rc_recv->mouse.press_right == MOUSE_PRESS){
            robot->use_vision_flag = ENABLE;
        }
        else robot->use_vision_flag = DISABLE;
    }
    #if 0
    else if(robot->robot_basic_mode == CHECK_MODE){
        // 检录模式怎么用键鼠? 这里可以扬了
    }
    #endif
    else if(robot->robot_basic_mode == AUTO_MODE){
        // 半自动模式
        // 鸽了
    }
}
/**
 * @brief 控制源为遥控器的机器人控制变量更新
 * @param rc_recv 遥控指令结构体指针
 * @param robot 机器人结构体指针
 * @retval None
 */
void update_robot_status_for_remoteControl(rc_ctrl_t *rc_recv, robot_t *robot){
    asr_cmds_t *asr = get_asr_cmds(); 
    robot->magazine_switch_flag = MAG_OFF; //遥控器下不断控关闭弹舱盖
    // robot->magazine_switch_flag = MAG_ON; // debug
    robot->use_vision_flag = ENABLE; // 遥控器下自瞄常驻 
    // 获取输入值
    rocker_rx += CLAMP((float)rc_recv->rc.ch1 * T_ACC_CNT / 660.0f - rocker_rx, S_CURVE_RX_ACC);
    rocker_ry += CLAMP((float)rc_recv->rc.ch0 * T_ACC_CNT / 660.0f - rocker_ry, S_CURVE_RY_ACC);
    rocker_lx = -((float)rc_recv->rc.ch3 * GAIN_RC_X / 660);
    rocker_ly = -((float)rc_recv->rc.ch2 * GAIN_RC_Y / 660);
    static bool is_switch_change = false;
    if(rc_recv->rc.switch_left != rc_recv->rc.last_switch_left)
        is_switch_change = true;
    else
        is_switch_change = false;
    if(robot->robot_basic_mode == NORMAL_MODE){
        
        /*  ↓↓↓左侧钮子开关控制开火↓↓↓    */
        if(robot->chassis_tag != C_DEFAULT_MODE)
            update_chassis_tag(robot, C_DEFAULT_MODE);

        if (rc_recv->rc.switch_left == RC_SW_DOWN){
            robot->fire_flag = DISABLE;
            robot->use_fric_flag = DISABLE;
            update_tag(robot,NULL,NULL,S_STOP_MODE);
            if(robot->shooter_tag != robot->shooter_tag_last && is_switch_change){
                asr_shooter_stop_cmd();
            }
        }
        else if (rc_recv->rc.switch_left == RC_SW_MID){
            robot->use_fric_flag = ENABLE;
            update_tag(robot,NULL,NULL,S_RESTRICT_MODE);
            if(robot->shooter_tag != robot->shooter_tag_last && is_switch_change){
                asr_shooter_start_cmd();
            }
            #if USE_DT7_WHEEL
            if(abs(rc_recv->wheel-1024) > 50){
                robot->fire_flag = ENABLE;
            }
            else{
                robot->fire_flag = DISABLE;
            }
            #else
            robot->fire_flag = DISABLE;
            #endif
        }
        else if (rc_recv->rc.switch_left == RC_SW_UP){
            robot->use_fric_flag = ENABLE;
            #if USE_DT7_WHEEL
            static bool restrict_lock = false;
            robot->use_fric_flag = ENABLE;
            update_tag(robot,NULL,NULL,S_SINGLE_MODE);
            if(robot->shooter_tag != robot->shooter_tag_last && is_switch_change){
                asr_shooter_start_cmd();
            }
            if(abs(rc_recv->wheel-1024) > 400){
                if(!restrict_lock){
                    robot->fire_flag = ENABLE;
                    restrict_lock = true;
                }
                // else robot->fire_flag = DISABLE;
            }
            else{
                robot->fire_flag = DISABLE;
                restrict_lock = false; // 解除限制
            }
            #else
            robot->fire_flag = ENABLE;
            #endif
        }
        /*  ↑↑↑左侧钮子开关控制开火↑↑↑    */
    }
    // ↓感觉写的有点bug
    else if(robot->robot_basic_mode == CHECK_MODE){
        robot->fire_flag = DISABLE;
        robot->use_fric_flag = DISABLE;
        if(rc_recv->rc.switch_left == RC_SW_DOWN){
            update_tag(robot,C_DEFAULT_MODE,G_DEFAULT_MODE,NULL);
        }
        else if(rc_recv->rc.switch_left == RC_SW_MID){
            if(rc_recv->rc.last_switch_left == RC_SW_DOWN)
                spin_direction = -spin_direction;
            update_tag(robot,C_SPIN_MODE,G_MANUAL_MODE,NULL); 
        }
        else if(rc_recv->rc.switch_left == RC_SW_UP){
            update_tag(robot,C_SPIN_MODE,G_PATROL_MODE,NULL);
        }
    }
    else if(robot->robot_basic_mode == AUTO_MODE){
        // 半自动模式
        // 格了
    }
}
robot_t *get_robot_ptr(void){return &robot;}

void update_chassis_tag(robot_t* robot, chassis_tag_t C_TAG){
    if(robot->chassis_tag != C_TAG) // 防刷
    robot->chassis_tag_last = robot->chassis_tag;
    robot->chassis_tag = C_TAG;
}
void update_gimbal_tag(robot_t* robot, gimbal_tag_t G_TAG){
    if(robot->gimbal_tag != G_TAG) // 防刷
    robot->gimbal_tag_last = robot->gimbal_tag;
    robot->gimbal_tag = G_TAG;
}
void update_shooter_tag(robot_t* robot, shooter_tag_t S_TAG){
    if(robot->shooter_tag != S_TAG) // 防刷
    robot->shooter_tag_last = robot->shooter_tag;
    robot->shooter_tag = S_TAG;
}
/**
 * @brief 机器人机构模式更新函数
 * @param robot 机器人结构体指针
 * @param C_TAG 新的底盘模式 为null时不更新
 * @param G_TAG 新的云台模式 为null时不更新
 * @param S_TAG 新的射击模式 为null时不更新
 * @retval None
 */
void update_tag(robot_t* robot, uint8_t C_TAG, uint8_t G_TAG, uint8_t S_TAG){
    if(C_TAG != NULL){update_chassis_tag(robot, (chassis_tag_t)C_TAG);}
    if(G_TAG != NULL){update_gimbal_tag(robot, (gimbal_tag_t)G_TAG);}
    if(S_TAG != NULL){update_shooter_tag(robot, (shooter_tag_t)S_TAG);}
}
