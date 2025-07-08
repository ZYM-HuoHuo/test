#include "stdint.h"
#ifndef _REFEREE_CONF_H
#define _REFEREE_CONF_H

#define REFEREE_SYS_VERSION 2024.05.27

#define LEN_HEADER 5U
#define LEN_CMDID 2U
#define LEN_TAIL 2U

#define SOF_BYTE 0xA5U // ֡ͷ

#define CMDID_OFFSET 5U
#define CMDID_H 6U
#define CMDID_L 5U

#define CMDID_TABLE_COL 16U
#define CMDID_TABLE_ROW 4U

/* RFID������ */
#define CARD_ATTACK ((uint8_t)0x00)
#define CARD_PROTECT ((uint8_t)0x01)
#define CARD_BLOOD_RED ((uint8_t)0x02)
#define CARD_BLOOD_BLUE ((uint8_t)0x03)
#define CARD_HEAL_RED ((uint8_t)0x04)
#define CARD_HEAL_BLUE ((uint8_t)0x05)
#define CARD_COLD_RED ((uint8_t)0x06)
#define CARD_COLD_BLUE ((uint8_t)0x07)
#define CARD_FORT ((uint8_t)0x08)

/**
 * @enum CmdID
 * @brief cmd_id 命令码 ID 说明
 */
typedef enum
{
    ID_game_state = 0x0001,                         ///< 比赛状态数据，1Hz 周期发送      
    ID_game_result = 0x0002,                        ///< 比赛结果数据，比赛结束后发送
    ID_game_robot_HP = 0x0003,                      ///< 比赛机器人血量数据，1Hz 周期发送
    ID_event_data = 0x0101,                         ///< 场地事件数据，1Hz 周期发送
    ID_supply_projectile_action = 0x0102,           ///< 场地补给站动作标识数据，动作改变后发送
    ID_supply_projectile_booking = 0x0103,          ///< 请求补给站补弹数据，由参赛队发送，上限 10Hz。（RM 对抗赛尚未开放）
    ID_refee_alert = 0x0104,                        ///< 裁判警告数据，警告发生后发送
    ID_dart_countdown = 0x0105,                     ///< 飞镖发射口倒计时，1Hz 周期发送
    ID_game_robot_state = 0x0201,                   ///< 机器人状态数据，10Hz 周期发送
    ID_power_heat_data = 0x0202,                    ///< 实时功率热量数据，50Hz 周期发送
    ID_game_robot_pos = 0x0203,                     ///< 机器人位置数据，1Hz 发送给对应机器人
    ID_buff_musk = 0x0204,                          ///< 机器人增益数据，固定以 3Hz 频率发送
    ID_aerial_robot_energy = 0x0205,                ///< 空中支援时间数据，固定以 1Hz 频率发送
    ID_robot_hurt = 0x0206,                         ///< 伤害状态数据，伤害发生后发送
    ID_shoot_data = 0x0207,                         ///< 实时射击数据，子弹发射后发送
    ID_bullet_remaining_num = 0x0208,               ///< 允许发弹量，固定以 10Hz 频率发送对应机器人
    ID_RFID_status = 0x0209,                        ///< 机器人 RFID 状态，3Hz 周期发送 该RFID持有机器人
    ID_dart_client_data = 0x020A,                   ///< 飞镖选手端指令数据，固定以 3Hz 频率发送给己方飞镖机器人
    ID_game_ground_robot_pos_data = 0x020B,         ///< 地面机器人位置数据，固定以 1Hz 频率发送给己方烧饼机器人
    ID_radar_marking_data = 0x020C,                 ///< 雷达进度标记数据，固定以 1Hz 频率发送给己方雷达机器人
    ID_sentry_autonomous_syn = 0x020D,              ///< 哨兵机器人自主决策信息同步，固定以 1Hz 频率发送给己方哨兵机器人
    ID_radar_autonomous_syn = 0x020E,               ///< 雷达机器人自主决策信息同步，固定以 1Hz 频率发送给己方雷达机器人
    ID_robot_interaction_data = 0x0301,             ///< 机器人间交互数据，发送方触发发送，上限 10Hz
    ID_custom_robot_data = 0x0302,                  ///< 自定义控制器与机器人交互数据，发送方触发发送，频率上限为 30Hz
    ID_map_command = 0x0303,                        ///< 客户端小地图交互数据，触发发送
    ID_vt_data = 0x0304,                            ///< 键鼠遥控数据，固定 30Hz 频率发送
    ID_radarTarget_position_x = 0x0305,             ///< 选手端小地图接收雷达数据，频率上限为 10Hz
    ID_custormer_ctrl_to_client_data = 0x0306,      ///< 自定义控制器与选手端交互数据，发送方触发发送，频率上限为 30Hz
    ID_autoRobot_to_clientMap = 0x307,              ///< 选手端小地图接收半自动机器人数据，频率上限为 1Hz
    ID_robot_to_clientMap = 0x0308,                 ///< 选手端小地图接收机器人位置数据，频率上限为 3Hz
} CmdID;

typedef __packed struct
{
    uint8_t sof;
    uint16_t data_len;
    uint8_t seq;
    uint8_t crc8;
} frame_header_t;

/*  比赛状态数据：0x0001。发送频率：1Hz */
typedef __packed struct
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} ext_game_status_t;

/* 比赛结果数据：0x0002。发送频率：比赛结束后发送 */
typedef __packed struct
{
    uint8_t winner;
} ext_game_result_t;

/* 机器人血量数据：0x0003。发送频率：1Hz */
typedef __packed struct
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;

/* 场地事件数据：0x0101。发送频率：1Hz */
typedef __packed struct
{
    uint32_t event_type;
} ext_event_data_t;

/* 补给站动作标识：0x0102。发送频率：动作改变后发送, 发送范围：己方机器人 */
typedef __packed struct
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

/* 裁判警告信息：cmd_id (0x0104)。发送频率：己方警告发生后发送 */
typedef __packed struct
{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
} ext_referee_warning_t;

/* 飞镖发射口倒计时：cmd_id (0x0105)。发送频率：1Hz 周期发送，发送范围：己方机器人 */
typedef __packed struct
{
    uint8_t dart_remaining_time;
    uint16_t dart_info;
} ext_dart_remaining_time_t;

/* 比赛机器人状态：0x0201。发送频率：10Hz */
typedef __packed struct
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} ext_game_robot_status_t;

/*  实时功率热量数据：0x0202。发送频率：50Hz */
typedef __packed struct
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t chassis_power_buffer;
    uint16_t shooter_id1_17mm_cooling_heat;
    uint16_t shooter_id2_17mm_cooling_heat;
    uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

/* 机器人位置：0x0203。发送频率：10Hz */
typedef __packed struct
{
    float x;
    float y;
    float angle;
} ext_game_robot_pos_t;

/* 机器人增益：0x0204。发送频率：1Hz */
typedef __packed struct
{
    uint8_t recovery_buff;
    uint8_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
} ext_buff_t;

/* 0x0205 空中支援时间数据，固定以 1Hz 频率发送*/
typedef __packed struct
{
    uint8_t airforce_status;
    uint8_t time_remain;
} aerial_robot_energy_t;

/* 0x0206 伤害状态数据，伤害发生后发送*/
typedef __packed struct
{
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;

/* 0x0207 实时射击数据，子弹发射后发送*/
typedef __packed struct
{
    uint8_t bullet_type;
    uint8_t shooter_id;
    uint8_t bullet_freq;
    float bullet_speed;
} ext_shoot_data_t;

/* 0x0208 允许发弹量，固定以 10Hz 频率发送对应机器人*/
typedef __packed struct
{
    uint16_t bullet_remaining_num_17mm;
    uint16_t bullet_remaining_num_42mm;
    uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

/* 0x0209 机器人 RFID 状态，3Hz 周期发送 该RFID持有机器人*/
typedef __packed struct
{
    uint32_t rfid_status;
} ext_rfid_status_t;

/* 0x020A 飞镖选手端指令数据，固定以 3Hz 频率发送给己方飞镖机器人*/
typedef __packed struct
{
    uint8_t dart_launch_opening_status;
    uint8_t dart_attack_target;
    uint16_t target_change_time;
    uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;

/* 0x020B 地面机器人位置数据，固定以 1Hz 频率发送给己方烧饼机器人*/
typedef __packed struct
{
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float standard_5_x;
    float standard_5_y;
} ext_ground_robot_position_t;

// 0x020C 雷达进度标记数据，固定以 1Hz 频率发送给己方雷达机器人
typedef __packed struct
{
    uint8_t mark_hero_progress;
    uint8_t mark_engineer_progress;
    uint8_t mark_standard_3_progress;
    uint8_t mark_standard_4_progress;
    uint8_t mark_standard_5_progress;
    uint8_t mark_sentry_progress;
} ext_radar_mark_data_t;

// 0x020D 哨兵机器人自主决策信息同步，固定以 1Hz 频率发送给己方哨兵机器人
    typedef __packed struct
{
    uint32_t sentry_info;
} ext_sentry_info_t;

// 0x020E 雷达机器人自主决策信息同步，固定以 1Hz 频率发送给己方雷达机器人
typedef __packed struct
{
    uint8_t radar_info;
} ext_radar_info_t;

/* 0x0301 机器人间交互数据，发送方触发发送，上限 10Hz 根据data_cmd_id子内容的不同 user_data代表含义不同 */
typedef __packed struct 
{ 
    uint16_t data_cmd_id; 
    uint16_t sender_id; 
    uint16_t receiver_id; 
    uint8_t user_data[112]; 
} ext_robot_interaction_data_t;

    /*data_cmd_id:0x0200~0x02FF 内容数据段长度x<=112 机器人之间通信*/ 
    typedef __packed struct
    {
        uint8_t data[112];
    } robots_interaction_t;
    /*data_cmd_id:0x0100 选手端删除图层*/ 
    typedef __packed struct
    {
        uint8_t delete_type;
        uint8_t layer;
    } interaction_layer_delete_t;
    /*data_cmd_id:0x0101 选手端绘制一个图形*/ 
    typedef __packed struct 
    { 
        uint8_t figure_name[3]; 
        uint32_t operate_type:3; 
        uint32_t figure_type:3; 
        uint32_t layer:4; 
        uint32_t color:4; 
        uint32_t details_a:9; 
        uint32_t details_b:9; 
        uint32_t width:10; 
        uint32_t start_x:11; 
        uint32_t start_y:11; 
        uint32_t details_c:10; 
        uint32_t details_d:11; 
        uint32_t details_e:11; 
    }interaction_figure_t;
    /*data_cmd_id:0x0102 选手端绘制两个图形*/ 
    typedef __packed struct 
    { 
        interaction_figure_t _[2]; 
    }interaction_figure_2_t; 
    /*data_cmd_id:0x0103 选手端绘制五个图形*/
    typedef __packed struct 
    { 
        interaction_figure_t _[5]; 
    }interaction_figure_3_t; 
    /*data_cmd_id:0x0114 选手端绘制七个图形*/ 
    typedef __packed struct 
    { 
        interaction_figure_t _[7]; 
    }interaction_figure_4_t; 
    /*data_cmd_id:0x0110 选手端绘制字符图形*/ 
    typedef __packed struct 
    { 
        interaction_figure_t config; 
        uint8_t data[30]; 
    } interaction_figure_char_t; 
    /*data_cmd_id:0x0120 哨兵自主决策指令*/ 
    typedef __packed struct 
    { 
        uint32_t sentry_cmd; 
    } sentry_cmd_t; 
    /*data_cmd_id:0x0121 雷达自主决策指令*/ 
    typedef __packed struct 
    { 
        uint8_t radar_cmd; 
    } radar_cmd_t;
/* 0x0302 自定义控制器发送数据*/
typedef __packed struct 
{ 
    uint8_t data[30]; 
}ext_custom_robot_data_t;
/* 0x0303 客户端小地图交互数据，触发发送*/
typedef __packed struct 
{ 
    float target_position_x; 
    float target_position_y; 
    uint8_t cmd_keyboard; 
    uint8_t target_robot_id; 
    uint8_t cmd_source; 
}ext_map_command_t; 
/* 0x0304 键鼠遥控数据*/
typedef __packed struct 
{ 
    int16_t mouse_x; 
    int16_t mouse_y; 
    int16_t mouse_z; 
    int8_t left_button_down; 
    int8_t right_button_down; 
    uint16_t keyboard_value; 
    uint16_t reserved; 
}ext_remote_control_t;

/* 0x0305 选手端小地图接收雷达数据，频率上限为 10Hz*/
typedef __packed struct 
{ 
    uint16_t target_robot_id; 
    float target_position_x; 
    float target_position_y; 
}ext_map_robot_data_t; 

/* 0x0306 自定义控制器与选手端交互数据，发送方触发发送，频率上限为 30Hz */
typedef __packed struct 
{ 
    uint16_t key_value; 
    uint16_t x_position:12; 
    uint16_t mouse_left:4; 
    uint16_t y_position:12; 
    uint16_t mouse_right:4; 
    uint16_t reserved; 
}ext_custom_client_data_t; 

/* 0x0307 选手端小地图接收半自动机器人数据，频率上限为 1Hz*/
typedef __packed struct 
{ 
    uint8_t intention; 
    uint16_t start_position_x; 
    uint16_t start_position_y; 
    int8_t delta_x[49]; 
    int8_t delta_y[49]; 
    uint16_t sender_id; 
}ext_map_data_t; 

/* 0x0308 选手端小地图接收机器人位置数据，频率上限为 3Hz*/
typedef __packed struct 
{ 
    uint16_t sender_id; 
    uint16_t receiver_id; 
    uint8_t user_data[30]; 
}ext_custom_info_t; 

static uint8_t cmdid_len_table[CMDID_TABLE_ROW][CMDID_TABLE_COL] = {
    {0, sizeof(ext_game_status_t), sizeof(ext_game_result_t), sizeof(ext_game_robot_HP_t), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, sizeof(ext_event_data_t), sizeof(ext_supply_projectile_action_t), 0, sizeof(ext_referee_warning_t), sizeof(ext_dart_remaining_time_t), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, sizeof(ext_game_robot_status_t), sizeof(ext_power_heat_data_t), sizeof(ext_game_robot_pos_t), sizeof(ext_buff_t), sizeof(aerial_robot_energy_t), sizeof(ext_robot_hurt_t), sizeof(ext_shoot_data_t), sizeof(ext_bullet_remaining_t), sizeof(ext_rfid_status_t), sizeof(ext_dart_client_cmd_t), sizeof(ext_ground_robot_position_t), sizeof(ext_radar_mark_data_t), sizeof(ext_sentry_info_t), sizeof(ext_radar_info_t)},
    {0, sizeof(ext_robot_interaction_data_t), sizeof(ext_custom_robot_data_t), sizeof(ext_map_command_t), sizeof(ext_remote_control_t), sizeof(ext_map_robot_data_t), sizeof(ext_custom_client_data_t), sizeof(ext_map_data_t), sizeof(ext_custom_info_t), 0, 0, 0, 0, 0, 0, 0}};

#endif
