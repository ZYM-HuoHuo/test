#ifndef _BEHAVIOUR_H_
#define _BEHAVIOUR_H_

#include "drv_conf.h"
#include "Base/rc.h"

#include "algorithm/imu_fusion.h"

#define UNFINISH 0
#define FINISH 1
typedef enum{
  NORMAL_MODE = 1u,
  AUTO_MODE,
  CHECK_MODE,
}robot_basic_mode_t; //基本模式

typedef enum{
  DT7_CONTROLLER = 1u,
  VT_CONTROLLER,
  BOARDS_CONTROLLER,
  RADAR_CONTROLLER,
}robot_rc_t; //控制源

/**
 * @note 为了有所区分 故把控制模式的每个TAG加上对应机构的前缀 C = chassis G = gimbal S = shooter
 * @note 默认的东西应为1u 0u是留给空值(NULL)的
 * @note 非发射机构的机构上控时应为LAUNCH_MODE
 */
typedef enum {
  C_STOP_MODE = 0u,             //底盘停止
  C_FOLLOW_MODE = 1u,           //底盘跟随云台 
  C_TANK_MODE,                  //坦克模式 底盘不跟随云台  
  C_FLY_MODE,                   //飞坡模式 可能需要调整底盘姿态
  C_SIDEWAYS_MODE,              //侧身模式 侧对保护装甲板
  C_RUSH_MODE,                  //冲刺模式 会自动开超电
  C_SPIN_MODE,                  //小陀螺模式 底盘不跟随云台
  C_SNIPE_MODE,                 //狙击模式 英雄特供
  C_CREEP_MODE,                 //匍匐模式 平衡步兵特供
  C_LAUNCH_MODE,                //缓启动
}chassis_tag_t; //底盘姿态
typedef enum {
  G_STOP_MODE = 0u,             //云台停止
  G_MANUAL_MODE = 1u,           //人工控制
  G_PATROL_MODE,                //巡逻控制
  G_QUICKTURN_MODE,             //快速回头控制
  G_LAUNCH_MODE,                //缓启动
}gimbal_tag_t; //云台姿态

typedef enum {
  S_STOP_MODE = 0u,             //发射断控   
  S_MULTIPLE_MODE = 1u,         //连发模式 连发 FUCK
  S_SINGLE_MODE,                //单发模式 单发 BUFF
  S_RESTRICT_MODE,              //火控模式 受限
}shooter_tag_t; //发射模式

extern chassis_tag_t C_DEFAULT_MODE;
extern gimbal_tag_t G_DEFAULT_MODE;

/**
 * @note 机器人整机状态量 坐标系->主枪管方向为正方向<-
 */
typedef struct _robot_state_t{
  float v_x,v_y,w_z;          //机器人对应三轴速度
  float yaw_ang,pitch_ang;    //云台两自由度角
  float roll_ang;             //机体roll角
}robot_state_t;

typedef struct{
  imu_data_raw_t raw_data; // 单位 m/s^2  rad/s
  imu_data_fp_t fp_data; // 单位 m/s^2  rad/s
  eular_t eular; // 单位 rad
  float temperature; // 单位 ℃
}imu_data_t;

/**
 * @note robot 中枚举是0的为默认值
 * @note robot_state在behaviour中改变 *机器人控制模式* *机器人控制源* 
 * @note 解释一下为什么不搞个G_VISION_MODE 开挂的时候没有瞄上目标实际上也属于MANUAL_MODE 故使用flag控制
 */
// tips 尽量别塞太多东西 怕把堆栈写爆 到时就HARDFAULT咯
typedef struct {
    bool robot_control_flag  ;    //机器人泄力状态
    bool robot_survival_flag ;    //机器人存活状态

    bool use_keyboard_flag   ;    //键盘启用使能标志
    bool use_supercap_flag   ;    //超电启用使能标志
    bool use_radar_flag      ;    //雷达启用使能标志
    bool use_fric_flag       ;    //摩擦轮启用使能标志
    bool use_vision_flag     ;    //挂 启用使能标志

    bool fire_flag           ;    //!!!开火!!!
    bool rush_flag           ;    //!!!开创!!!  

    bool reset_flag          ;    //相应机构没有缓启动完成不会主动改变任何tag

    enum _magazine_switch_flag{
      MAG_OFF = 0u,
      MAG_ON,
      UNCONTROL,
    }magazine_switch_flag;                  //弹仓补充机构开关

    robot_basic_mode_t robot_basic_mode;    //机器人基础控制模式
    robot_rc_t rc_sc;                       //机器人控制源

    chassis_tag_t chassis_tag,chassis_tag_last;
    gimbal_tag_t gimbal_tag,gimbal_tag_last;
    shooter_tag_t shooter_tag,shooter_tag_last;

    robot_state_t *expected_state, *current_state;
    
    imu_data_t *imu_data;
    imu_data_t *ext_imu;
    imu_data_t *chassis_imu;
#if ENABLE_SNIPE_MODE == 1
    struct shooter_offset_t {
    float yaw;
    float pitch;
    }   shooter_offset,            // 吊射模式自瞄手动纠偏值
        outpost_spin_armor_offset, // 前哨站旋转装甲板偏置
        outpost_top_armor_offset,  // 前哨站顶部装甲板偏置
        base_mid_armor_offset,     // 基地顶部装甲板
        other_offset;
#endif
}robot_t;

extern int8_t spin_direction;

void behaviour_init(robot_t* state);
void behaviour_ctrl_loop(rc_ctrl_t *rc_recv, robot_t* state);

void motor_switch(uint8_t switch_flag);

void magazine_ctrl_loop(enum _magazine_switch_flag s_flag);

void select_rc_sc(robot_t *robot);
void killed_protect(robot_t *robot);
void select_control_flag(rc_ctrl_t *rc_recv, robot_t *robot);
void select_robot_basic_mode(rc_ctrl_t *rc_recv, robot_t *robot);

void update_robot_status_for_keyboard(rc_ctrl_t *rc_recv, robot_t *robot);
void update_robot_status_for_remoteControl(rc_ctrl_t *rc_recv, robot_t *robot);

robot_t *get_robot_ptr(void);

void update_chassis_tag(robot_t* robot, chassis_tag_t C_TAG);
void update_gimbal_tag(robot_t* robot, gimbal_tag_t G_TAG);
void update_shooter_tag(robot_t* robot, shooter_tag_t S_TAG);
void update_tag(robot_t* state, uint8_t C_TAG, uint8_t G_TAG, uint8_t S_TAG); //一键更新的偷懒函数


#define GAIN_MOUSE_X 0.057f		//鼠标横向速度增益
#define GAIN_MOUSE_Y 0.0513f	//鼠标纵向速度增益
#define GAIN_RC_X 9.5f
#define GAIN_RC_Y 9.5f


#define S_CURVE_RX_ACC 2.0f   
#define S_CURVE_RY_ACC 2.0f

#if ENABLE_SNIPE_MODE == 1
/**
 * @brief 视觉辅助吊射模式函数,其实就是将吊射模式yaw/pitch角度值给自瞄去纠偏
 */
static inline snipe_mode_with_vision_ctrl(rc_ctrl_t *rc_recv) {
  extern attack_target_type_t attack_target_type;
  extern robot_t robot;
  extern float k_snipe_yaw, k_snipe_pitch;

  ///<<< 吊射击打目标选择 >>>///
  switch (attack_target_type) {
  case outpost_spin_armor:            // 击打前哨站旋转装甲板
    robot.shooter_offset.pitch = //
        robot.outpost_spin_armor_offset.pitch;
    robot.shooter_offset.yaw = //
        robot.outpost_spin_armor_offset.yaw;
    k_snipe_yaw = Gimbal_snipe_yaw_level2;
    k_snipe_pitch = Gimbal_snipe_pitch_level2;
    break;
  case outpost_top_armor:             // 击打前哨站顶部装甲板
    robot.shooter_offset.pitch = //
        robot.outpost_top_armor_offset.pitch;
    robot.shooter_offset.yaw = //
        robot.outpost_top_armor_offset.yaw;
    k_snipe_yaw = Gimbal_snipe_yaw_level2;
    k_snipe_pitch = Gimbal_snipe_pitch_level2;
    break;
  case base_mid_armor:                // 击打基地顶部装甲板
    robot.shooter_offset.pitch = //
        robot.base_mid_armor_offset.pitch;
    robot.shooter_offset.yaw = //
        robot.base_mid_armor_offset.yaw;
    k_snipe_yaw = Gimbal_snipe_yaw_level2;
    k_snipe_pitch = Gimbal_snipe_pitch_level2;
    break;
  default:
    robot.shooter_offset.pitch = robot.other_offset.pitch;
    robot.shooter_offset.yaw = robot.other_offset.yaw;
    k_snipe_yaw = Gimbal_snipe_yaw_level2;
    k_snipe_pitch = Gimbal_snipe_pitch_level2;
    break;
  }

  ///<<< 修正值输入 >>>///
  extern vision_ctrl_t vision_ctrl_data;
  float xc2 = vision_ctrl_data.x * vision_ctrl_data.x;
  float yc2 = vision_ctrl_data.y * vision_ctrl_data.y;
  float zc2 = vision_ctrl_data.z * vision_ctrl_data.z;
  float distance_xyz = 1;
  distance_xyz = sqrtf(xc2 + yc2 + zc2);
  if (distance_xyz <= 0.5)
    distance_xyz = 0.5;

  _vec_t *vec = _curve(rc_recv);
  robot.shooter_offset.pitch -=
      k_snipe_pitch / 1000.f / distance_xyz * (-vec->y) / fs_tim_freq;
  robot.shooter_offset.yaw -=
      k_snipe_yaw / 1000.f / distance_xyz * (-vec->x) / fs_tim_freq;

  CLAMP(robot.shooter_offset.pitch, deg2rad(15));
  CLAMP(robot.shooter_offset.yaw, deg2rad(15));
}

/**
 * @brief 基地模式启用
 */
static inline base_mode_with_vision_ctrl(rc_ctrl_t *rc_recv) {
  extern bool base_mode_enable;
  extern bool had_find_base;
  static bool base_mode_enable_flag = false;
  if (IS_KEY_PRESS(rc_recv, KEY_B) == true &&
      IS_KEY_LAST_PRESS(rc_recv, KEY_B) == false) {
    base_mode_enable_flag = !base_mode_enable_flag;
  }
  if (base_mode_enable_flag == true) {
    base_mode_enable = true;
  } else {
    base_mode_enable = false;
    had_find_base = false;
  }
}

/**
 * @brief 手动调整曝光
 */
static inline base_mode_with_vision_ctrl(rc_ctrl_t *rc_recv) {
  extern bool base_mode_enable;
  extern bool had_find_base;
  static bool base_mode_enable_flag = false;
  if (IS_KEY_PRESS(rc_recv, KEY_B) == true &&
      IS_KEY_LAST_PRESS(rc_recv, KEY_B) == false) {
    base_mode_enable_flag = !base_mode_enable_flag;
  }
  if (base_mode_enable_flag == true) {
    base_mode_enable = true;
  } else {
    base_mode_enable = false;
    had_find_base = false;
  }
}

#endif

#endif // !_BEHAVIOUR_H_
