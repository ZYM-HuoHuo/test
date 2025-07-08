#ifndef _VISION_H
#define _VISION_H

#include "drv_conf.h"

#include HAL_INCLUDE

//typedef enum
//{
//  FALSE = 0,
//  TRUE = !FALSE
//} bool;

// vision request digital difference

typedef __packed struct
{
  uint8_t header; // 0x5A
  uint8_t local_color : 1;
  uint8_t task_mode : 2;
  bool reset_tracker : 1;
  uint8_t is_play : 1;
  bool target_change : 1;
  uint8_t reserved : 2;
  float roll;
  float pitch;
  float yaw; // 主要需要两个轴的角度数据
  float aim_x;
  float aim_y;
  float aim_z; // aim的三个参数主要可以提供给视觉调试时使用，可不发送数据
  uint16_t game_time;
  uint32_t s_timestamp;
  uint16_t checksum;
} vision_req_t;

typedef __packed struct
{
  uint8_t header; // 0xA5
  uint8_t target : 2;
  uint8_t id_num : 3;
  uint8_t armor_num : 3;
  float x, y, z; // 在世界坐标系（imu的坐标系中的位置），z的值为当前瞄准装甲板的高度
  float yaw;     // 当前瞄准的装甲板朝向（在imu坐标系中的角度）
  float vx;
  float vy;
  float vz;
  float v_yaw;          // 小陀螺速度
  float r1;             // 当前瞄准的装甲板与车身中心的水平距离
  float r2;             // 另一侧装甲板与车身中心的水平距离
  float dz;             // 另一侧装甲板的高度
  uint32_t r_timestamp; //(ms)frame capture time
  uint16_t first_phase; //(ms) speed_t offset
  uint16_t checksum;
} vision_ctrl_t;

// #define  r_timestamp checksum
// #define  first_phase checksum

/*是否使用USB，需要打开虚拟串口*/
#define VISION_INF_USB 1

#define VISION_UART_REQ_SOF 0x5A
#define VISION_UART_CTRL_SOF 0xA5
#define VISION_CTRL_FRAME_LEN (sizeof(vision_ctrl_t))

#define VISION_DATA_NOERR 0x00
#define VISION_DATA_ERR 0xFF
#define VISION_NOTARGET 0xFE
#define VISION_OK 0x00

#define VISION_RX_LOST_MAX 5

// system delay digital difference
#define SYS_DELAY 0.067f


enum e_color
{
  COLOR_BLUE = 0,
  COLOR_RED = 1
};

enum e_buff_status
{
  STATE_NONE = 0xEE,
  STATE_SMALL_BUF = 0xAA,
  STATE_LARGE_BUF = 0xBB
};

enum e_task_mode
{
  MODE_AUTO = 0,
  MODE_ROBOT = 1,
  MODE_BUFF = 2
};

enum e_shoot_mode
{
  NO_FIRE = 0xDD,
  FIRE = 0xFF
};

extern vision_ctrl_t vision_ctrl_data;
extern float distance_xy;
extern float est_x, est_y, est_z;
extern float aim_x, aim_y, aim_z;
extern float predict_time;

#define INIT_SHOOT_SPEED 27.2f

#if VISION_INF_USB == 0
#define VISION_UART_HANDLE huart1
#define VISION_UART USART1

void vision_uart_idle_handler(UART_HandleTypeDef *huart);
HAL_StatusTypeDef vision_uart_dma_recv_init(void);

#else
uint8_t parse_vision_data(uint8_t *buf, uint16_t len);
#endif

uint8_t get_vision_cmd(void);
// void set_vision_req_ang(float pitch, float yaw);
uint8_t vision_ctrl(float *pitch_ang, float *yaw_ang, float pitch_span,
                    float yaw_span);

void send_vision_request(float current_roll, float current_pitch, float current_yaw);

uint32_t is_vision_offline(void);
void inc_vision_rx_lost(void);

uint8_t get_armour_state(void);
void update_cur_v0(void);
uint8_t is_vision_req(void);

vision_ctrl_t *get_vision_ctrl_data_ptr(void);

// void get_spinning_state(void);

#endif
