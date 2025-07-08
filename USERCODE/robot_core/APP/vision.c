/**
 * @file vision.c
 * @author Lizi
 * @brief 视觉数据对接
 *
 *
 * @copyright SCNU-PIONEER (c) 2022-2023
 *
 */
#include "vision.h"

#include <string.h>
#include <math.h>

#include "Base/referee.h"
#include "Devices/MOTOR/motor_headers.h"
#include "algorithm/ballistic.h"
#include "algorithm/imu_fusion.h"
#include "algorithm/cordic.h"
#include "algorithm/KF.h"

#include "algorithm/crc.h"
#include "algorithm/util.h"
#include "shooter.h"
#include "algorithm/filter.h"
#include "fire_ctrl.h"
#include "imu_bmi088.h"

#if VISION_INF_USB == 0
#include "Base/drv_uart.h"

int32_t vision_frame_offset = 0;
int32_t vision_receive_size = 0;

uint8_t vision_rx_buffer[VISION_CTRL_FRAME_LEN * 2]__attribute__((at(0x20000064)));

extern UART_HandleTypeDef VISION_UART_HANDLE;
#else

#include "usbd_cdc_if.h"

#endif

uint16_t vision_rx_lost = VISION_RX_LOST_MAX;
uint8_t vision_req_flag = 0;

vision_req_t vision_request;
vision_ctrl_t vision_ctrl_data;

float current_pitch, current_yaw;
float vision_pitch, vision_yaw;

#define MAX_PREDICT_T 0.6f
#define MAX_TARGET_LOST 60
#define SPIN_V_THRESHOLD 0.05f

float predict_time = 0.0f;
float cur_v0 = INIT_SHOOT_SPEED;
float est_x, est_y, est_z;
float aim_x, aim_y, aim_z;
float distance_xy;
extern uint64_t now_timestamp;
uint8_t game_is_play;
uint8_t target_req_change;
uint8_t target_req_reset_tracker;

target_spec_t target;
ballistic_sol_t solution;

one_vec_kf_t ks = {
	.x = INIT_SHOOT_SPEED,
	.p = 0.0f,
	.A = 1,
	.H = 1,
	.q = 0.02f, // 10e-6;  /* predict noise convariance */
	.r = 100.0f // 10e-5;  /* measure error convariance */
};

// anti-spinning variance
uint8_t shoot_flag = 0;
uint8_t spin_flag = 0;
// vision_ctrl_t spinning_target;

/**
 * @brief      return the self color for vision
 * @return     0 for red, others for blue
 */
uint8_t is_red_or_blue(void)
{
	if (game_robot_status.robot_id == 0 ||
		(game_robot_status.robot_id > 9 && game_robot_status.robot_id < 101) ||
		(game_robot_status.robot_id > 109))
		return COLOR_RED; // 裁判系统错误数据保护
	else
		return (game_robot_status.robot_id < 100);
}

// uint8_t get_vision_cmd(void){ return vision_ctrl_data.command;}

void set_vision_req_ang(float pitch, float yaw)
{
	current_pitch = pitch;
	current_yaw = yaw;
}

#if VISION_INF_USB == 0
HAL_StatusTypeDef vision_uart_dma_recv_init(void)
{
	vision_rx_lost = VISION_RX_LOST_MAX;

	__HAL_UART_ENABLE_IT(&(VISION_UART_HANDLE), UART_IT_IDLE);
	return uart_recv_dma_init(&(VISION_UART_HANDLE), vision_rx_buffer, VISION_CTRL_FRAME_LEN * 2);
}

void vision_uart_idle_handler(UART_HandleTypeDef *huart)
{
	if (huart->Instance == VISION_UART && __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		DMA_Stream_TypeDef *uhdma = huart->hdmarx->Instance;
		vision_receive_size = (int32_t)(2 * VISION_CTRL_FRAME_LEN - uhdma->NDTR) - vision_frame_offset;
		if (vision_receive_size == VISION_CTRL_FRAME_LEN || vision_receive_size == -VISION_CTRL_FRAME_LEN)
		{
			if (parse_vision_data(&vision_rx_buffer[vision_frame_offset], VISION_CTRL_FRAME_LEN) == VISION_DATA_NOERR)
			{
				vision_rx_lost = 0;
			}
			vision_frame_offset = (int32_t)(vision_frame_offset == 0) * VISION_CTRL_FRAME_LEN;
		}
		else if (vision_frame_offset != 0)
		{
			// some bytes lost, reset DMA buffer
			CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);
			__HAL_DMA_DISABLE(huart->hdmarx);
			vision_frame_offset = 0;
			uhdma->NDTR = (uint32_t)(VISION_CTRL_FRAME_LEN * 2);
			__HAL_DMA_CLEAR_FLAG(huart->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmarx));
			__HAL_DMA_CLEAR_FLAG(huart->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(huart->hdmarx));
			__HAL_DMA_CLEAR_FLAG(huart->hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(huart->hdmarx));
			__HAL_DMA_ENABLE(huart->hdmarx);
			SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
		}
	}
}
#else
float const_dl = SYS_DELAY;
uint8_t parse_vision_data(uint8_t *buf, uint16_t len)
{
	robot_t *robot = get_robot_ptr();
	if (buf[0] == VISION_UART_CTRL_SOF && len == VISION_CTRL_FRAME_LEN &&
		Verify_CRC16_Check_Sum(&buf[0], VISION_CTRL_FRAME_LEN))
	{
		memcpy((uint8_t *)&vision_ctrl_data, buf, VISION_CTRL_FRAME_LEN);

		float init_predict_time = 0.0f;
		if (vision_ctrl_data.target)
		{
			if (cur_v0 != 0.0f)
			{
				init_predict_time = target.x0 / (cur_v0 * cosf(robot->current_state->pitch_ang * (2.0f * PI / IMU_RANGE))) + const_dl;
			}
			else
				init_predict_time = const_dl;
			LIMIT_MIN_MAX(init_predict_time, 0.0f, MAX_PREDICT_T);
			predict_time = init_predict_time;
		}
		else
		{
			predict_time = 0.0f;
		}

		vision_rx_lost = 0;
		return VISION_DATA_NOERR;
	}
	else
		return VISION_DATA_ERR;
}
float *yaw_watch;
#endif
uint8_t vision_ctrl(float *pitch_ang, float *yaw_ang, float pitch_span,
                    float yaw_span) {
  if (vision_ctrl_data.target) {
    expected_preview_calc();

    if (pitch_ang != NULL && yaw_ang != NULL) {
      distance_xy = sqrtf(aim_x * aim_x + aim_y * aim_y);

      *yaw_ang = atan2f(aim_y, aim_x) * yaw_span / (2.0f * PI);
      yaw_watch = yaw_ang;
      // 装甲板中心至云台中心距离 distance_xy为 水平距离, z0为竖直距离
      // 观测静态云台朝向 可清晰判断该值极性
#if 1
      target.x0 = distance_xy;
      target.z0 = aim_z; // 非打真值
#else
      target.x0 = const_realx0;
      target.z0 = const_realz0; // 打真值
#endif
      projectile_solve(cur_v0, &target, &solution);

      if (solution.solution_num > 0) {
        if (solution.ang_solution1 < solution.ang_solution2) {
          *pitch_ang = solution.ang_solution1 * pitch_span / (2.0f * PI);
        } else {
          *pitch_ang = solution.ang_solution2 * pitch_span / (2.0f * PI);
        }
      } else {
        *pitch_ang = atan2f(aim_z, distance_xy) * pitch_span / (2.0f * PI);
      }
    }
    return VISION_OK;
  } else {
    return VISION_NOTARGET;
  }
}

/**
 * @brief  send upper data
 * @return none
 */

void send_vision_request(float current_roll, float current_pitch, float current_yaw)
{
	robot_t *robot = get_robot_ptr();
	vision_request.header = VISION_UART_REQ_SOF;
// vision send difference
	// vision_request.task_mode = buff_mode_flag;
	vision_request.roll = current_roll;
	vision_request.pitch = current_pitch;
	vision_request.yaw = current_yaw;
	vision_request.aim_x = est_x;
	vision_request.aim_y = est_y;
	vision_request.aim_z = est_z;
	vision_request.local_color = is_red_or_blue();
	//	if(game_status.game_progress == 4)
	//		vision_request.game_time = game_status.stage_remain_time;
	vision_request.s_timestamp = now_timestamp;
	if (game_status.game_progress == 4)
		game_is_play = 1;
	else
		game_is_play = 0;
	vision_request.is_play = game_is_play;
	vision_request.target_change = target_req_change;
	vision_request.reset_tracker = target_req_reset_tracker;
	if (robot->shooter_tag == S_SINGLE_MODE)
		vision_request.task_mode = 2;
	else
		vision_request.task_mode = 1;
	Append_CRC16_Check_Sum((uint8_t *)&vision_request, sizeof(vision_req_t));
	
	vision_request.game_time = game_status.stage_remain_time;

#if VISION_INF_USB == 0
	uart_send(&(VISION_UART_HANDLE), (uint8_t *)&vision_request, sizeof(vision_req_t));
#else
	CDC_Transmit_FS((uint8_t *)&vision_request, sizeof(vision_req_t));
#endif
}

/**
 * @brief  check if vision upper is offline
 * @return 0 for  vision upper online, others for offline
 */
uint32_t is_vision_offline(void) { return vision_rx_lost >= VISION_RX_LOST_MAX; }

/**
 * @brief  increment of vision_rx_lost counter, should be call after process of all rc data
 * @return none
 */
void inc_vision_rx_lost(void)
{
	if (vision_rx_lost < VISION_RX_LOST_MAX)
		vision_rx_lost++;
}

/**
 * @brief  check vision request flag
 * @return 1 for vision upper request, others for not receive
 * @note calling this function will set vision_rx_flag to zero
 */
uint8_t is_vision_req(void)
{
	uint8_t rx = vision_req_flag;
	vision_req_flag = 0;
	return rx;
}

void update_cur_v0(void)
{
	if (shoot_data.bullet_speed > 9.0f)
	{
		cur_v0 = mean_filter_2(shoot_data.bullet_speed); // INIT_SHOOT_SPEED;
	}
}

vision_ctrl_t *get_vision_ctrl_data_ptr(void)
{
	return &vision_ctrl_data;
}
