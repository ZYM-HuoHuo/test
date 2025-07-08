/**
 * @file fire_ctrl.c
 * @author zy
 * @brief 预测+火控
 *
 *
 * @copyright SCNU-PIONEER (c) 2023-2024
 *
 */
#include "fire_ctrl.h"

#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "vision.h"
#include "gimbal.h"

#include "algorithm/imu_fusion.h"
#include "algorithm/cordic.h"
#include "algorithm/filter.h"

#include "algorithm/util.h"
#include "math.h"

uint8_t aim_status = 0, armor_choose = 1;
uint8_t suggest_fire = 1;

float yaw_motor_res_speed = YAW_MOTOR_RES_SPEED;

extern uint32_t now_timestamp;

#define VISION_DATA_ASSIGNMENT                                         \
	uint8_t target = vision_ctrl_data.target;                          \
	uint8_t id_num = vision_ctrl_data.id_num;                          \
	float yaw = vision_ctrl_data.yaw;                                  \
	float r1 = vision_ctrl_data.r1;                                    \
	float r2 = vision_ctrl_data.r2;                                    \
	float xc = vision_ctrl_data.x;                                     \
	float yc = vision_ctrl_data.y;                                     \
	float zc = vision_ctrl_data.z;                                     \
	float vx = vision_ctrl_data.vx;                                    \
	float vy = vision_ctrl_data.vy;                                    \
	float vz = vision_ctrl_data.vz;                                    \
	float vyaw = vision_ctrl_data.v_yaw; 							   \
	float dz = vision_ctrl_data.dz;                                    \
	uint8_t armor_num = vision_ctrl_data.armor_num;                    \
	/*buff*/                                                           \
	float b_x = vision_ctrl_data.x;                                    \
	float b_y = vision_ctrl_data.y;                                    \
	float b_z = vision_ctrl_data.z;                                    \
	float spd_a = vision_ctrl_data.vx;                                 \
	float spd_b = vision_ctrl_data.vy;  							   \
	float spd_w = vision_ctrl_data.vz;                                 \
	uint32_t r_timestamp = vision_ctrl_data.r_timestamp;               \
	uint16_t first_phase_1 = vision_ctrl_data.first_phase;

float speed=0.0f;
float cnt_1 = 0.f;
float cnt_2;
float cnt_ang;
/**
 * @brief 选板和预测允范围函数
 * @retval None
 */
void expected_preview_calc(void)
{
	float armor_x = 0.f, armor_y = 0.f, armor_z = 0.f;
	float center_x = 0.f, center_y = 0.f;

	VISION_DATA_ASSIGNMENT;
	
	float allow_fire_ang_max = 0.f, allow_fire_ang_min = 0.f;

	// aim_status judge
	if (target == 1)
	{
		aim_status = ARMOR;

		// Prediction
		float predict_time_final = (float)(now_timestamp - r_timestamp )/ 1000 + predict_time;
		
		xc = xc + predict_time_final * vx;
		yc = yc + predict_time_final * vy;
		zc = zc + predict_time_final * vz;
		float predict_yaw = yaw + predict_time_final * vyaw;
		float center_theta = atan2(yc, xc);

		uint8_t use_1 = 1;
		float diff_angle = 2 * PI / armor_num;

		for (size_t i = 0; i < armor_num; i++)
		{
			float armor_yaw = predict_yaw + i * diff_angle;
			float yaw_diff = get_delta_ang_pi(armor_yaw, center_theta);
			if (fabsf(yaw_diff) < diff_angle  / 2)
			{
				armor_choose = 1;
				
				// Only 4 armors has 2 radius and height
				float r = r1;
				armor_z = zc;
				if (armor_num == 4)
				{
					r = use_1 ? r1 : r2;
					armor_z = use_1 ? zc : (zc + dz);
				}

				// Robot state to armor
				armor_x = xc - r * cos(armor_yaw);
				armor_y = yc - r * sin(armor_yaw);

				center_x = xc - r * cos(center_theta);
				center_y = yc - r * sin(center_theta);
				
				// Calculate angle of advance
				
				float armor_z_next = zc;
				if (armor_num == 4)
				{
					r = !use_1 ? r1 : r2;
					armor_z_next = !use_1 ? zc : (zc + dz);
				}
				float next_armor_yaw = armor_yaw - sign(vyaw) * diff_angle; // TODO: use future span to calculate target delta angle
				float armor_x_next = xc - r * cos(next_armor_yaw);
				float armor_y_next = yc - r * sin(next_armor_yaw);

				float yaw_motor_delta = get_delta_ang_pi(atan2(armor_y, armor_x), atan2(armor_y_next, armor_x_next));
				float angle_of_advance = fabsf(yaw_motor_delta) / yaw_motor_res_speed * fabsf(vyaw) / 2;
				cnt_ang = angle_of_advance;
				
				float est_yaw;
				if(sign(vyaw) * yaw_diff < diff_angle / 2 - angle_of_advance || angle_of_advance > diff_angle / 4)
				{
					est_x = armor_x;
					est_y = armor_y;
					est_z = armor_z;
					est_yaw = armor_yaw;
				}
				else
				{
					est_x = armor_x_next;
					est_y = armor_y_next;
					est_z = armor_z_next;
					est_yaw = next_armor_yaw;
				}

				// Calculate fire control params
				float armor_w;
				if (armor_num == 2 || id_num == 1)
					armor_w = LARGE_ARMOR_WIDTH;
				else
					armor_w = SMALL_ARMOR_WIDTH;
				float ax = est_x - 0.5f * armor_w * sin(est_yaw);
				float ay = est_y + 0.5f * armor_w * cos(est_yaw);
				float bx = est_x + 0.5f * armor_w * sin(est_yaw);
				float by = est_y - 0.5f * armor_w * cos(est_yaw);
				float angle_a = atan2(ay, ax);
				float angle_b = atan2(by, bx);
				float angle_c = atan2(est_y, est_x);
				allow_fire_ang_max = angle_c - angle_b;
				allow_fire_ang_min = angle_c - angle_a;
				
				break;
			}
			else
			{
				armor_choose = 0;
			}
			use_1 = !use_1;
		}
	}
	else if(target == 2)
	{
		aim_status = BUFF;
		float theta = yaw;
		float t0 = (float)(r_timestamp + first_phase_1) /1000;
		float t1 = (float)(now_timestamp + first_phase_1) / 1000 + predict_time;

		if (spd_w == 0)
		{
			theta += spd_b * (t1 - t0);
		}
		else
		{
			theta += spd_a / spd_w * (cosf(spd_w * t0) - cosf(spd_w * t1)) + spd_b * (t1 - t0);
			speed=spd_a*sinf(spd_w*t0)+spd_b;
		}
		float predict_distance = sqrtf(powf(b_x, 2) + powf(b_y, 2));
		LIMIT_MIN_MAX(predict_distance, 7.f, 10.f);
		est_x = b_x + BUFF_R * (sinf(theta) * b_y / predict_distance);
		est_y = b_y + BUFF_R * (-sinf(theta) * b_x / predict_distance);
		est_z = b_z + BUFF_R * cosf(theta);
		
	}
	// Arrange gimbal expected point
	uint8_t use_center_fire = USE_CENTER_FIRE;
	if(use_center_fire){
		aim_x = center_x;
		aim_y = center_y;
	}
	else{
		aim_x = est_x;
		aim_y = est_y;
	}
	aim_z = est_z; // hit armors with two heights

	fire_ctrl(allow_fire_ang_max, allow_fire_ang_min, target);
}

/**
 * @brief 产suggest_fire
 * @param allow_fire_ang_max 目标最大偏角(正向偏角)
 * @param allow_fire_ang_min 目标最小偏角(负向偏角)
 * @param target 目标类型
 * @retval None
 */
void fire_ctrl(float allow_fire_ang_max,  float allow_fire_ang_min, uint8_t target)
{
	robot_t *robot = get_robot_ptr();
	cnt_2 = allow_fire_ang_max - allow_fire_ang_min;
	if (target == 0 || armor_choose ==0)
	{
		suggest_fire = 0;
	}
	else if (target == 1)
	{
		// control_delta_angle单位应该为rad 需要*PI
		// float control_delta_angle = get_delta_ang_pi(robot->expected_state->yaw_ang*PI/4096,robot->current_state->yaw_ang*PI/4096);
		float control_delta_angle = get_delta_ang_pi( robot->expected_state->yaw_ang/ 4096, robot->current_state->yaw_ang / 4096);
		suggest_fire = (control_delta_angle < allow_fire_ang_max &&
									 control_delta_angle > allow_fire_ang_min);
	}
	else if (target == 2)
	{
		// TODO: Buff fire control
		suggest_fire = 1;
	}
	else
	{
		// ERROR
		suggest_fire = 0;
	}
}
