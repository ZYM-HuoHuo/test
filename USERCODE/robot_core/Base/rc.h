#ifndef _REMOTE_CTRL_H
#define _REMOTE_CTRL_H

#include "drv_conf.h"
#include HAL_INCLUDE

/* ----------------------Internal Data------------------------------*/

/* --------------- RC Channel Definition-----------------*/
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
#define RC_CH_VALUE_RANGE ((uint16_t)660)
/* ----------RC Switch Definition------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
/* ----------PC Key Definition---------------------- ------*/
#define MOUSE_PRESS ((uint8_t)1)
#define MOUSE_NOT_PRESS ((uint8_t)0)
#define MOUSE_MAX_ABS ((int16_t)32768)

#define KEY_W ((uint16_t)0x01 << 0)
#define KEY_S ((uint16_t)0x01 << 1)
#define KEY_A ((uint16_t)0x01 << 2)
#define KEY_D ((uint16_t)0x01 << 3)
#define KEY_SHIFT ((uint16_t)0x01 << 4)
#define KEY_CTRL ((uint16_t)0x01 << 5)
#define KEY_Q ((uint16_t)0x01 << 6)
#define KEY_E ((uint16_t)0x01 << 7)

#define KEY_R ((uint16_t)0x01 << 8)
#define KEY_F ((uint16_t)0x01 << 9)
#define KEY_G ((uint16_t)0x01 << 10)
#define KEY_Z ((uint16_t)0x01 << 11)
#define KEY_X ((uint16_t)0x01 << 12)
#define KEY_C ((uint16_t)0x01 << 13)
#define KEY_V ((uint16_t)0x01 << 14)
#define KEY_B ((uint16_t)0x01 << 15)
/* ------------------Defined Errors------------------*/
#define RC_NO_ERROR 0
#define RC_CH_ERROR 0xFF
#define RC_VERIFY_ERR 0xFE
#define RC_RX_LOST_MAX ((uint8_t)5)

/* ------------------Defined Marcos------------------*/

#define IS_KEY_PRESS(CODE, KEY) (((CODE) & (KEY)) == (KEY))

/* ------------------Data Struct Data Struct -------*/
#define RC_FRAME_LENGTH 18

/**
 * @struct rc_ctrl_t
 * @brief all remote control data
 */
typedef struct
{
	struct
	{
		int16_t ch0;			   ///< joystick channel 1 (11bit, 364-1684)
		int16_t ch1;			   ///< joystick channel 2 (11bit, 364-1684)
		int16_t ch2;			   ///< joystick channel 3 (11bit, 364-1684)
		int16_t ch3;			   ///< joystick channel 4 (11bit, 364-1684)
		uint8_t switch_left;	   ///< left switch(2bit, 0-2)
		uint8_t switch_right;	   ///< right switch(2bit, 0-2)
		uint8_t last_switch_left;  ///< last left switch(2bit, 0-2)
		uint8_t last_switch_right; ///< last right switch(2bit, 0-2)
	} rc;
	struct
	{
		int16_t x;				  ///< mouse velocity of x axis(16bit, -32767-32767)
		int16_t y;				  ///< mouse velocity of y axis(16bit, -32767-32767)
		int16_t z;				  ///< mouse velocity of z axis(16bit, -32767-32767)
		uint8_t press_left;		  ///< the left key of mouse(8bit,0 or 1)
		uint8_t press_right;	  ///< the right key of mouse(8bit,0 or 1)
		uint8_t last_press_left;  ///< the last press left key of mouse(8bit,0 or 1)
		uint8_t last_press_right; ///< the last press right key of mouse(8bit,0 or 1)
	} mouse;
	struct
	{
		uint16_t keycode;	   ///< the flags current pressed key, use marco IS_KEY_PRESS to parse
		uint16_t last_keycode; ///< the flags last pressed key, use marco IS_KEY_PRESS to parse
	} keyboard;

	int16_t wheel;
} rc_ctrl_t;

enum key_codes
{
	W = 1,
	S = 2,
	A = 3,
	D = 4,
	SHIFT = 5,
	CTRL = 6,
	Q = 7,
	E = 8,
	R = 9,
	F = 10,
	G = 11,
	Z = 12,
	X = 13,
	C = 14,
	V = 15,
	B = 16
};

#if USE_VT_RC_UART == 1

#include "referee_conf.h"
#include "referee.h"

typedef __packed struct
{
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	uint8_t left_button_down;
	uint8_t right_button_down;
	uint16_t keyboard_value;
	uint16_t reserved;
} vt_rc_data_t;

typedef __packed struct
{
	frame_header_t header;
	uint16_t vt_rc_cmdid;
	vt_rc_data_t data;
	uint16_t crc16;
} vt_rc_frame_t;

#define VT_FRAME_CMDID (0x304)
#define VT_RC_FRAME_LEN (sizeof(vt_rc_frame_t))

HAL_StatusTypeDef vt_rc_recv_dma_init(void);

#endif
#define VT_RC_RX_MAX_LOST ((uint8_t)5)

#if USE_RC_UART == 1
HAL_StatusTypeDef rc_recv_dma_init(void);
#endif

rc_ctrl_t get_rc_data(void);
rc_ctrl_t *get_rc_data_ptr(void);
rc_ctrl_t *get_dt7_data_ptr(void);
rc_ctrl_t *get_vt_data_ptr(void);

uint32_t is_key_pressed(uint16_t key);
uint32_t is_key_last_pressed(uint16_t key);
void update_rc_last_key(uint8_t rc_sc);

uint32_t is_rc_offline(void);
void inc_rc_rx_lost(void);

#if USE_VT_RC_UART == 1 || USE_RC_UART == 1
void rc_uart_idle_handle(UART_HandleTypeDef *huart);
#endif

#endif
