#ifndef __UART_BSP_H__
#define __UART_BSP_H__

#include "main.h"

#define BUFF_SIZE	25
#define DBUS_MAX_LEN     (50)
#define DBUS_BUFLEN      (18)
#define DBUS_HUART       huart5


/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)

#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)

extern uint8_t   dbus_buf[DBUS_BUFLEN];

typedef __packed struct
{
  int16_t ch0;   //右摇杆水平  右正左负 （-660  660） 
  int16_t ch1;   //右摇杆竖直  上正下负 （-660  660） 
  int16_t ch2;   //左摇杆水平  右正左负 （-660  660） 
  int16_t ch3;   //右摇杆竖直  上正下负 （-660  660） 
  int16_t roll;  //左上滑轮    下正上负
  uint8_t sw1;   //左拨杆 下 中 上 2 3 1
  uint8_t sw2;   //右拨杆 下 中 上 2 3 1
} rc_info_t;
 
#define rc_Init   \
{                 \
		0,            \
		0,            \
		0,            \
		0,            \
		0,            \
		0,            \
		0,            \
}
extern rc_info_t rc_ctrl;

extern const rc_info_t *get_remote_control_point(void);
extern void dbus_uart_init(void);//DBUS串口初始化
extern void rc_callback_handler(rc_info_t *rc_ctrl, uint8_t *buff);
extern void uart_receive_handler(UART_HandleTypeDef *huart);
#endif /*__UART_BSP_H__ */

