#ifndef _CAN_BSP_H
#define _CAN_BSP_H


#include "main.h"

typedef FDCAN_HandleTypeDef hcan_t;

extern void FDCAN1_Config(void);
extern void FDCAN2_Config(void);
extern void FDCAN3_Config(void);
extern uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);

/* CAN send and receive ID */
typedef enum
{
    CAN_GIMBAL_ID   = 0x307,
		CAN_CHASSIS_ID  = 0x308,
    CAN_SUPERCAP_ID = 0x309,
	
    CAN_CHASSIS_ALL3508_ID = 0x200,
    CAN_3508_L_ID = 0x201,
    CAN_3508_R_ID = 0x202
} can_msg_id_e;

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
	  int16_t delta_ecd;
	  int16_t round; 
}   motor_measure_t;

extern const motor_measure_t *get_3508motor_measure_point(uint8_t i);
extern void CAN_cmd_chassis_All3508(int16_t motor1,int16_t motor2);

#endif

