#include "struct_typedef.h"
typedef struct {
    float       curr_yaw;
    float       curr_pitch;
    float       curr_omega;
    uint8_t     state;
    uint8_t     autoaim;
    uint8_t     enemy_color;
} __attribute__((packed)) OutputData;


typedef struct {
    float       shoot_yaw;
    float       shoot_pitch;
    uint8_t     fire;
    uint8_t     target_id;
} __attribute__((packed)) InputData;

typedef struct  {
    uint8_t     sof;
    uint8_t     crc8;
} __attribute__((packed))FrameHeader;

typedef struct {
    uint16_t    crc16;
} __attribute__((packed))FrameTailer ;

typedef struct  {            
    FrameHeader frame_header;
    InputData   input_data;
    FrameTailer frame_tailer;   
} __attribute__((packed))RECEIVE_DATA;

typedef struct  {         
    FrameHeader frame_header;
    OutputData  output_data;
    FrameTailer frame_tailer;
	
} __attribute__((packed))SEND_DATA;

#define RECEIVE_DATA_SIZE 12
#define SEND_DATA_SIZE 19
#define FRAME_HEADER      0X7B //Frame_header 
#define FRAME_TAIL        0X7D //Frame_tail   
#define FRAME_HEADER_SOF  0xA5  
#define FRAME_HEADER_CRC8 0xFF  
#define FRAME_TAILER_CRC16_init 0xFFFF  
void Self_aim_task(void const *pvParameters);
InputData *get_selfaim_data();