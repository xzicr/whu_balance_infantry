/**
  ****************************(C) WHU_BALANCE_GIMBAL****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  *  V2.0.0     Nov-16-2025     xzicr           增加控制模块
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C)WHU_BALANCE_GIMBAL****************************
  */

#include "gimbal_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "detect_task.h"
#include "remote_control.h"
#include "INS_task.h"
#include "pid.h"
#include "Self_aim.h"
#include "stm32f4xx_hal.h"
#include "usart.h"

#define rc_deadband_limit(input, output, dealine)    \
  {                                                  \
    if ((input) > (dealine) || (input) < -(dealine)) \
    {                                                \
      (output) = (input);                            \
    }                                                \
    else                                             \
    {                                                \
      (output) = 0;                                  \
    }                                                \
  }


#define gimbal_total_pid_clear(gimbal_clear)                                               \
  {                                                                                        \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
    PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
                                                                                           \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
    PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
  }

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;

#endif

#define WINDOW_SIZE 10
float yaw_buffer[WINDOW_SIZE];
int data_index = 0;

float sliding_average(float new_yaw)
{
  yaw_buffer[data_index] = new_yaw;
  data_index = (data_index + 1) % WINDOW_SIZE;
  float sum = 0;
  for (int i = 0; i < WINDOW_SIZE; i++)
    sum += yaw_buffer[i];
  return sum / WINDOW_SIZE;
}
uint8_t start;
uint8_t close;

/*--------------云台控制所有相关数据----------------*/
//云台数据结构体
gimbal_control_t gimbal_control;

//自瞄 转向标志位
uint8_t  aimflag = 0, turnflag = 0;

//PID参数
static const fp32 Pitch_angle_pid_1[3] = {PITCH_GYRO_ABSOLUTE_PID_KP_1, PITCH_GYRO_ABSOLUTE_PID_KI_1, PITCH_GYRO_ABSOLUTE_PID_KD_1};



/*------------底盘数据------------------*/
chassis_data_t chassis_data;
InputData *Self_aim_data;
first_order_filter_type_t chassis_self_aim_yaw;
const static fp32 chassis_self_aim_yaw_filter[1] = {CHASSIS_ACCEL_Y_NUM};


//func
static void gimbal_init(gimbal_control_t *init);
void leg_control_init(chassis_data_t *leg_contorl);
void chassis_rc_to_control_vector(gimbal_control_t *gimbal_control_set, chassis_data_t *chassis_data);
void rc_control(gimbal_control_t *gimbal_control_set,chassis_data_t *chassis_data);
void key_control(gimbal_control_t *gimbal_control_set,chassis_data_t *chassis_data);
void yaw_set(gimbal_control_t *gimbal_control_set,chassis_data_t *chassis_data);

static void gimbal_set_mode(gimbal_control_t *set_mode);
static void gimbal_feedback_update(gimbal_control_t *feedback_update);
static void gimbal_set_control(gimbal_control_t *set_control);
static void gimbal_control_loop(gimbal_control_t *control_loop);
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 intergral_limit, fp32 kp, fp32 ki, fp32 kd);
static void gimbal_PID_clear(gimbal_PID_t *pid_clear);
static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);







void gimbal_task(void const *pvParameters)
{
  vTaskDelay(GIMBAL_TASK_INIT_TIME);

  //云台初始化
  gimbal_init(&gimbal_control);

  //腿部控制初始化
  leg_control_init(&chassis_data);

	//射击初始化
  shoot_Init();

  while (1)
  {
    //射击控制
    gimbal_control.shoot = shoot_control_loop();

    //设置底盘控制量
    chassis_rc_to_control_vector(&gimbal_control, &chassis_data); 

    // 设置云台控制模式
    gimbal_set_mode(&gimbal_control);                 
    
    // 云台数据反馈
    gimbal_feedback_update(&gimbal_control);     
    
    // 设置云台PITCH轴目标角度
    gimbal_set_control(&gimbal_control);     
    
     // 云台控制PID计算
    gimbal_control_loop(&gimbal_control);    
    
    gimbal_control.shoot = shoot_control_loop();

    CAN_cmd_gimbal(0, -gimbal_control.gimbal_pitch_motor.gimbal_motor_angle_pid_1.out,0,0);

    CAN_cmd_chassis(gimbal_control.shoot->shoot_left_given_current,gimbal_control.shoot->shoot_right_given_current, 0, 0);
    vTaskDelay(GIMBAL_CONTROL_TIME);
  }
}



static void gimbal_init(gimbal_control_t *init)
{
  //初始化陀螺仪 遥控器  自瞄数据
  init->INS = get_INS();
  init->gimbal_INT_angle_point = get_INS_angle_point();
  init->gimbal_INT_gyro_point = get_gyro_data_point();
  init->gimbal_rc_ctrl = get_remote_control_point();
  Self_aim_data = get_selfaim_data();

  //模式初始化
  init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_OFF;
  chassis_data.shoot_mode = 0;
  chassis_data.chassis_mode = 0;
  init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
  init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;
  chassis_data.yaw_angle_set = init->gimbal_yaw_motor.absolute_angle;

  //PID初始化
  PID_init(&init->gimbal_pitch_motor.gimbal_motor_angle_pid_1, PID_POSITION, Pitch_angle_pid_1, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT_1, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT_1);

  gimbal_feedback_update(init);
  
}

void leg_control_init(chassis_data_t *leg_contorl)
{
  leg_contorl->high_set=0;
}

static void gimbal_set_mode(gimbal_control_t *set_mode)
{
  if (set_mode == NULL)
  {
    return;
  }

  if (set_mode->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_INIT)
  {
    static uint16_t init_time = 0;
    static uint16_t init_stop_time = 0;
    if (fabs(set_mode->gimbal_pitch_motor.absolute_angle - INIT_PITCH_SET) < GIMBAL_INIT_ANGLE_ERROR)
    {
      if (init_stop_time < GIMBAL_INIT_STOP_TIME)
      {
        init_stop_time++;
      }
    }
    else
    {
      if (init_time < GIMBAL_INIT_TIME)
      {
        init_time++;
      }
    }

    // 超过初始化最大时间，或者已经稳定到中值一段时间，退出初始化状态开关打下档，或者掉线
    if (init_time < GIMBAL_INIT_TIME && init_stop_time < GIMBAL_INIT_STOP_TIME &&
        !switch_is_down(set_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]) && !toe_is_error(DBUS_TOE))
    {
      return;
    }
    else
    {
      init_stop_time = 0;
      init_time = 0;
    }
  }
  if (set_mode->gimbal_rc_ctrl->rc.s[0] == 0 || toe_is_error(DBUS_TOE))
  {
    set_mode->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_OFF;
  }
  else if (set_mode->gimbal_rc_ctrl->rc.s[0] == 1 || set_mode->gimbal_rc_ctrl->rc.s[1] == 2)
  {
    set_mode->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
  }

  // 判断进入init状态机
  if (set_mode->gimbal_pitch_motor.last_gimbal_motor_mode == GIMBAL_MOTOR_OFF && set_mode->gimbal_pitch_motor.gimbal_motor_mode != GIMBAL_MOTOR_OFF)
  {
    set_mode->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_INIT;
  }
  set_mode->gimbal_pitch_motor.last_gimbal_motor_mode = set_mode->gimbal_pitch_motor.gimbal_motor_mode;
}

static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
  if (feedback_update == NULL)
  {
    return;
  }
  // 云台数据更新
  feedback_update->gimbal_pitch_motor.relative_angle = feedback_update->gimbal_pitch_motor.gimbal_motor_measure->last_ecd;
  feedback_update->gimbal_pitch_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);
  feedback_update->gimbal_pitch_motor.motor_gyro = *(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);
  feedback_update->gimbal_yaw_motor.absolute_angle = INS.YawTotalAngle;
  // 键鼠数据获取
  feedback_update->last_press_l = feedback_update->press_l;
  feedback_update->press_l = feedback_update->gimbal_rc_ctrl->mouse.press_l;
  feedback_update->lastkeyboard = feedback_update->keyboard;
  feedback_update->keyboard = feedback_update->gimbal_rc_ctrl->key.v;
  // 自瞄数据获取
  feedback_update->gimbal_pitch_motor.last_self_aim_pitch_angle = feedback_update->gimbal_pitch_motor.self_aim_pitch_angle;
  feedback_update->gimbal_pitch_motor.self_aim_pitch_angle = Self_aim_data->pitch / 3.14 * 180;
  feedback_update->gimbal_yaw_motor.last_self_aim_yaw_angle = feedback_update->gimbal_yaw_motor.self_aim_yaw_angle;
  feedback_update->gimbal_yaw_motor.self_aim_yaw_angle = sliding_average(Self_aim_data->yaw / 3.14 * 180);
}




static void gimbal_set_control(gimbal_control_t *set_control)
{
  static int16_t pitch_channel = 0;
  rc_deadband_limit(set_control->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);
  if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO && aimflag == 1)
  {
    set_control->gimbal_pitch_motor.absolute_angle_set += pitch_channel * PITCH_RC_SEN + set_control->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN * 0.5;
    if (set_control->gimbal_pitch_motor.absolute_angle_set < -26)
    {
      set_control->gimbal_pitch_motor.absolute_angle_set = -26;
    }
    else if (set_control->gimbal_pitch_motor.absolute_angle_set > 26)
    {
      set_control->gimbal_pitch_motor.absolute_angle_set = 26;
    }
    else
    {
      set_control->gimbal_pitch_motor.absolute_angle_set = set_control->gimbal_pitch_motor.absolute_angle_set;
    }
  }
  else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO && aimflag == 0)
  {
    set_control->gimbal_pitch_motor.absolute_angle_set += pitch_channel * PITCH_RC_SEN + set_control->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN;
    if (set_control->gimbal_pitch_motor.absolute_angle_set < -26)
    {
      set_control->gimbal_pitch_motor.absolute_angle_set = -26;
    }
    else if (set_control->gimbal_pitch_motor.absolute_angle_set > 26)
    {
      set_control->gimbal_pitch_motor.absolute_angle_set = 26;
    }
    else
    {
      set_control->gimbal_pitch_motor.absolute_angle_set = set_control->gimbal_pitch_motor.absolute_angle_set;
    }
  }
  /* 初始化云台pitch轴角度 */
  else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_INIT)
  {
    set_control->gimbal_pitch_motor.absolute_angle_set = INIT_PITCH_SET;
  }
}


static void gimbal_control_loop(gimbal_control_t *control_loop)
{
  if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_OFF)
  {
    gimbal_control.gimbal_pitch_motor.gimbal_motor_angle_pid_1.out=0;
  }
  else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
  {
    PID_calc(&control_loop->gimbal_pitch_motor.gimbal_motor_angle_pid_1, control_loop->gimbal_pitch_motor.absolute_angle, control_loop->gimbal_pitch_motor.absolute_angle_set);
  }
  else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_INIT)
  {
    PID_calc(&control_loop->gimbal_pitch_motor.gimbal_motor_angle_pid_1, control_loop->gimbal_pitch_motor.absolute_angle, control_loop->gimbal_pitch_motor.absolute_angle_set);
  }
}

static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor)
{
  if (gimbal_motor == NULL)
  {
    return;
  }
  // 角度环，速度环串级pid调试
  gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
  gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
  // 控制值赋值
  gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
  if (pid == NULL)
  {
    return;
  }
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;

  pid->err = 0.0f;
  pid->get = 0.0f;

  pid->max_iout = max_iout;
  pid->max_out = maxout;
}

static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
  fp32 err;
  if (pid == NULL)
  {
    return 0.0f;
  }
  pid->get = get;
  pid->set = set;

  err = set - get;
  pid->err = rad_format(err);
  pid->Pout = pid->kp * pid->err;
  pid->Iout += pid->ki * pid->err;
  pid->Dout = pid->kd * error_delta;
  abs_limit(pid->Iout, pid->max_iout);
  pid->out = pid->Pout + pid->Iout + pid->Dout;
  abs_limit(pid->out, pid->max_out);
  return pid->out;
}


static void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear)
{
  if (gimbal_pid_clear == NULL)
  {
    return;
  }
  gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
  gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}
void chassis_rc_to_control_vector(gimbal_control_t *gimbal_control_set, chassis_data_t *chassis_data)
{
  /* --------------进入函数前提条件------------------ */
  if (gimbal_control_set == NULL)
  {
    return;
  }
  if (chassis_data->chassis_mode == CHASSIS_MODE_INIT)
  {
    if ( fabs(gimbal_control_set->gimbal_pitch_motor.absolute_angle - INIT_PITCH_SET) > GIMBAL_INIT_ANGLE_ERROR)
    {
      return;
    }
  }
  //遥控控制
  rc_control(gimbal_control_set,chassis_data);

  //键盘控制
  key_control(gimbal_control_set,chassis_data);
  
  //yaw轴设置更新
  yaw_set(gimbal_control_set,chassis_data);

  chassis_data->yaw_angle = gimbal_control_set->gimbal_yaw_motor.absolute_angle;
  chassis_data->yaw_gyro = gimbal_control_set->gimbal_yaw_motor.motor_gyro;
}

chassis_data_t *get_chassis_data_point()
{
  return &chassis_data;
}
gimbal_control_t *get_gimbal_data()
{
  return &gimbal_control;
}

/**
 * @brief 遥控控制输入
 * @param[in] gimbal_control_set
 * @param[out] chassis_data
 * @retval 
 * @note 
 */
void rc_control(gimbal_control_t *gimbal_control_set,chassis_data_t *chassis_data)
{
  //模式设置
  if (gimbal_control_set->gimbal_rc_ctrl->rc.s[0] == 0)
  {
    chassis_data->chassis_mode = CHASSIS_MODE_OFF;
  }
  else if (gimbal_control_set->gimbal_rc_ctrl->rc.s[0] == 1)
  {
    chassis_data->chassis_mode = CHASSIS_MODE_NO_FOLLOW_GIMBAL;
  }
  else if (gimbal_control_set->gimbal_rc_ctrl->rc.s[0] == 2)
  {
    chassis_data->chassis_mode = CHASSIS_MODE_FOLLOW_GIMBAL;
  }
  int16_t vx_channel, vy_channel;
  fp32 vx_set_channel, vy_set_channel;
  fp32 high_control;
  rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
  rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
  rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[HEIGHT_CHANNEL], high_control, RC_DEADBAND);
  vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
  vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;
  
  chassis_data->vx_set = vx_set_channel;
  chassis_data->vy_set = vy_set_channel;
  chassis_data->wz_set = -CHASSIS_WZ_RC_SEN * gimbal_control_set->gimbal_rc_ctrl->rc.ch[CHASSIS_WZ_CHANNEL];
  chassis_data->high_set += high_control*HIGH_SEN;
  chassis_data->high_set = fp32_constrain(chassis_data->high_set,0.0,0.34);
  chassis_data->jump_flag = gimbal_control_set->gimbal_rc_ctrl->rc.s[2]==1;
}

/**
 * @brief 键盘控制输入
 * @param[in] gimbal_control_set
 * @param[in] chassis_data
 * @retval 
 * @note 
 */
void key_control(gimbal_control_t *gimbal_control_set,chassis_data_t *chassis_data)
{
    if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_W)
  {
    chassis_data->vx_set = 1.5;
  }
  else if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_S)
  {
    chassis_data->vx_set = -1.5;
  }
  else if (gimbal_control_set->keyboard& KEY_PRESSED_OFFSET_A)
  {
    chassis_data->vy_set = 1.5;
  }
  else if (gimbal_control_set->keyboard& KEY_PRESSED_OFFSET_D)
  {
    chassis_data->vy_set = -1.5;
  }

  if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_W && gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_SHIFT)
  {
    chassis_data->vx_set = 3.0;
  }
  else if (gimbal_control_set->keyboard& KEY_PRESSED_OFFSET_S && gimbal_control_set->keyboard& KEY_PRESSED_OFFSET_SHIFT)
  {
    chassis_data->vx_set = -3.0;
  }
  else if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_A && gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_SHIFT)
  {
    chassis_data->vy_set = 3.0;
  }
  else if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_D && gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_SHIFT)
  {
    chassis_data->vy_set = -3.0;
  }

  // if(gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_B&& !(gimbal_control_set->lastkeyboard & KEY_PRESSED_OFFSET_B))
  // {
  //   chassis_data->jump_flag = 1;
  // }
  // else
  // {
  //   chassis_data->jump_flag = 0;
  // }

  if(gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_F&& gimbal_control_set->lastkeyboard & KEY_PRESSED_OFFSET_F)
  {
    chassis_data->high_set = 0.34f;
  }
  else if(gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_C&& gimbal_control_set->lastkeyboard & KEY_PRESSED_OFFSET_C)
  {
    chassis_data->high_set = 0.1f;
  }

  if(gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_Q&&gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_SHIFT)  
  {
    chassis_data->high_set += 0.05f;
    fp32_constrain(chassis_data->high_set,0.1,0.34);
  }
  else if(gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_E&&gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_SHIFT)  
  {
    chassis_data->high_set -= 0.05f;
    fp32_constrain(chassis_data->high_set,0.1,0.34);
  }

  if(gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_V&&(gimbal_control_set->lastkeyboard & KEY_PRESSED_OFFSET_V) == 0)
  {
    aimflag = !aimflag;
  }

  if (gimbal_control_set->keyboard & KEY_PRESSED_OFFSET_R && (gimbal_control_set->lastkeyboard & KEY_PRESSED_OFFSET_R) == 0)
  {
    turnflag = 1;
  }

  chassis_data->shoot_mode = shoot_control.shoot_mode ;

}


/**
 * @brief yaw轴设置
 * @param[in] gimbal_control_set
 * @param[out] chassis_data
 * @retval 
 * @note 
 */
void yaw_set(gimbal_control_t *gimbal_control_set,chassis_data_t *chassis_data)
{
  int16_t yaw_channel = 0;
  if (chassis_data->chassis_mode == CHASSIS_MODE_INIT)
  {
    chassis_data->yaw_angle_set = chassis_data->yaw_angle;
  }
  if (chassis_data->chassis_mode == CHASSIS_MODE_OFF)
  {
    chassis_data->yaw_angle_set = chassis_data->yaw_angle;
    chassis_data->high_set = 0;
  }
  rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
  if ((chassis_data->chassis_mode == CHASSIS_MODE_FOLLOW_GIMBAL || chassis_data->chassis_mode == CHASSIS_MODE_NO_FOLLOW_GIMBAL) && aimflag == 0 && turnflag == 0)
  {
    chassis_data->yaw_angle_set -= yaw_channel * YAW_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN;
  }
  else if ((chassis_data->chassis_mode == CHASSIS_MODE_FOLLOW_GIMBAL || chassis_data->chassis_mode == CHASSIS_MODE_NO_FOLLOW_GIMBAL) && aimflag == 1)
  {
    chassis_data->yaw_angle_set = gimbal_control_set->gimbal_yaw_motor.self_aim_yaw_angle ;
  }
  if ((chassis_data->chassis_mode == CHASSIS_MODE_FOLLOW_GIMBAL || chassis_data->chassis_mode == CHASSIS_MODE_NO_FOLLOW_GIMBAL) && turnflag == 1)
  {
    chassis_data->yaw_angle_set += 180;
    turnflag = 0;
  }

}



