#ifndef ROBOT_H
#define ROBOT_H

#include <stdint.h>

typedef struct {
  uint8_t enabled;
  uint8_t safely_started;
  uint8_t spintop_mode;
  uint8_t autoaiming_enabled;
  uint8_t UI_enabled;
  float chassis_x_speed;
  float chassis_y_speed;
  float chassis_omega;

  float chassis_power_buffer[500];
  uint16_t chassis_power_index;
  uint16_t chassis_power_count;
  float chassis_avg_power;
  float chassis_total_power;
  float power_increment_ratio;

  float gimbal_pitch_angle;
  float gimbal_yaw_angle;
  float vx;
  float vy;
  float vx_keyboard;
  float vy_keyboard;
  float chassis_move_speed_ratio;

  /* Wheel Legged */
  float chassis_height;
  uint8_t wheel_facing_mode;
  uint8_t gimbal_switching_dir_pending;
} Robot_State_t;

typedef struct {
  uint8_t prev_B;
  uint8_t prev_G;
  uint8_t prev_V;
  uint8_t prev_C;
  uint8_t prev_F;
  uint8_t prev_Q;
  uint8_t prev_E;
  uint8_t prev_R;
  uint8_t prev_left_switch;
  uint8_t prev_right_switch;
} Key_Prev_t;

void Robot_Init(void);
void Robot_Ctrl_Loop(void);

extern Robot_State_t g_robot_state;

#endif // ROBOT_H
