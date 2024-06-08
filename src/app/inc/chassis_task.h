#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include <stdint.h>

typedef struct chassis_s
{
    float target_yaw;
    float target_yaw_speed;
    float last_yaw_raw;
    float current_yaw;
    int16_t total_turns;
    float target_vel;
    float wheel_x_turning_offset;
    float turning_radius;
    float centripetal_force;
} Chassis_t;

// Function prototypes
void Chassis_Task_Init(void);
void Chassis_Ctrl_Loop(void);

#endif // CHASSIS_TASK_H
