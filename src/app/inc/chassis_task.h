#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include <stdint.h>

typedef struct chassis_s
{
    float target_yaw;
    float last_yaw_raw;
    float current_yaw;
    int16_t total_turns;
} Chassis_t;

// Function prototypes
void Chassis_Task_Init(void);
void Chassis_Ctrl_Loop(void);

#endif // CHASSIS_TASK_H
