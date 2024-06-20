#include "motor_task.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "mf_motor.h"
#include "robot_param.h"

extern CAN_Instance_t *g_board_comm_part1;
extern CAN_Instance_t *g_board_comm_part2;
extern uint8_t g_board_comm_first_part_sending_pending;
extern uint8_t g_board_comm_second_part_sending_pending;

void Motor_Task_Loop() {
    DJI_Motor_Send();
    MF_Motor_Send();
    DM_Motor_Send();

#ifndef MASTER
    if (g_board_comm_first_part_sending_pending == 1) {
        CAN_Transmit(g_board_comm_part1);
        g_board_comm_first_part_sending_pending = 0;
    }
    if (g_board_comm_second_part_sending_pending == 1) {
        CAN_Transmit(g_board_comm_part2);
        g_board_comm_second_part_sending_pending = 0;
    }
#endif
}

