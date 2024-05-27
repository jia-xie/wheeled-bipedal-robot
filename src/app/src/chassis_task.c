#include "chassis_task.h"

#include "robot.h"
#include "mf_motor.h"
#include "remote.h"
#include "imu_task.h"

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;
extern IMU_t g_imu;

MF_Motor_Handle_t *g_motor_lf, *g_motor_rf, *g_motor_lb, *g_motor_rb;

void Chassis_Hip_Motor_Torq_Ctrl(float torq_lf, float torq_rf, float torq_lb, float torq_rb); 

void Chassis_Hip_Motor_Torq_Ctrl(float torq_lf, float torq_rf, float torq_lb, float torq_rb)
{
    MF_Motor_TorqueCtrl(g_motor_lf, torq_lf / MG8016_TORQ_CONSTANT * (2048.0f / 16.5f));
    // MF_Motor_TorqueCtrl(g_motor_rf, torq_rf / MG8016_TORQ_CONSTANT * (2048.0f / 16.5f));
    // MF_Motor_TorqueCtrl(g_motor_lb, torq_lb / MG8016_TORQ_CONSTANT * (2048.0f / 16.5f));
    // MF_Motor_TorqueCtrl(g_motor_rb, torq_rb / MG8016_TORQ_CONSTANT * (2048.0f / 16.5f));
}

void Chassis_Task_Init()
{
    // Initialize motors
    MF_Motor_Config_t motor_config = {
        .can_bus = 1,
        .tx_id = 0x141,
        .rx_id = 0x141,
        .control_mode = 0,
        .kp_ang = 0.0f,
        .ki_ang = 0.0f,
        .kp_vel = 0.0f,
        .ki_vel = 0.0f,
        .kp_torq = 0.0f,
        .ki_torq = 0.0f,
        .pos_offset = 0.0f,
    };
    g_motor_lf = MF_Motor_Init(motor_config);
    motor_config.tx_id = 0x142;
    motor_config.rx_id = 0x142;
    g_motor_lb = MF_Motor_Init(motor_config);
    motor_config.tx_id = 0x143;
    motor_config.rx_id = 0x143;
    g_motor_rb = MF_Motor_Init(motor_config);
    motor_config.tx_id = 0x144;
    motor_config.rx_id = 0x144;
    g_motor_rf = MF_Motor_Init(motor_config);
}

void Chassis_Ctrl_Loop()
{
    if (g_robot_state.enabled) {
        Chassis_Hip_Motor_Torq_Ctrl(0.5f, 0, 0, 0);
        // Chassis_Hip_Motor_Torq_Ctrl(0.5f, 0, 0, 0);
    }
    else {
        Chassis_Hip_Motor_Torq_Ctrl(0.0f, 0, 0, 0);
    }
}