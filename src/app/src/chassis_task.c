#include "chassis_task.h"

#include "robot.h"
#include "mf_motor.h"
#include "remote.h"
#include "imu_task.h"
#include "bsp_can.h"
#include "five_bar_leg.h"

/* Statistics */
#define UP_ANGLE_ODD (-48.0f)
#define UP_ANGLE_EVEN (180.0f - (-48.0f))
#define UP_1 (58396.0f)
#define UP_2 (17889.0f)
#define UP_3 (51440.0f)
#define UP_4 (22877.0f)

#define DOWN_ANGLE_ODD (87.0f)
#define DOWN_ANGLE_EVEN (180.0f - 87.0f)
#define DOWN_1	(33540.0f)
#define DOWN_2	(42464.0f)
#define DOWN_3	(26586.0f)
#define DOWN_4	(47774.0f)

#define LQR11 (0.0f)
#define LQR12 (0.0f)
#define LQR13 (0.0f)
#define LQR14 (0.0f)
#define LQR15 (0.0f)
#define LQR16 (0.0f)
#define LQR21 (0.0f)
#define LQR22 (0.0f)
#define LQR23 (0.0f)
#define LQR24 (0.0f)
#define LQR25 (0.0f)
#define LQR26 (0.0f)

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;
extern IMU_t g_imu;

MF_Motor_Handle_t *g_motor_lf, *g_motor_rf, *g_motor_lb, *g_motor_rb;
Leg_t g_leg_left, g_leg_right;
PID_t g_pid_left_leg_length;
PID_t g_pid_right_leg_length;

void _get_leg_statistics();
void _wheel_leg_estimation();
void _leg_length_controller(float chassis_height);
void _lqr_balancce_controller();
void _vmc_torq_calc();

void Chassis_Hip_Motor_Torq_Ctrl(float torq_lf, float torq_rf, float torq_lb, float torq_rb)
{
    int16_t torq1 = torq_lf / MG8016_TORQ_CONSTANT * (2048.0f / 16.5f);
    int16_t torq2 = torq_lb / MG8016_TORQ_CONSTANT * (2048.0f / 16.5f);
    int16_t torq3 = torq_rb / MG8016_TORQ_CONSTANT * (2048.0f / 16.5f);
    int16_t torq4 = torq_rf / MG8016_TORQ_CONSTANT * (2048.0f / 16.5f);

    MF_Motor_Broadcast_Torq_Ctrl(1, torq1, torq2, torq3, torq4);
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

    MF_Motor_Broadcast_Init(1);
}

void Chassis_Ctrl_Loop()
{
    _wheel_leg_estimation();
    if (g_robot_state.enabled) {
        _leg_length_controller(g_robot_state.chassis_height);
        _lqr_balancce_controller();
        _vmc_torq_calc();
    }
    else {
        Chassis_Hip_Motor_Torq_Ctrl(0.0f, 0, 0, 0);
    }
}

void _wheel_leg_estimation()
{
    _get_leg_statistics();
    Leg_ForwardKinematics(&g_leg_left, g_leg_left.phi1, g_leg_left.phi4, g_leg_left.phi1_dot, g_leg_left.phi4_dot);
    Leg_ForwardKinematics(&g_leg_right, g_leg_right.phi1, g_leg_right.phi4, g_leg_right.phi1_dot, g_leg_right.phi4_dot);
}

void _get_leg_statistics()
{
    g_leg_left.phi1 = (g_motor_lb->stats->angle - DOWN_2) * ((UP_ANGLE_EVEN - DOWN_ANGLE_EVEN)/(UP_2 - DOWN_2)) + DOWN_ANGLE_EVEN;
    // g_leg_left.phi1_dot = g_motor_lb->stats->velocity;
    g_leg_left.phi4 = (g_motor_lf->stats->angle - DOWN_1) * ((UP_ANGLE_ODD - DOWN_ANGLE_ODD)/(UP_1 - DOWN_1)) + DOWN_ANGLE_ODD;
    // g_leg_left.phi4_dot = g_motor_lf->stats->velocity;

    g_leg_right.phi1 = (g_motor_rf->stats->angle - DOWN_4) * ((UP_ANGLE_EVEN - DOWN_ANGLE_EVEN)/(UP_4 - DOWN_4)) + DOWN_ANGLE_EVEN;
    // g_leg_right.phi1_dot = g_motor_rf->stats->velocity;
    g_leg_right.phi4 = (g_motor_rb->stats->angle - DOWN_3) * ((UP_ANGLE_ODD - DOWN_ANGLE_ODD)/(UP_3 - DOWN_3)) + DOWN_ANGLE_ODD;
    // g_leg_right.phi4_dot = g_motor_rb->stats->velocity;
}

void _leg_length_controller(float chassis_height)
{
    g_leg_left.force = PID(&g_pid_left_leg_length, chassis_height - g_leg_left.length); // + feedforward weight
    g_leg_right.force = PID(&g_pid_left_leg_length, chassis_height - g_leg_right.length); // + feedforward weight
}
void _lqr_balancce_controller()
{
    //g_leg_left.target_leg_virtual_torq = LQR11 * g_leg_left.current_disp + LQR12 * g_leg_left.current_vel + LQR13 * g_leg_left.current_theta + LQR14 * g_leg_left.current_theta_dot;
}
void _vmc_torq_calc()
{
    Leg_VMC(&g_leg_left);
    Leg_VMC(&g_leg_right);
    //Chassis_Hip_Motor_Torq_Ctrl(g_leg_left.torq1, g_leg_right.torq4, g_leg_left.torq1, g_leg_right.torq4);
}