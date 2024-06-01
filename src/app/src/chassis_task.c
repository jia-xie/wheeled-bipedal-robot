#include "chassis_task.h"

#include "robot.h"
#include "mf_motor.h"
#include "remote.h"
#include "imu_task.h"
#include "bsp_can.h"
#include "five_bar_leg.h"
#include "wlb_lqr_controller.h"
#include "robot_param.h"

#define TASK_TIME (0.002f)
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

#define FOOT_MOTOR_MAX_TORQ (3.0f)
#define FOOT_MF9025_MAX_TORQ_INT ((FOOT_MOTOR_MAX_TORQ / MF9025_TORQ_CONSTANT) / 16.5f * 2048.0f)

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;
extern IMU_t g_imu;
uint8_t g_left_foot_initialized = 0;
uint8_t g_right_foot_initialized = 0;
MF_Motor_Handle_t *g_motor_lf, *g_motor_rf, *g_motor_lb, *g_motor_rb;
MF_Motor_Handle_t *g_left_foot_motor, *g_right_foot_motor;
Leg_t g_leg_left, g_leg_right;
lqr_ss_t g_lqr_left_state, g_lqr_right_state = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
lqr_u_t g_u_left, g_u_right;
PID_t g_pid_left_leg_length, g_pid_left_leg_angle;
PID_t g_pid_right_leg_length, g_pid_right_leg_angle;
PID_t g_balance_angle_pid, g_balance_vel_pid;
Leg_t test = {
        .phi1 = PI,
        .phi4 = 0,
        .phi1_dot = 0,
        .phi4_dot = 0,
    };
void _get_leg_statistics();
void _wheel_leg_estimation(float robot_pitch, float robot_pitch_dot);
void _leg_length_controller(float chassis_height);
void _lqr_balancce_controller();
void _vmc_torq_calc();

void Chassis_Task_Init()
{
    // Initialize motors
    MF_Motor_Config_t motor_config = {
        .can_bus = 1,
        .tx_id = 0x141,
        .rx_id = 0x141,
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

    /* Foot Motor Init */
    motor_config.tx_id = 0x147;
    motor_config.rx_id = 0x147;
    g_left_foot_motor = MF_Motor_Init(motor_config);
    motor_config.tx_id = 0x148;
    motor_config.rx_id = 0x148;
    g_right_foot_motor = MF_Motor_Init(motor_config);

    MF_Motor_Broadcast_Init(1);
    PID_Init(&g_pid_left_leg_length, 1000.0f, 0.0f, 10.0f, 50.0f, 0.0f, 0.0f);
    PID_Init(&g_pid_right_leg_length, 1000.0f, 0.0f, 10.0f, 50.0f, 0.0f, 0.0f);

    PID_Init(&g_pid_left_leg_angle, 15.0f, 0.0f, 5.75f, 10.0f, 0.0f, 0.0f);
    PID_Init(&g_pid_right_leg_angle, 15.0f, 0.0f, 5.75f, 10.0f, 0.0f, 0.0f);

    PID_Init(&g_balance_angle_pid, 600.0f, 0.0f, .065f, 10.0f, 0.0f, 0.0f);
    PID_Init(&g_balance_vel_pid, 10.0f, 0.0f, 0.001f, 10.0f, 0.0f, 0.0f);

}

void _hip_motor_torq_ctrl(float torq_lf, float torq_lb, float torq_rb, float torq_rf)
{
    int16_t torq1 = torq_lf / MG8016_TORQ_CONSTANT * (2048.0f / 16.5f);
    int16_t torq2 = torq_lb / MG8016_TORQ_CONSTANT * (2048.0f / 16.5f);
    int16_t torq3 = torq_rb / MG8016_TORQ_CONSTANT * (2048.0f / 16.5f);
    int16_t torq4 = torq_rf / MG8016_TORQ_CONSTANT * (2048.0f / 16.5f);
    __MAX_LIMIT(torq1, -2000, 2000);
    __MAX_LIMIT(torq2, -2000, 2000);
    __MAX_LIMIT(torq3, -2000, 2000);
    __MAX_LIMIT(torq4, -2000, 2000);
    MF_Motor_Broadcast_Torq_Ctrl(1, torq1, torq2, torq3, torq4);
}

void _foot_motor_torq_ctrl(float torq_left, float torq_right)
{
    int16_t torq_left_int = torq_left / MF9025_TORQ_CONSTANT * (2048.0f / 16.5f);
    int16_t torq_right_int = torq_right / MF9025_TORQ_CONSTANT * (2048.0f / 16.5f);
    __MAX_LIMIT(torq_left, -FOOT_MF9025_MAX_TORQ_INT, FOOT_MF9025_MAX_TORQ_INT);
    __MAX_LIMIT(torq_right, -FOOT_MF9025_MAX_TORQ_INT, FOOT_MF9025_MAX_TORQ_INT);
    MF_Motor_TorqueCtrl(g_left_foot_motor, torq_left_int);
    MF_Motor_TorqueCtrl(g_right_foot_motor, torq_right_int);
}

void _foot_motor_init_offset()
{
     // load the offset value when the robot start, this is for state space x to be 0 at startup
    if (!g_left_foot_initialized)
    {
        if (g_left_foot_motor->stats->angle != 0)
        {
            g_left_foot_motor->angle_offset = -g_left_foot_motor->stats->total_angle;
            g_left_foot_initialized = 1;
        }
    }
    if (!g_right_foot_initialized)
    {
        if (g_right_foot_motor->stats->angle != 0)
        {
            g_right_foot_motor->angle_offset = -g_right_foot_motor->stats->total_angle;
            g_right_foot_initialized = 1;
        }
    }
}

/**
 * @brief Wheel leg estimation
 * 
 * @note robot pitch is in the positive direction, adjust sign and offset before passing into this function
*/
void _wheel_leg_estimation(float robot_pitch, float robot_pitch_dot)
{
    _foot_motor_init_offset();
    _get_leg_statistics();
    Leg_ForwardKinematics(&g_leg_left, g_leg_left.phi1, g_leg_left.phi4, g_leg_left.phi1_dot, g_leg_left.phi4_dot);
    g_lqr_left_state.x          = -(g_left_foot_motor->stats->total_angle + g_left_foot_motor->angle_offset) * FOOT_WHEEL_RADIUS;
    g_lqr_left_state.x_dot      = -g_left_foot_motor->stats->velocity * DEG_TO_RAD * FOOT_WHEEL_RADIUS;
    g_lqr_left_state.theta      = -(g_leg_left.phi0 - PI/2 + robot_pitch);
    g_lqr_left_state.theta_dot  = -(g_leg_left.phi0_dot + robot_pitch_dot);
    g_lqr_left_state.phi        = robot_pitch;
    g_lqr_left_state.phi_dot    = robot_pitch_dot;
    Leg_ForwardKinematics(&g_leg_right, g_leg_right.phi1, g_leg_right.phi4, g_leg_right.phi1_dot, g_leg_right.phi4_dot);
    g_lqr_right_state.x          = (g_right_foot_motor->stats->total_angle + g_right_foot_motor->angle_offset) * FOOT_WHEEL_RADIUS;
    g_lqr_right_state.x_dot      = (g_right_foot_motor->stats->velocity * DEG_TO_RAD *  FOOT_WHEEL_RADIUS);
    g_lqr_right_state.theta      = -(-g_leg_right.phi0 + PI/2 + robot_pitch);
    g_lqr_right_state.theta_dot  = -(-g_leg_right.phi0_dot + robot_pitch_dot);
    g_lqr_right_state.phi        = (robot_pitch);
    g_lqr_right_state.phi_dot    = (robot_pitch_dot);
}

void _get_leg_statistics()
{
    g_leg_left.phi1 = ((g_motor_lb->stats->angle - DOWN_2) * ((UP_ANGLE_EVEN - DOWN_ANGLE_EVEN)/(UP_2 - DOWN_2)) + DOWN_ANGLE_EVEN) * DEG_TO_RAD;
    // g_leg_left.phi1_dot = g_motor_lb->stats->velocity;
    g_leg_left.phi4 = ((g_motor_lf->stats->angle - DOWN_1) * ((UP_ANGLE_ODD - DOWN_ANGLE_ODD)/(UP_1 - DOWN_1)) + DOWN_ANGLE_ODD) * DEG_TO_RAD;
    // g_leg_left.phi4_dot = g_motor_lf->stats->velocity;

    g_leg_right.phi1 = ((g_motor_rf->stats->angle - DOWN_4) * ((UP_ANGLE_EVEN - DOWN_ANGLE_EVEN)/(UP_4 - DOWN_4)) + DOWN_ANGLE_EVEN) * DEG_TO_RAD;
    // g_leg_right.phi1_dot = g_motor_rf->stats->velocity;
    g_leg_right.phi4 = ((g_motor_rb->stats->angle - DOWN_3) * ((UP_ANGLE_ODD - DOWN_ANGLE_ODD)/(UP_3 - DOWN_3)) + DOWN_ANGLE_ODD) * DEG_TO_RAD;
    // g_leg_right.phi4_dot = g_motor_rb->stats->velocity;
}

void _leg_length_controller(float chassis_height)
{
    // float feedforward_weight = 40.0f;
    g_leg_left.target_leg_virtual_force = 0;//PID_dt(&g_pid_left_leg_length, chassis_height - g_leg_left.length, TASK_TIME);//+ feedforward_weight;//0+g_remote.controller.right_stick.y/660.0f*10.0f;//PID(&g_pid_left_leg_length, chassis_height - g_leg_left.length);// + feedforward_weight;
    g_leg_right.target_leg_virtual_force = 0;//PID_dt(&g_pid_right_leg_length, chassis_height - g_leg_right.length, TASK_TIME);//+ feedforward_weight;//PID(&g_pid_left_leg_length, chassis_height - g_leg_right.length);// + feedforward_weight;
    g_leg_left.target_leg_virtual_torq =  g_u_left.T_B;//PID_dt(&g_pid_left_leg_angle, PI/2 + g_remote.controller.right_stick.y/660.0f - g_leg_left.phi0, TASK_TIME); //g_u_left.T_B;// + PID(&g_pid_left_leg_angle, PI/2 + g_remote.controller.right_stick.y/660.0f - g_leg_left.phi0);
    g_leg_right.target_leg_virtual_torq = -g_u_right.T_B;//PID_dt(&g_pid_right_leg_angle, PI/2 + g_remote.controller.right_stick.y/660.0f - g_leg_right.phi0, TASK_TIME);//g_u_right.T_B;// + PID(&g_pid_right_leg_angle, PI/2 + g_remote.controller.right_stick.y/660.0f - g_leg_right.phi0);
}
void _lqr_balancce_controller()
{
    // float target_x_dot = 0;
    // float robot_x_dot = (g_lqr_left_state.x_dot + g_lqr_right_state.x_dot) / 2.0f;
    // float vel_pid_output = PID_dt(&g_balance_vel_pid, robot_x_dot - target_x_dot, TASK_TIME);
    // float robot_pitch = (-g_imu.rad.roll);
    // float offset = 3.8f * DEG_TO_RAD;

    // float wheel_output = -PID_dt(&g_balance_angle_pid, offset - robot_pitch, TASK_TIME);
    // g_u_left.T_A = wheel_output + vel_pid_output;
    // g_u_right.T_A = wheel_output + vel_pid_output;

    /* LQR */
    LQR_Output(&g_u_left, &g_lqr_left_state);
    LQR_Output(&g_u_right, &g_lqr_right_state);
}
void _vmc_torq_calc()
{
    Leg_VMC(&g_leg_left);
    Leg_VMC(&g_leg_right);
    //_hip_motor_torq_ctrl(g_leg_left.torq1, g_leg_right.torq4, g_leg_left.torq1, g_leg_right.torq4);
}

void Chassis_Disable()
{
    _hip_motor_torq_ctrl(0.0f, 0.0f, 0.0f, 0.0f);
    _foot_motor_torq_ctrl(0.0f, 0.0f);
}

void Chassis_Ctrl_Loop()
{
    Leg_ForwardKinematics(&test, test.phi1, test.phi4, test.phi1_dot, test.phi4_dot);
    test.target_leg_virtual_torq = g_remote.controller.right_stick.x/660.0f * 10.0f;
    test.target_leg_virtual_force = g_remote.controller.left_stick.x/660.0f * 10.0f;
    Leg_VMC(&test);
    _wheel_leg_estimation(-g_imu.rad.roll, -g_imu.bmi088_raw.gyro[0]);
    g_robot_state.chassis_height = 0.1f;
    _leg_length_controller(g_robot_state.chassis_height);
    _lqr_balancce_controller();
    _vmc_torq_calc();
    if (g_robot_state.enabled) {
        _hip_motor_torq_ctrl(-g_leg_left.torq4, -g_leg_left.torq1, -g_leg_right.torq4, -g_leg_right.torq1);
        _foot_motor_torq_ctrl(-g_u_left.T_A,  g_u_right.T_A);
    }
    else {
        Chassis_Disable();
        g_left_foot_initialized = 0;
        g_right_foot_initialized = 0;
        PID_Reset(&g_pid_left_leg_length);
        PID_Reset(&g_pid_right_leg_length);
        PID_Reset(&g_pid_left_leg_angle);
        PID_Reset(&g_pid_right_leg_angle);
        PID_Reset(&g_balance_angle_pid);
        PID_Reset(&g_balance_vel_pid);
    }
}