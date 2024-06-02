#include "debug_task.h"
#include "bsp_serial.h"
#include "launch_task.h"
#include "remote.h"
#include "gimbal_task.h"
#include "Swerve_Locomotion.h"
#include "user_math.h"
#include "imu_task.h"
#include "robot.h"
#include "referee_system.h"
#include "jetson_orin.h"
#include "bsp_daemon.h"
#include "wlb_lqr_controller.h"

extern lqr_ss_t g_lqr_left_state, g_lqr_right_state;
extern Robot_State_t g_robot_state;
extern DJI_Motor_Handle_t *g_yaw;
extern IMU_t g_imu;
extern Swerve_Module_t g_swerve_fl;
extern Remote_t g_remote; 
extern Launch_Target_t g_launch_target;
extern uint64_t t;
extern Daemon_Instance_t *g_daemon_instances[3];
extern Daemon_Instance_t *g_remote_daemon;
#define PRINT_RUNTIME_STATS
#ifdef PRINT_RUNTIME_STATS
char g_debug_buffer[1024*2] = {0};
#endif
extern PID_t g_balance_angle_pid, g_balance_vel_pid;
extern PID_t g_pid_anti_split;
const char* top_border = "\r\n\r\n\r\n/***** System Info *****/\r\n";
const char* bottom_border = "/***** End of Info *****/\r\n";
extern lqr_u_t g_u_left, g_u_right;
#define DEBUG_ENABLED

void Debug_Task_Loop(void)
{
#ifdef DEBUG_ENABLED
    // static uint32_t counter = 0;
    // #ifdef PRINT_RUNTIME_STATS
    // if (counter % 100 == 0) // Print every 100 cycles
    // {
    //     vTaskGetRunTimeStats(g_debug_buffer);
    //     DEBUG_PRINTF(&huart6, "%s", top_border);
    //     DEBUG_PRINTF(&huart6, "%s", g_debug_buffer);
    //     DEBUG_PRINTF(&huart6, "%s", bottom_border);
    // }
    // #endif
    
    // DEBUG_PRINTF(&huart6, ">time:%.1f,yaw:%f,pitch:%f,roll:%f\n", (float) counter / 1000.0f * DEBUG_PERIOD, 
    //             g_imu.deg.yaw, g_imu.deg.pitch, g_imu.deg.roll);
    // DEBUG_PRINTF(&huart6, ">remote_daemon:%d\n", g_remote_daemon->counter);
    // counter++;
    // if (counter > 0xFFFFFFFF) {
    //     counter = 0;
    // }
    //DEBUG_PRINTF(&huart6, "/*%f,%f*/", g_swerve_fl.azimuth_motor->angle_pid->ref,g_swerve_fl.azimuth_motor->stats->absolute_angle_rad);
    // DEBUG_PRINTF(&huart6, "/*%f,%f*/", g_robot_state.chassis_total_power, Referee_Robot_State.Chassis_Power);
    DEBUG_PRINTF(&huart6, ">TA_l:%f\n>TB_l:%f\n", g_u_left.T_A, g_u_left.T_B);
    DEBUG_PRINTF(&huart6, ">TA_r:%f\n>TB_r:%f\n", g_u_right.T_A, g_u_right.T_B);
    
    DEBUG_PRINTF(&huart6, ">x_l:%f\n>x_dot_l:%f\n>theta_l:%f\n>theta_dot_l:%f\n>phi_l:%f\n>phi_dot_l:%f\n", g_lqr_left_state.x, g_lqr_left_state.x_dot, g_lqr_left_state.theta, g_lqr_left_state.theta_dot, g_lqr_left_state.phi, g_lqr_left_state.phi_dot);
    DEBUG_PRINTF(&huart6, ">x_r:%f\n>x_dot_r:%f\n>theta_r:%f\n>theta_dot_r:%f\n>phi_r:%f\n>phi_dot_r:%f\n", g_lqr_right_state.x, g_lqr_right_state.x_dot, g_lqr_right_state.theta, g_lqr_right_state.theta_dot, g_lqr_right_state.phi, g_lqr_right_state.phi_dot);
    DEBUG_PRINTF(&huart6, ">anti:%f\n", g_pid_anti_split.output);
    // DEBUG_PRINTF(&huart6, ">pitch:%f\n>pid_vel:%f\n>pid_ang:%f\n", g_lqr_right_state.phi, g_balance_vel_pid.output, g_balance_angle_pid.output);
#endif
}