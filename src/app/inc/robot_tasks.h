#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "robot.h"
#include "launch_task.h"
#include "motor_task.h"
#include "debug_task.h"
#include "jetson_orin.h"
#include "bsp_serial.h"
#include "bsp_daemon.h"
#include "ui_task.h"
#include "board_comm_task.h"

extern void IMU_Task(void const *pvParameters);

osThreadId imu_task_handle;
osThreadId robot_control_task_handle;
osThreadId motor_task_handle;
osThreadId ui_task_handle;
osThreadId debug_task_handle;
osThreadId jetson_orin_task_handle;
osThreadId daemon_task_handle;
osThreadId board_comm_task_handle;

void Robot_Tasks_Robot_Control(void const *argument);
void Robot_Tasks_Motor(void const *argument);
void Robot_Tasks_IMU(void const *argument);
void Robot_Tasks_UI(void const *argument);
void Robot_Tasks_Debug(void const *argument);
void Robot_Tasks_Jetson_Orin(void const *argument);
void Robot_Tasks_Daemon(void const *argument);
void Robot_Tasks_Board_Comm(void const *argument);

void Robot_Tasks_Start()
{
    osThreadDef(imu_task, Robot_Tasks_IMU, osPriorityAboveNormal, 0, 256);
    imu_task_handle = osThreadCreate(osThread(imu_task), NULL);

    osThreadDef(motor_task, Robot_Tasks_Motor, osPriorityNormal, 0, 256);
    motor_task_handle = osThreadCreate(osThread(motor_task), NULL);

    osThreadDef(robot_control_task, Robot_Tasks_Robot_Control, osPriorityHigh, 0, 2048);
    robot_control_task_handle = osThreadCreate(osThread(robot_control_task), NULL);

    osThreadDef(ui_task, Robot_Tasks_UI, osPriorityAboveNormal, 0, 256);
    ui_task_handle = osThreadCreate(osThread(ui_task), NULL);

    osThreadDef(debug_task, Robot_Tasks_Debug, osPriorityNormal, 0, 256);
    debug_task_handle = osThreadCreate(osThread(debug_task), NULL);

    osThreadDef(jetson_orin_task, Robot_Tasks_Jetson_Orin, osPriorityAboveNormal, 0, 256);
    jetson_orin_task_handle = osThreadCreate(osThread(jetson_orin_task), NULL);

    osThreadDef(daemon_task, Robot_Tasks_Daemon, osPriorityAboveNormal, 0, 256);
    daemon_task_handle = osThreadCreate(osThread(daemon_task), NULL);

    osThreadDef(board_comm_task, Robot_Tasks_Board_Comm, osPriorityAboveNormal, 0, 256);
    board_comm_task_handle = osThreadCreate(osThread(board_comm_task), NULL);
}

void Robot_Tasks_Robot_Control(void const *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2);
    while (1)
    {
        Robot_Ctrl_Loop();
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

__weak void Robot_Tasks_IMU(void const *argument)
{
    IMU_Task(argument);
}

void Robot_Tasks_Motor(void const *argument)
{
    // portTickType xLastWakeTime;
    // xLastWakeTime = xTaskGetTickCount();
    //const TickType_t TimeIncrement = pdMS_TO_TICKS(1);
    while (1)
    {
        Motor_Task_Loop();
        //vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

void Robot_Tasks_UI(void const *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(100);
    while (1)
    {
        UI_Task_Loop();
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

void Robot_Tasks_Debug(void const *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(DEBUG_PERIOD);
    while (1)
    {
        Debug_Task_Loop();
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

void Robot_Tasks_Jetson_Orin(void const *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(JETSON_ORIN_PERIOD);
    while (1)
    {
        Jetson_Orin_Send_Data();
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

void Robot_Tasks_Daemon(void const *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(DAEMON_PERIOD);
    while (1)
    {
        Daemon_Task_Loop();
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

void Robot_Tasks_Board_Comm(void const *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(1);
    while (1)
    {
        Board_Comm_Task_Loop();
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
