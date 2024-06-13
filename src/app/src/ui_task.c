#include "ui_task.h"
#include "dji_motor.h"
#include <math.h>

extern DJI_Motor_Handle_t *g_yaw; // Used for chassis orientation

void UI_Task_Loop(void)
{
   ui_self_id = Referee_Robot_State.ID;
        if (!g_robot_state.UI_enabled)
        {
            ui_remove_indicator_0();
            ui_init_indicator_0();
            ui_remove_indicator_1();
            ui_init_indicator_1();
            g_robot_state.UI_enabled = 1;
        }
        if (g_launch_target.flywheel_enabled)
        {
            ui_indicator_1_Flywheel_Select->start_x = 270;
            ui_indicator_1_Flywheel_Select->end_x = 320;
        }
        else
        {
            ui_indicator_1_Flywheel_Select->start_x = 335;
            ui_indicator_1_Flywheel_Select->end_x = 385;
        }
        if (g_robot_state.spintop_mode)
        {
            ui_indicator_1_Spintop_Select->start_x = 270;
            ui_indicator_1_Spintop_Select->end_x = 320;
        }
        else
        {
            ui_indicator_1_Spintop_Select->start_x = 335;
            ui_indicator_1_Spintop_Select->end_x = 385;
        }
        if (g_robot_state.autoaiming_enabled)
        {
            ui_indicator_1_Autoaim_Select->start_x = 270;
            ui_indicator_1_Autoaim_Select->end_x = 320;
            ui_indicator_1_Aim_H_Line->color = 2;
            ui_indicator_1_Aim_V_Line->color = 2;
        }
        else
        {
            ui_indicator_1_Autoaim_Select->start_x = 335;
            ui_indicator_1_Autoaim_Select->end_x = 385;
            ui_indicator_1_Aim_H_Line->color = 3;
            ui_indicator_1_Aim_V_Line->color = 3;
        }
        if (ui_indicator_1_Supercap->number>=99)
        {
            ui_indicator_1_Supercap->number = 0;
        }
        ui_indicator_1_Supercap->number++;
        
        // Angle Difference
        float chassis_orientation = -DJI_Motor_Get_Absolute_Angle(g_yaw);
		ui_indicator_1_Orientation_Line->start_x = 1700.0f + sinf(chassis_orientation)*140.0f;
		ui_indicator_1_Orientation_Line->start_y = 480.0f - cosf(chassis_orientation)*140.0f;
		ui_indicator_1_Orientation_Line->end_x = 1700.0f - sinf(chassis_orientation)*140.0f;
		ui_indicator_1_Orientation_Line->end_y = 480.0f + cosf(chassis_orientation)*140.0f;
        ui_update_indicator_1();
}