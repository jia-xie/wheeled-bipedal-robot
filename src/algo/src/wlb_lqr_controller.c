#include "wlb_lqr_controller.h"
#include <stdint.h>

#define LQR11 (-9.0381f)
#define LQR12 (-0.8002f)
#define LQR13 (-1.5454f)
#define LQR14 (-1.8952f)
#define LQR15 (7.0606f)
#define LQR16 (0.8181f)
#define LQR21 (29.8781f)
#define LQR22 (3.7447f)
#define LQR23 (11.3292f)
#define LQR24 (12.9867f)
#define LQR25 (25.9207f)
#define LQR26 (0.8019f)

#define OPENSOURCE_COORDINATE
float lqr_poly_fit_param[12][4] = 
{
{-162.7817,184.7027,-92.9200,1.9894},
{9.3663,-5.9800,-5.6335,0.2701},
{-104.5829,96.8277,-31.8193,1.3915},
{-109.5617,102.7483,-35.3979,1.4859},
{62.6867,-9.9544,-23.5856,10.6094},
{13.1709,-6.8554,-0.8303,1.1092},
{848.2953,-670.0691,151.4642,8.5111},
{82.7918,-76.2390,24.3390,-0.0572},
{59.2303,0.7792,-31.0979,12.3883},
{84.7422,-18.7684,-27.3311,13.2261},
{1337.1600,-1243.9041,411.9104,-19.7058},
{130.9799,-125.3947,43.3152,-3.2934}
};
void LQR_Output(lqr_u_t *u, lqr_ss_t *state)
{
float len = state->leg_len;
float len_sqrt = len * len;
float len_cub = len * len * len;

    u->T_A = (lqr_poly_fit_param[0 ][0] * len_cub + lqr_poly_fit_param[0 ][1] * len_sqrt + lqr_poly_fit_param[0 ][2] * len + lqr_poly_fit_param[0 ][3]) * -state->theta +
             (lqr_poly_fit_param[1 ][0] * len_cub + lqr_poly_fit_param[1 ][1] * len_sqrt + lqr_poly_fit_param[1 ][2] * len + lqr_poly_fit_param[1 ][3]) * -state->theta_dot +
             (lqr_poly_fit_param[2 ][0] * len_cub + lqr_poly_fit_param[2 ][1] * len_sqrt + lqr_poly_fit_param[2 ][2] * len + lqr_poly_fit_param[2 ][3]) * (state->target_x-state->x) +
             (lqr_poly_fit_param[3 ][0] * len_cub + lqr_poly_fit_param[3 ][1] * len_sqrt + lqr_poly_fit_param[3 ][2] * len + lqr_poly_fit_param[3 ][3]) * (state->target_x_dot - state->x_dot) +
             (lqr_poly_fit_param[4 ][0] * len_cub + lqr_poly_fit_param[4 ][1] * len_sqrt + lqr_poly_fit_param[4 ][2] * len + lqr_poly_fit_param[4 ][3]) * -state->phi +
             (lqr_poly_fit_param[5 ][0] * len_cub + lqr_poly_fit_param[5 ][1] * len_sqrt + lqr_poly_fit_param[5 ][2] * len + lqr_poly_fit_param[5 ][3]) * -state->phi_dot;
    u->T_B = (lqr_poly_fit_param[6 ][0] * len_cub + lqr_poly_fit_param[6 ][1] * len_sqrt + lqr_poly_fit_param[6 ][2] * len + lqr_poly_fit_param[6 ][3]) * -state->theta +
             (lqr_poly_fit_param[7 ][0] * len_cub + lqr_poly_fit_param[7 ][1] * len_sqrt + lqr_poly_fit_param[7 ][2] * len + lqr_poly_fit_param[7 ][3]) * -state->theta_dot +
             (lqr_poly_fit_param[8 ][0] * len_cub + lqr_poly_fit_param[8 ][1] * len_sqrt + lqr_poly_fit_param[8 ][2] * len + lqr_poly_fit_param[8 ][3]) * (state->target_x-state->x) +
             (lqr_poly_fit_param[9 ][0] * len_cub + lqr_poly_fit_param[9 ][1] * len_sqrt + lqr_poly_fit_param[9 ][2] * len + lqr_poly_fit_param[9 ][3]) * (state->target_x_dot-state->x_dot) +
             (lqr_poly_fit_param[10][0] * len_cub + lqr_poly_fit_param[10][1] * len_sqrt + lqr_poly_fit_param[10][2] * len + lqr_poly_fit_param[10][3]) * -state->phi +
             (lqr_poly_fit_param[11][0] * len_cub + lqr_poly_fit_param[11][1] * len_sqrt + lqr_poly_fit_param[11][2] * len + lqr_poly_fit_param[11][3]) * -state->phi_dot;
}
