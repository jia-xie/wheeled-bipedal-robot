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
//    Q=diag([0.1 0.1 100 50 5000 1]);
//    R=diag([150 5]);  
// float lqr_poly_fit_param[12][4] = 
// {{-146.8397,178.1973,-83.7881,2.8443},
// {8.5304,-3.7974,-3.5802,0.2173},
// {-40.5217,39.2950,-13.5266,0.9230},
// {-57.9757,56.6803,-19.9446,1.3387},
// {195.0357,-122.9509,9.7593,6.2244},
// {28.3143,-20.0813,3.1549,0.5661},
// {1473.2038,-1142.9067,247.5109,5.8574},
// {105.5269,-94.7226,27.2942,-0.5018},
// {135.5466,-83.0411,4.9135,4.6654},
// {206.4120,-129.2789,9.6747,6.7037},
// {1603.1755,-1567.6944,545.5435,-38.7507},
// {130.3261,-137.4586,52.2075,-5.0431}};

// Q=diag([0.1 0.1 100 50 5000 1]);
// R=[150 0;0 5];
float lqr_poly_fit_param[12][4] = 
{{-135.3453,158.9821,-75.0086,2.0930},
{4.8803,-1.4456,-3.8973,0.1909},
{-37.5561,35.2304,-11.7066,0.6412},
{-54.4328,51.3769,-17.4431,0.9291},
{138.9470,-77.9373,-2.4892,7.9823},
{21.5725,-14.9521,2.0154,0.6933},
{1117.0967,-875.1741,193.5456,7.1032},
{85.3890,-76.2583,21.8655,-0.1857},
{65.3828,-28.1241,-8.2274,5.3604},
{103.0676,-47.8553,-10.0767,7.8005},
{1622.2835,-1555.2473,533.1584,-34.9396},
{129.6584,-132.4556,49.3328,-4.6592}};
void LQR_Output(lqr_u_t *u, lqr_ss_t *state)
{
float len = state->leg_len;
float len_sqrt = len * len;
float len_cub = len * len * len;

    u->T_A = (lqr_poly_fit_param[0 ][0] * len_cub + lqr_poly_fit_param[0 ][1] * len_sqrt + lqr_poly_fit_param[0 ][2] * len + lqr_poly_fit_param[0 ][3]) * -state->theta +
             (lqr_poly_fit_param[1 ][0] * len_cub + lqr_poly_fit_param[1 ][1] * len_sqrt + lqr_poly_fit_param[1 ][2] * len + lqr_poly_fit_param[1 ][3]) * -state->theta_dot +
             0*(lqr_poly_fit_param[2 ][0] * len_cub + lqr_poly_fit_param[2 ][1] * len_sqrt + lqr_poly_fit_param[2 ][2] * len + lqr_poly_fit_param[2 ][3]) * (state->target_x-state->x) +
             (lqr_poly_fit_param[3 ][0] * len_cub + lqr_poly_fit_param[3 ][1] * len_sqrt + lqr_poly_fit_param[3 ][2] * len + lqr_poly_fit_param[3 ][3]) * (state->target_x_dot - state->x_dot) +
             (lqr_poly_fit_param[4 ][0] * len_cub + lqr_poly_fit_param[4 ][1] * len_sqrt + lqr_poly_fit_param[4 ][2] * len + lqr_poly_fit_param[4 ][3]) * -state->phi +
             (lqr_poly_fit_param[5 ][0] * len_cub + lqr_poly_fit_param[5 ][1] * len_sqrt + lqr_poly_fit_param[5 ][2] * len + lqr_poly_fit_param[5 ][3]) * -state->phi_dot;
    u->T_B = (lqr_poly_fit_param[6 ][0] * len_cub + lqr_poly_fit_param[6 ][1] * len_sqrt + lqr_poly_fit_param[6 ][2] * len + lqr_poly_fit_param[6 ][3]) * -state->theta +
             (lqr_poly_fit_param[7 ][0] * len_cub + lqr_poly_fit_param[7 ][1] * len_sqrt + lqr_poly_fit_param[7 ][2] * len + lqr_poly_fit_param[7 ][3]) * -state->theta_dot +
             0*(lqr_poly_fit_param[8 ][0] * len_cub + lqr_poly_fit_param[8 ][1] * len_sqrt + lqr_poly_fit_param[8 ][2] * len + lqr_poly_fit_param[8 ][3]) * (state->target_x-state->x) +
             (lqr_poly_fit_param[9 ][0] * len_cub + lqr_poly_fit_param[9 ][1] * len_sqrt + lqr_poly_fit_param[9 ][2] * len + lqr_poly_fit_param[9 ][3]) * (state->target_x_dot-state->x_dot) +
             (lqr_poly_fit_param[10][0] * len_cub + lqr_poly_fit_param[10][1] * len_sqrt + lqr_poly_fit_param[10][2] * len + lqr_poly_fit_param[10][3]) * -state->phi +
             (lqr_poly_fit_param[11][0] * len_cub + lqr_poly_fit_param[11][1] * len_sqrt + lqr_poly_fit_param[11][2] * len + lqr_poly_fit_param[11][3]) * -state->phi_dot;
}
