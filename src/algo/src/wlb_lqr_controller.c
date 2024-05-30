#include "wlb_lqr_controller.h"

void LQR_Output(lqr_u_t *u, lqr_ss_t *state)
{
    u->T_A = LQR11 * state->x + LQR12 * state->x_dot + LQR13 * state->theta + LQR14 * state->theta_dot + LQR15 * state->phi + LQR16 * state->phi_dot;
    u->T_B = LQR21 * state->x + LQR22 * state->x_dot + LQR23 * state->theta + LQR24 * state->theta_dot + LQR25 * state->phi + LQR26 * state->phi_dot;
}