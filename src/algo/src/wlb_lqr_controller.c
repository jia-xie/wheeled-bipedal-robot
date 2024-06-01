#include "wlb_lqr_controller.h"

// #define LQR11 (-5.0925f)
// #define LQR12 (-0.3268f)
// #define LQR13 (-0.0117f)
// #define LQR14 (-0.0839f)
// #define LQR15 (4.0051f)
// #define LQR16 (0.6872f)
// #define LQR21 (16.0664f)//(26.0664f)
// #define LQR22 (2.1179f)
// #define LQR23 (4.4718f)
// #define LQR24 (8.7205f)
// #define LQR25 (-3.3557f)
// #define LQR26 (-1.3210f)


//   -18.6616   -1.9907   -6.6883  -10.1705   21.8747    2.8896
//    20.5496    2.3890   11.8771   17.0779   37.3818    1.6535

// #define LQR11 (-18.6616f)
// #define LQR12 (-1.9907f)
// #define LQR13 (-6.6883f)
// #define LQR14 (-10.1705f)
// #define LQR15 (21.8747f)
// #define LQR16 (2.8896f)
// #define LQR21 (20.5496f)
// #define LQR22 (2.3890f)
// #define LQR23 (11.8771f)
// #define LQR24 (17.0779f)
// #define LQR25 (37.3818f)
// #define LQR26 (1.6535f)

//    -8.4909   -0.6257   -0.6143   -1.2605    4.5057    1.4940
//    20.1501    1.6378    3.3924    6.6094    8.2635    4.0474

// #define LQR11 (-8.4909f)
// #define LQR12 (-0.6257f)
// #define LQR13 (-0.6143f)
// #define LQR14 (-1.2605f)
// #define LQR15 (4.5057f)
// #define LQR16 (1.4940f)
// #define LQR21 (20.1501f)
// #define LQR22 (1.6378f)
// #define LQR23 (3.3924f)
// #define LQR24 (6.6094f)
// #define LQR25 (8.2635f)
// #define LQR26 (4.0474f)
//   -11.5014   -1.0004   -2.0702   -3.4957    5.6606    1.8665
//     5.9845    0.4865    1.6902    2.7065   15.8996    6.6167

// #define LQR11 (-11.5014f)
// #define LQR12 (-1.0004f)
// #define LQR13 (-2.0702f)
// #define LQR14 (-3.4957f)
// #define LQR15 (5.6606f)
// #define LQR16 (1.8665f)
// #define LQR21 (5.9845f)
// #define LQR22 (0.4865f)
// #define LQR23 (1.6902f)
// #define LQR24 (2.7065f)
// #define LQR25 (15.8996f)
// #define LQR26 (6.6167f)

// -11.4959   -1.0314   -4.5662   -4.2584   12.8567    1.7772
// 9.1508    0.8890    6.4497    5.5134   29.3017    1.9084

#define LQR11 (-11.4959f)
#define LQR12 (-1.0314f)
#define LQR13 (-4.5662f)
#define LQR14 (-4.2584f)
#define LQR15 (12.8567f)
#define LQR16 (1.7772f)
#define LQR21 (9.1508f)
#define LQR22 (0.8890f)
#define LQR23 (6.4497f)
#define LQR24 (5.5134f)
#define LQR25 (29.3017f)
#define LQR26 (1.9084f)

// hello copilot, this is a test


void LQR_Output(lqr_u_t *u, lqr_ss_t *state)
{
    u->T_A = -(LQR13 * state->x + LQR14 * state->x_dot + LQR11 * state->theta + LQR12 * state->theta_dot + LQR15 * state->phi + LQR16 * state->phi_dot);
    u->T_B = -(LQR23 * state->x + LQR24 * state->x_dot + LQR21 * state->theta + LQR22 * state->theta_dot + LQR25 * state->phi + LQR26 * state->phi_dot);
}

