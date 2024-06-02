#include "wlb_lqr_controller.h"
#include <stdint.h>
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

// #define LQR11 (-11.4959f)
// #define LQR12 (-1.0314f)
// #define LQR13 (-4.5662f)
// #define LQR14 (-4.2584f)
// #define LQR15 (12.8567f)
// #define LQR16 (1.7772f)
// #define LQR21 (9.1508f)
// #define LQR22 (0.8890f)
// #define LQR23 (6.4497f)
// #define LQR24 (5.5134f)
// #define LQR25 (29.3017f)
// #define LQR26 (1.9084f)

// 1.2230    2.8087  -18.7756   -2.3300   -4.3443   -0.6511
// -0.1463   -0.2457    4.1124    0.0802   -8.1982   -2.2855

// #define LQR11 (-1.2230f)
// #define LQR12 (-2.8087f)
// #define LQR13 (-18.7756f)
// #define LQR14 (-2.3300f)
// #define LQR15 (4.3443f)
// #define LQR16 (0.6511f)
// #define LQR21 (0.1463f)
// #define LQR22 (0.2457f)
// #define LQR23 (4.1124f)
// #define LQR24 (0.0802f)
// #define LQR25 (8.1982f)
// #define LQR26 (2.2855f)

//     1.2232    2.8088  -18.7893   -2.3314   -4.4180   -0.6522
//    -0.1378   -0.2613    3.7433    0.0440  -11.7099   -2.3239

// #define LQR11 (-1.2232f)
// #define LQR12 (-2.8088f)
// #define LQR13 (-18.7893f)
// #define LQR14 (-2.3314f)
// #define LQR15 (4.4180f)
// #define LQR16 (0.6522f)
// #define LQR21 (0.1378f)
// #define LQR22 (0.2613f)
// #define LQR23 (3.7433f)
// #define LQR24 (0.0440f)
// #define LQR25 (11.7099f)
// #define LQR26 (2.3239f)

// Q=diag([150 220 0.1 3 500 100]);   %x x_dot theta theta_dot phi phi_dot
// R=[100 0;0 20];
//     1.2230    2.8086  -18.7829   -2.3307   -4.3855   -0.6517
//    -0.1441   -0.2640    3.9221    0.0626   -9.9888   -2.3050
// #define LQR11 (1.2230f)
// #define LQR12 (2.8086f)
// #define LQR13 (-18.7829f)
// #define LQR14 (-2.3307f)
// #define LQR15 (-4.3855f)
// #define LQR16 (-0.6517f)
// #define LQR21 (-0.1441f)
// #define LQR22 (-0.2640f)
// #define LQR23 (3.9221f)
// #define LQR24 (0.0626f)
// #define LQR25 (-9.9888f)
// #define LQR26 (-2.3050f)

// Q=diag([150 220 0.1 3 700 100]);   %x x_dot theta theta_dot phi phi_dot
// R=[100 0;0 20];
//     1.2231    2.8087  -18.7858   -2.3310   -4.4003   -0.6519
//    -0.1418   -0.2645    3.8447    0.0547  -10.7251   -2.3131
// #define LQR11 (-1.2231f)
// #define LQR12 (-2.8087f)
// #define LQR13 (-18.7858f)
// #define LQR14 (-2.3310f)
// #define LQR15 (4.4003f)
// #define LQR16 (0.6519f)
// #define LQR21 (0.1418f)
// #define LQR22 (0.2645f)
// #define LQR23 (3.8447f)
// #define LQR24 (0.0547f)
// #define LQR25 (10.7251f)
// #define LQR26 (2.3131f)

// Trial June 1 7:20pm
//  Q=diag([150 220 0.1 3 600 100]);   %x x_dot theta theta_dot phi phi_dot
//  R=[100 0;0 20];
//      1.2231    2.8086  -18.7844   -2.3309   -4.3933   -0.6518
//     -0.1430   -0.2646    3.8823    0.0586  -10.3665   -2.3091
//  #define LQR11 (-1.2231f)
//  #define LQR12 (-2.8086f)
//  #define LQR13 (-18.7844f)
//  #define LQR14 (-2.3309f)
//  #define LQR15 (4.3933f)
//  #define LQR16 (0.6518f)
//  #define LQR21 (0.1430f)
//  #define LQR22 (20.2646f)
//  #define LQR23 (3.8823f)
//  #define LQR24 (0.0586f)
//  #define LQR25 (10.3665f)
//  #define LQR26 (2.3091f)

// Trial June 1 7:38pm
// Change model
// Q=diag([150 220 0.1 3 600 100]);   %x x_dot theta theta_dot phi phi_dot
// R=[100 0;0 20];
//     0.8409    4.2613  -17.0625   -2.5968   -3.8508   -1.4408
//    -1.9911   -9.6391   16.5211    2.4317   -3.1184   -1.2297
// #define LQR11 (-0.8409f)
// #define LQR12 (-4.2613f)
// #define LQR13 (-17.0625f)
// #define LQR14 (-2.5968f)
// #define LQR15 (3.8508f)
// #define LQR16 (1.4408f)
// #define LQR21 (1.9911f)
// #define LQR22 (9.6391f)
// #define LQR23 (16.5211f)
// #define LQR24 (2.4317f)
// #define LQR25 (3.1184f)
// #define LQR26 (1.2297f)

// hello copilot, this is a test

//    -6.0343   -0.2557   -0.4117   -0.7053    3.9434    0.3431
//    14.6845    0.8073    2.5709    4.2583    9.2644    0.3138
// #define LQR11 (-6.0343f)
// #define LQR12 (-0.2557f)
// #define LQR13 (-0.4117f)
// #define LQR14 (-0.7053f)
// #define LQR15 (3.9434f)
// #define LQR16 (0.3431f)
// #define LQR21 (14.6845f)
// #define LQR22 (0.8073f)
// #define LQR23 (2.5709f)
// #define LQR24 (4.2583f)
// #define LQR25 (9.2644f)
// #define LQR26 (0.3138f)

//-8.0758   -0.4879   -0.3471   -0.6163    3.8499    0.6250
// 22.3963    1.7842    2.7552    4.6207    8.2952    0.5604

// #define LQR11 (-8.0758f)
// #define LQR12 (-0.4879f)
// #define LQR13 (-0.3471f)
// #define LQR14 (-0.6163f)
// #define LQR15 (3.8499f)
// #define LQR16 (0.6250f)
// #define LQR21 (22.3963f)
// #define LQR22 (1.7842f)
// #define LQR23 (2.7552f)
// #define LQR24 (4.6207f)
// #define LQR25 (8.2952f)
// #define LQR26 (0.5604f)

// -13.8467   -1.3234   -3.0537   -3.5599    5.3132    0.8911
//     2.8673    0.2730    0.8216    0.9128   17.6696    2.0909

// #define LQR11 (-13.8467f)
// #define LQR12 (-1.3234f)
// #define LQR13 (-3.0537f)
// #define LQR14 (-3.5599f)
// #define LQR15 (5.3132f)
// #define LQR16 (0.8911f)
// #define LQR21 (2.8673f)
// #define LQR22 (0.2730f)
// #define LQR23 (0.8216f)
// #define LQR24 (0.9128f)
// #define LQR25 (17.6696f)
// #define LQR26 (2.0909f)

// -11.6022   -1.0905   -0.4467   -0.9343    3.6821    0.7051
//    25.5971    2.6293    2.4515    4.7956   10.5535    0.6999

// #define LQR11 (-11.6022f)
// #define LQR12 (-1.0905f)
// #define LQR13 (-0.4467f)
// #define LQR14 (-0.9343f)
// #define LQR15 (3.6821f)
// #define LQR16 (0.7051f)
// #define LQR21 (25.5971f)
// #define LQR22 (2.6293f)
// #define LQR23 (2.4515f)
// #define LQR24 (4.7956f)
// #define LQR25 (10.5535f)
// #define LQR26 (0.6999f)

// -8.5616   -0.9434   -0.5700   -1.0551    2.7670    0.5010
//    12.9815    1.5123    1.8716    3.2300   13.7888    0.8987

// #define LQR11 (-8.5616f)
// #define LQR12 (-0.9434f)
// #define LQR13 (-0.5700f)
// #define LQR14 (-1.0551f)
// #define LQR15 (2.7670f)
// #define LQR16 (0.5010f)
// #define LQR21 (12.9815f)
// #define LQR22 (1.5123f)
// #define LQR23 (1.8716f)
// #define LQR24 (3.2300f)
// #define LQR25 (13.7888f)
// #define LQR26 (0.8987f)

// #define LQR11 (-6.4712f)
// #define LQR12 (-0.5690f)
// #define LQR13 (-0.8405f)
// #define LQR14 (-1.1326f)
// #define LQR15 (3.3275f)
// #define LQR16 (0.4917f)
// #define LQR21 (6.7619f)
// #define LQR22 (0.7195f)
// #define LQR23 (1.7135f)
// #define LQR24 (2.1867f)
// #define LQR25 (14.7292f)
// #define LQR26 (1.1761f)

// #define LQR11 (-6.9907f)
// #define LQR12 (-0.6272f)
// #define LQR13 (-0.9798f)
// #define LQR14 (-1.3101f)
// #define LQR15 (2.3855f)
// #define LQR16 (0.6274f)
// #define LQR21 (2.6991f)
// #define LQR22 (0.2555f)
// #define LQR23 (0.6319f)
// #define LQR24 (0.8032f)
// #define LQR25 (31.3791f)
// #define LQR26 (7.1274f)

// #define LQR11 (-7.2780f)
// #define LQR12 (-0.7158f)
// #define LQR13 (-2.5650f)
// #define LQR14 (-1.9524f)
// #define LQR15 (3.6078f)
// #define LQR16 (0.5395f)
// #define LQR21 (8.5704f)
// #define LQR22 (1.0448f)
// #define LQR23 (5.8489f)
// #define LQR24 (4.0098f)
// #define LQR25 (14.1205f)
// #define LQR26 (1.0713f)

// // THIS WORKED IN A GOOD BUT WRONG WAY
//  #define LQR11 (-8.0085f)
//  #define LQR12 (-0.8489f)
//  #define LQR13 (-0.7628f)
//  #define LQR14 (-2.0278f)
//  #define LQR15 (3.9863f)
//  #define LQR16 (0.5810f)
//  #define LQR21 (11.7007f)
//  #define LQR22 (1.5637f)
//  #define LQR23 (2.0447f)
//  #define LQR24 (5.2835f)
//  #define LQR25 (13.1109f)
//  #define LQR26 (0.9254f)

// #define LQR11 (-8.1439f)
// #define LQR12 (-0.8633f)
// #define LQR13 (-0.5727f)
// #define LQR14 (-2.0504f)
// #define LQR15 (4.5998f)
// #define LQR16 (0.6262f)
// #define LQR21 (10.5315f)
// #define LQR22 (1.3986f)
// #define LQR23 (1.3117f)
// #define LQR24 (4.6094f)
// #define LQR25 (18.0709f)
// #define LQR26 (1.2444f)

// #define LQR11 (-9.8009f)
// #define LQR12 (-1.1325f)
// #define LQR13 (-0.5933f)
// #define LQR14 (-2.8804f)
// #define LQR15 (3.5238f)
// #define LQR16 (0.8241f)
// #define LQR21 (12.1023f)
// #define LQR22 (1.6977f)
// #define LQR23 (1.2166f)
// #define LQR24 (5.8249f)
// #define LQR25 (14.4089f)
// #define LQR26 (2.6404f)

// #define LQR11 (-9.8009f)
// #define LQR12 (-1.1325f)
// #define LQR13 (-0.5933f)
// #define LQR14 (-2.8804f)
// #define LQR15 (3.5238f)
// #define LQR16 (0.8241f)
// #define LQR21 (12.1023f)
// #define LQR22 (1.6977f)
// #define LQR23 (1.2166f)
// #define LQR24 (5.8249f)
// #define LQR25 (14.4089f)
// #define LQR26 (2.6404f)

// #define LQR11 (-12.8471f)
// #define LQR12 (-1.6197f)
// #define LQR13 (-0.8867f)
// #define LQR14 (-4.2930f)
// #define LQR15 (4.2940f)
// #define LQR16 (1.0118f)
// #define LQR21 (10.0015f)
// #define LQR22 (1.4477f)
// #define LQR23 (1.0338f)
// #define LQR24 (4.9366f)
// #define LQR25 (15.3530f)
// #define LQR26 (2.8595f)

// #define LQR11 (-13.3368f)
// #define LQR12 (-1.6882f)
// #define LQR13 (-0.9856f)
// #define LQR14 (-4.7514f)
// #define LQR15 (3.1273f)
// #define LQR16 (1.3094f)
// #define LQR21 (3.7051f)
// #define LQR22 (0.4972f)
// #define LQR23 (0.3777f)
// #define LQR24 (1.7988f)
// #define LQR25 (31.5068f)
// #define LQR26 (14.0078f)

// #define LQR11 (-13.3305f)
// #define LQR12 (-1.6872f)
// #define LQR13 (-0.9847f)
// #define LQR14 (-4.7470f)
// #define LQR15 (2.4800f)
// #define LQR16 (1.3062f)
// #define LQR21 (3.7876f)
// #define LQR22 (0.5094f)
// #define LQR23 (0.3895f)
// #define LQR24 (1.8541f)
// #define LQR25 (22.3546f)
// #define LQR26 (13.9640f)

// #define LQR11 (-12.6116f)
// #define LQR12 (-1.5805f)
// #define LQR13 (-0.8983f)
// #define LQR14 (-4.3370f)
// #define LQR15 (5.0931f)
// #define LQR16 (1.0251f)
// #define LQR21 (9.1794f)
// #define LQR22 (1.3210f)
// #define LQR23 (0.9825f)
// #define LQR24 (4.6830f)
// #define LQR25 (20.1233f)
// #define LQR26 (2.9694f)

// #define LQR11 (-5.7018f)
// #define LQR12 (-0.4978f)
// #define LQR13 (-0.5241f)
// #define LQR14 (-0.9153f)
// #define LQR15 (2.6358f)
// #define LQR16 (0.4076f)
// #define LQR21 (16.4323f)
// #define LQR22 (1.9065f)
// #define LQR23 (3.7176f)
// #define LQR24 (6.0888f)
// #define LQR25 (6.9894f)
// #define LQR26 (0.3000f)

// void LQR_Output(lqr_u_t *u, lqr_ss_t *state)
// {
//     static float k[12][3] = {
//     {64.570926, -78.151551, -15.098827},
//     {1.716153, -4.573289, -0.526836},
//     {25.565621, -20.634320, -17.563574},
//     {11.633020, -11.021911, -13.087704},
//     {212.754066, -196.450645, 56.227305},
//     {12.051262, -11.941105, 4.523285},
//     {74.149148, -71.296364, 23.673363},
//     {4.449069, -4.214434, 1.145555},
//     {115.321993, -106.373546, 29.626387},
//     {82.459182, -74.915697, 20.072512},
//     {-163.793049, 130.808796, 114.419419},
//     {-13.396998, 11.357026, 4.288949}};
//     float l = state->leg_len;
//     float lsqr = l * l;

//     uint8_t i, j;

//     // 离地时轮子输出置0
//     i = 0;
//     j = i * 6;
//     u->T_A = (k[j + 0][0] * lsqr + k[j + 0][1] * l + k[j + 0][2]) * -state->theta +
//              (k[j + 1][0] * lsqr + k[j + 1][1] * l + k[j + 1][2]) * -state->theta_dot +
//              (k[j + 2][0] * lsqr + k[j + 2][1] * l + k[j + 2][2]) * -state->x +
//              (k[j + 3][0] * lsqr + k[j + 3][1] * l + k[j + 3][2]) * -state->x_dot +
//              (k[j + 4][0] * lsqr + k[j + 4][1] * l + k[j + 4][2]) * -state->phi +
//              (k[j + 5][0] * lsqr + k[j + 5][1] * l + k[j + 5][2]) * -state->phi_dot;

//     // 离地时关节输出仅保留 theta 和 theta_dot，保证滞空时腿部竖直
//     i = 1;
//     j = i * 6;
//     u->T_B = (k[j + 0][0] * lsqr + k[j + 0][1] * l + k[j + 0][2]) * -state->theta +
//              (k[j + 1][0] * lsqr + k[j + 1][1] * l + k[j + 1][2]) * -state->theta_dot +
//              (k[j + 2][0] * lsqr + k[j + 2][1] * l + k[j + 2][2]) * -state->x +
//              (k[j + 3][0] * lsqr + k[j + 3][1] * l + k[j + 3][2]) * -state->x_dot +
//              (k[j + 4][0] * lsqr + k[j + 4][1] * l + k[j + 4][2]) * -state->phi +
//              (k[j + 5][0] * lsqr + k[j + 5][1] * l + k[j + 5][2]) * -state->phi_dot;
// }

//   -32.9564   -4.7071  -20.1324  -15.9528   32.6625    2.5431
//    25.1663    3.9483   19.4616   14.6525  126.7494    4.1493

// #define LQR11 (-32.9564f)
// #define LQR12 (-4.7071f)
// #define LQR13 (-20.1324f)
// #define LQR14 (-15.9528f)
// #define LQR15 (32.6625f)
// #define LQR16 (2.5431f)
// #define LQR21 (25.1663f)
// #define LQR22 (3.9483f)
// #define LQR23 (19.4616f)
// #define LQR24 (14.6525f)
// #define LQR25 (126.7494f)
// #define LQR26 (4.1493f)

// THIS WORKED IN A GOOD BUT WRONG WAY
//  #define LQR11 (-8.0085f)
//  #define LQR12 (-0.8489f)
//  #define LQR13 (-0.7628f)
//  #define LQR14 (-2.0278f)
//  #define LQR15 (3.9863f)
//  #define LQR16 (0.5810f)
//  #define LQR21 (11.7007f)
//  #define LQR22 (1.5637f)
//  #define LQR23 (2.0447f)
//  #define LQR24 (5.2835f)
//  #define LQR25 (13.1109f)
//  #define LQR26 (0.9254f)

//    -6.8616   -0.4879   -0.3917   -0.7308    2.9359    0.4908
//    23.1611    2.3407    4.0679    7.0680    5.0417    0.1615
// #define LQR11 (-6.8616f)
// #define LQR12 (-0.4879f)
// #define LQR13 (-0.3917f)
// #define LQR14 (-0.7308f)
// #define LQR15 (2.9359f)
// #define LQR16 (0.4908f)
// #define LQR21 (23.1611f)
// #define LQR22 (2.3407f)
// #define LQR23 (4.0679f)
// #define LQR24 (7.0680f)
// #define LQR25 (5.0417f)
// #define LQR26 (0.1615f)

//    -7.6990   -0.5859   -0.8094   -1.1081    4.4033    0.6204
//    22.1013    2.3409    5.9380    7.5638   11.7444    0.5077
// #define LQR11 (-7.6990f)
// #define LQR12 (-0.5859f)
// #define LQR13 (-0.8094f)
// #define LQR14 (-1.1081f)
// #define LQR15 (4.4033f)
// #define LQR16 (0.6204f)
// #define LQR21 (22.1013f)
// #define LQR22 (2.3409f)
// #define LQR23 (5.9380f)
// #define LQR24 (7.5638f)
// #define LQR25 (11.7444f)
// #define LQR26 (0.5077f)


//   -11.4305   -1.1382   -2.6668   -3.1192    6.0341    0.8530
//     9.3985    1.1668    3.8002    4.2159   19.1942    1.5592
// #define LQR11 (-11.4305f)
// #define LQR12 (-1.1382f)
// #define LQR13 (-2.6668f)
// #define LQR14 (-3.1192f)
// #define LQR15 (6.0341f)
// #define LQR16 (0.8530f)
// #define LQR21 (9.3985f)
// #define LQR22 (1.1668f)
// #define LQR23 (3.8002f)
// #define LQR24 (4.2159f)
// #define LQR25 (19.1942f)
// #define LQR26 (1.5592f)

//    -9.0381   -0.8002   -1.5454   -1.8952    7.0606    0.8181
//    29.8781    3.7447   11.3292   12.9867   25.9207    0.8019
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

static float lqr_param[12];
    u->T_A = (lqr_poly_fit_param[0 ][0] * len_cub + lqr_poly_fit_param[0 ][1] * len_sqrt + lqr_poly_fit_param[0 ][2] * len + lqr_poly_fit_param[0 ][3]) * -state->theta +
             (lqr_poly_fit_param[1 ][0] * len_cub + lqr_poly_fit_param[1 ][1] * len_sqrt + lqr_poly_fit_param[1 ][2] * len + lqr_poly_fit_param[1 ][3]) * -state->theta_dot +
             (lqr_poly_fit_param[2 ][0] * len_cub + lqr_poly_fit_param[2 ][1] * len_sqrt + lqr_poly_fit_param[2 ][2] * len + lqr_poly_fit_param[2 ][3]) * -state->x +
             (lqr_poly_fit_param[3 ][0] * len_cub + lqr_poly_fit_param[3 ][1] * len_sqrt + lqr_poly_fit_param[3 ][2] * len + lqr_poly_fit_param[3 ][3]) * -state->x_dot +
             (lqr_poly_fit_param[4 ][0] * len_cub + lqr_poly_fit_param[4 ][1] * len_sqrt + lqr_poly_fit_param[4 ][2] * len + lqr_poly_fit_param[4 ][3]) * -state->phi +
             (lqr_poly_fit_param[5 ][0] * len_cub + lqr_poly_fit_param[5 ][1] * len_sqrt + lqr_poly_fit_param[5 ][2] * len + lqr_poly_fit_param[5 ][3]) * -state->phi_dot;
    u->T_B = (lqr_poly_fit_param[6 ][0] * len_cub + lqr_poly_fit_param[6 ][1] * len_sqrt + lqr_poly_fit_param[6 ][2] * len + lqr_poly_fit_param[6 ][3]) * -state->theta +
             (lqr_poly_fit_param[7 ][0] * len_cub + lqr_poly_fit_param[7 ][1] * len_sqrt + lqr_poly_fit_param[7 ][2] * len + lqr_poly_fit_param[7 ][3]) * -state->theta_dot +
             (lqr_poly_fit_param[8 ][0] * len_cub + lqr_poly_fit_param[8 ][1] * len_sqrt + lqr_poly_fit_param[8 ][2] * len + lqr_poly_fit_param[8 ][3]) * -state->x +
             (lqr_poly_fit_param[9 ][0] * len_cub + lqr_poly_fit_param[9 ][1] * len_sqrt + lqr_poly_fit_param[9 ][2] * len + lqr_poly_fit_param[9 ][3]) * -state->x_dot +
             (lqr_poly_fit_param[10][0] * len_cub + lqr_poly_fit_param[10][1] * len_sqrt + lqr_poly_fit_param[10][2] * len + lqr_poly_fit_param[10][3]) * -state->phi +
             (lqr_poly_fit_param[11][0] * len_cub + lqr_poly_fit_param[11][1] * len_sqrt + lqr_poly_fit_param[11][2] * len + lqr_poly_fit_param[11][3]) * -state->phi_dot;
// #ifdef OPENSOURCE_COORDINATE
//     u->T_A = -(LQR13 * state->x + LQR14 * state->x_dot + LQR11 * state->theta + LQR12 * state->theta_dot + LQR15 * state->phi + LQR16 * state->phi_dot);
//     u->T_B = (LQR23 * state->x + LQR24 * state->x_dot + LQR21 * state->theta + LQR22 * state->theta_dot + LQR25 * state->phi + LQR26 * state->phi_dot);
// #else
//     u->T_A = -(LQR11 * 0 * state->x + LQR12 * state->x_dot + LQR13 * state->theta + LQR14 * state->theta_dot + LQR15 * state->phi + LQR16 * state->phi_dot);
//     u->T_B = -(LQR21 * 0 * state->x + LQR22 * state->x_dot + LQR23 * state->theta + LQR24 * state->theta_dot + LQR25 * state->phi + LQR26 * state->phi_dot);
// #endif
}
