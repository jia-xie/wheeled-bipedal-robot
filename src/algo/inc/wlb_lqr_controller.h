#ifndef HEADER_NAME_H
#define HEADER_NAME_H

typedef struct lqr_ss_s {
    float x;
    float x_dot;
    float theta;
    float theta_dot;
    float phi;
    float phi_dot;
} lqr_ss_t;

typedef struct lqr_u_s {
    float T_A;
    float T_B;
} lqr_u_t;

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

#endif // HEADER_NAME_H