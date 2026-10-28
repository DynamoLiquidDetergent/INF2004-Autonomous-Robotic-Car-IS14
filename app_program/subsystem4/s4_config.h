#ifndef S4_CONFIG_H
#define S4_CONFIG_H
#define S4_PHASE                 3
#define S4_UNPACKED_BASELINE      0
#define S4_OUTPUT_MAX_BYTES       32U
#define S4_PRIORITY              18
#define S4_PERIOD_MS             20U
#define S4_KEEP_EXAMPLE_BRIDGE    1
#define S4_LOG_EVERY_CYCLES       50U
#define S4_SENSOR_TIMEOUT_US     3000U
#define S4_PERIPHERAL_CLOCK_HZ    125000000U
/* Configure after physical axis check: nose-up must give positive pitch. */
#define S4_AXIS_X                0
#define S4_AXIS_Y                1
#define S4_AXIS_Z                2
#define S4_SIGN_X                1
#define S4_SIGN_Y                1
#define S4_SIGN_Z                1
/* Set to 1 only after hard/soft-iron calibration and motor-on validation. */
#define S4_MAG_CALIBRATED        0
#define S4_MAG_BIAS_X            0.0F
#define S4_MAG_BIAS_Y            0.0F
#define S4_MAG_BIAS_Z            0.0F
#define S4_MAG_SCALE_X           1.0F
#define S4_MAG_SCALE_Y           1.0F
#define S4_MAG_SCALE_Z           1.0F
#if S4_AXIS_X > 2 || S4_AXIS_Y > 2 || S4_AXIS_Z > 2
#error Invalid S4 axis index
#endif
#if S4_AXIS_X == S4_AXIS_Y || S4_AXIS_X == S4_AXIS_Z || S4_AXIS_Y == S4_AXIS_Z
#error S4 axes must be a permutation
#endif
#if (S4_SIGN_X != 1 && S4_SIGN_X != -1) || (S4_SIGN_Y != 1 && S4_SIGN_Y != -1) || (S4_SIGN_Z != 1 && S4_SIGN_Z != -1)
#error Invalid S4 axis sign
#endif
#endif
