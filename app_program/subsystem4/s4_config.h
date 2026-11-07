#ifndef S4_CONFIG_H
#define S4_CONFIG_H
#define S4_PHASE                 5
#define S4_UNPACKED_BASELINE      0
#define S4_OUTPUT_MAX_BYTES       32U
#define S4_PRIORITY              18
#define S4_PERIOD_MS             20U
#define S4_KEEP_EXAMPLE_BRIDGE    1
#define S4_LOG_EVERY_CYCLES       50U
#define S4_SENSOR_TIMEOUT_US     3000U
#define S4_PERIPHERAL_CLOCK_HZ    125000000U
/* Configure after physical axis check: nose-up must give positive pitch. */
#define S4_AXIS_X                1
#define S4_AXIS_Y                0
#define S4_AXIS_Z                2
#define S4_SIGN_X               -1
#define S4_SIGN_Y                1
#define S4_SIGN_Z                1
/* The integration build helper stamps its scratch image, not this checkout. */
#ifndef S4_BUILD_TAG
#define S4_BUILD_TAG             "UNSTAMPED"
#endif
#ifndef S4_BUILD_PROFILE
#define S4_BUILD_PROFILE         "REPO"
#endif
/* Test-image only: 0 off, 1 powered flat-distance run, 2 one powered hump.
 * S5 remains the only motion-command sender; it uses its existing ABORT stop. */
#ifndef S4_POWERED_BENCH_KIND
#define S4_POWERED_BENCH_KIND    0
#endif
/* Right-wheel-only model: measured tyre centre-to-centre spacing.
 * Left tyre must stay on the same flat floor throughout the crossing. */
#define S4_RIGHT_TRACK_MM        120.0F
/* Hump height: drive-axle centre to caster ground contact, in mm.
 * Measured 110 mm; used by the legacy both-supports model. 0 = legacy integral.
 * Must be below S4_HEIGHT_TRACK_LEN (192). */
#define S4_HEIGHT_WHEELBASE_MM   110U
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
