/*
 * tvc_config.h - every tunable in the flight software, in one place.
 *
 * Section 1 is copied from the Simulink model PID/thisistheend_v2.slx
 * (model workspace + PID/setup.m). Change it only together with the model,
 * then re-run verify/run_verification.m.
 *
 * Section 2 is hardware calibration for the TVC_Raptor board and your
 * airframe. These MUST be checked on the bench before flight (README.md,
 * "Bench calibration").
 */
#ifndef TVC_CONFIG_H
#define TVC_CONFIG_H

/* ======================================================================
 * 1. Values from the Simulink model (thisistheend_v2)
 * ====================================================================== */

/* --- Timing ----------------------------------------------------------- */
#define TVC_CTRL_DT_S             0.001   /* firmware control loop (IMU ODR 1 kHz) */
#define TVC_UKF_TS_S              0.01    /* ukfTs                               */
#define TVC_UKF_DECIMATION        10      /* TVC_UKF_TS_S / TVC_CTRL_DT_S         */
#define TVC_BURN_TIME_S           3.44    /* setup.m burnTime                    */

/* --- Attitude reference ----------------------------------------------- */
/* q_des = Angle2Quat(ZYX, [0 pi/2 0]): nose straight up in NED.
 * Quaternions are scalar-first, NED -> body (Aerospace Blockset convention). */
#define TVC_QDES_W                0.70710678118654752
#define TVC_QDES_X                0.0
#define TVC_QDES_Y                0.70710678118654752
#define TVC_QDES_Z                0.0

/* --- PID controllers (parallel form, back-calculation anti-windup) ----
 * "PID Controller1" drives pitch, "PID Controller" drives yaw.            */
#define TVC_PITCH_KP              1.5
#define TVC_PITCH_KI              0.3
#define TVC_PITCH_KD              0.06
#define TVC_PITCH_N               50.0
#define TVC_YAW_KP                1.5
#define TVC_YAW_KI                0.3
#define TVC_YAW_KD                0.05
#define TVC_YAW_N                 100.0
#define TVC_PID_KB                1.0     /* back-calculation gain           */

/* Rate damping on the measured gyro (deg/s in, deg out).
 * Model: Pitch_Rate_Damping = -Controller_PitchRateDamping,
 *        Yaw_Rate_Damping   = +Controller_YawRateDamping
 * (the yaw sign is opposite because the yaw error has gain -2).           */
#define TVC_PITCH_RATE_DAMPING    0.1
#define TVC_YAW_RATE_DAMPING      0.1

/* --- Actuator limits -------------------------------------------------- */
#define TVC_MAX_GIMBAL_DEG        12.0    /* max_gimbal: PID clamp + Saturation */
#define TVC_GIMBAL_SLEW_DEG_S     545.45  /* Rate Limiter                     */

/* --- UKF (Rocket_UKF block) ------------------------------------------- */
#define TVC_UKF_ALPHA             1e-3
#define TVC_UKF_BETA              2.0
#define TVC_UKF_KAPPA             0.0

/* ukfQ: quaternion block 1.9049e-9*I4, position/velocity coupled blocks */
#define TVC_UKF_Q_QUAT            1.9049e-09
#define TVC_UKF_Q_POS             2.25e-06
#define TVC_UKF_Q_POSVEL          4.5e-04
#define TVC_UKF_Q_VEL             0.09

/* ukfP0 diagonal */
#define TVC_UKF_P0_QW             1e-10
#define TVC_UKF_P0_QXYZ           3.0462e-04
#define TVC_UKF_P0_POS            0.01
#define TVC_UKF_P0_VEL            4.0

/* Measurement noise.
 * NOTE: TVC_UKF_R_GPS_DOWN (2 cm std) is the model value and is far
 * tighter than a real GNSS vertical fix (metres). Position/velocity do not
 * feed the controller in this design, so it only affects the logged
 * altitude estimate. Raise it (e.g. 9.0) if you care about that estimate. */
#define TVC_UKF_R_ACCEL           9e-04   /* ukfRAccel (per axis)              */
#define TVC_UKF_R_GPS_DOWN        4e-04   /* ukfRGps(3,3)                      */
#define TVC_UKF_R_BARO            0.01    /* ukfRBaro                          */

/* Accelerometer noise schedule vs |thrust| (Accel_Noise_Schedule) and its
 * first-order blend (Accel_Noise_Blend, tau = Estimator_AccelBlendTau).   */
#define TVC_ACCEL_BLEND_TAU_S     0.25
#define TVC_ACCEL_SCHED_N         5
#define TVC_ACCEL_SCHED_THRUST_N  { 0.0, 0.5, 2.0, 5.0, 20.0 }
#define TVC_ACCEL_SCHED_SCALE     { 1.0, 1.0, 100.0, 1000.0, 1000.0 }
/* Blend filter initial state = Estimator_AccelNoiseScale(end). This is the
 * Direct form II state, not the output (the output starts near 37). The
 * model starts at ignition, so the firmware loads this at launch detect. */
#define TVC_ACCEL_BLEND_INIT      1000.0

/* Motor thrust curve (1-D Lookup Table), N vs s since ignition.         */
#define TVC_THRUST_N_POINTS       23
#define TVC_THRUST_TIME_S { 0.0, 0.15, 0.23, 0.29, 0.35, 0.38, 0.42, 0.48, \
                            0.52, 0.69, 0.85, 1.03, 1.2, 1.42, 1.5, 1.96, \
                            2.21, 2.49, 2.77, 3.37, 3.4, 3.41, 3.44 }
#define TVC_THRUST_FORCE_N { 0.0, 7.8, 12.2, 16.5, 20.2, 22.8, 25.2, 23.1, \
                             20.8, 17.5, 16.2, 15.4, 14.9, 14.6, 14.8, 14.3, \
                             14.1, 13.8, 13.3, 13.0, 9.3, 4.9, 0.0 }

#define TVC_GRAVITY_MPS2          9.80665

/* ======================================================================
 * 2. Hardware / airframe calibration - CHECK ON THE BENCH
 * ====================================================================== */

/* Body axes follow the model: +X out of the nose, +Y and +Z complete a
 * right-handed frame. Positive pitch gimbal produces body +Z force and +My;
 * positive yaw gimbal produces body +Y force and -Mz ("MASS and Geometry").
 *
 * IMU -> body rotation, row-major: v_body = R * v_imu.
 * Default assumes the ICM-42688 X axis points at the nose. Fix it to match
 * how the board is mounted (README.md, step 2).                          */
#define TVC_R_BODY_IMU { 1.0f, 0.0f, 0.0f, \
                         0.0f, 1.0f, 0.0f, \
                         0.0f, 0.0f, 1.0f }

/* Servo outputs (TIM1). Channel 0..3 = J1, J9, J10, J11.                 */
#define TVC_SERVO_CH_PITCH        0       /* J1 */
#define TVC_SERVO_CH_YAW          1       /* J9 */
#define TVC_SERVO_PWM_HZ          333     /* digital servo frame rate; 50 for analog */

/* Gimbal angle -> pulse: us = center + gain * deg. gain carries both the
 * linkage ratio and the direction (negative to reverse).                  */
#define TVC_SERVO_PITCH_CENTER_US 1500.0f
#define TVC_SERVO_PITCH_US_PER_DEG 10.0f
#define TVC_SERVO_YAW_CENTER_US   1500.0f
#define TVC_SERVO_YAW_US_PER_DEG  10.0f
#define TVC_SERVO_MIN_US          1000.0f /* hard pulse clamp for all channels */
#define TVC_SERVO_MAX_US          2000.0f

/* --- Flight sequencing ----------------------------------------------- */
#define TVC_CAL_TIME_S            2.0     /* stationary averaging after ARM          */
#define TVC_CAL_MAX_GYRO_DPS      2.0     /* reject calibration if moving faster     */
#define TVC_CAL_ACCEL_TOL_G       0.10    /* |a| must be 1 g within this             */
#define TVC_MAX_PAD_TILT_DEG      15.0    /* refuse to arm if tilted more than this  */
#define TVC_LAUNCH_ACCEL_G        2.0     /* body-X specific force for launch detect */
#define TVC_LAUNCH_HOLD_S         0.05    /* ... sustained this long                 */
#define TVC_BURNOUT_ACCEL_G       0.5     /* early burnout when axial accel drops    */
#define TVC_BURNOUT_HOLD_S        0.10
#define TVC_BURNOUT_MIN_T_S       1.0     /* ignore burnout detect before this       */
#define TVC_LANDED_STILL_S        5.0
#define TVC_LANDED_MIN_T_S        10.0

/* --- Logging --------------------------------------------------------- */
#define TVC_LOG_MIN_FREE_BYTES    (2u * 1024u * 1024u) /* refuse to arm below this */

/* --- Telemetry ------------------------------------------------------- */
/* 915 MHz band. Set to 0 to disable the radio. NOTE: the Wio-SX1262 ANT
 * pad is unconnected on TVC_Raptor rev A - fit an antenna before relying
 * on the downlink.                                                       */
#define TVC_LORA_FREQ_HZ          915000000u
#define TVC_LORA_TX_DBM           14      /* -9..22 */
#define TVC_LORA_SF               7
#define TVC_LORA_PERIOD_MS        500u    /* check your duty-cycle limits */

#define TVC_PI_BAUD               115200u
#define TVC_PI_PERIOD_MS          50u     /* 20 Hz CSV stream on UART7 */

/* --- Battery ---------------------------------------------------------- */
#define TVC_VBAT_DIVIDER          (47.0f / 147.0f)  /* R66 / (R65 + R66) */
#define TVC_VBAT_MIN_ARM_V        7.0f

#endif /* TVC_CONFIG_H */
