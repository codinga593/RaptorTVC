/*
 * tvc_control.h - estimator wrapper and attitude controller from
 * thisistheend_v2 (top level of the model).
 *
 * Estimator (subsystem "State Estimate"):
 *   R_accel = ukfRAccel * Blend(Schedule(|thrust|))
 *   z_accel = -a / max(|a|, 1e-6)
 *   UKF: correct accel, gps down, baro, then predict with the gyro
 *   q_out   = UnitDelay( x(1:4) / |x(1:4)| )
 *
 * Controller:
 *   q_err   = inv(q_est) (x) q_des
 *   e_pitch = deg( 2 asin(sat(sgn(q_err.w) q_err.y)))   -> PID Controller1
 *   e_yaw   = deg(-2 asin(sat(sgn(q_err.w) q_err.z)))   -> PID Controller
 *   pitch   = PID_p - 0.1 * q_deg/s ;  yaw = PID_y + 0.1 * r_deg/s
 *   -> Saturation (+-max_gimbal) -> Rate Limiter (545.45 deg/s)
 * The servo lag, dead zone and backlash after this are plant models.
 */
#ifndef TVC_CONTROL_H
#define TVC_CONTROL_H

#include <stdbool.h>
#include "tvc_ukf.h"

/* ------------------------------------------------------------------ */
typedef struct {
    tvc_ukf_t ukf;
    tvc_accel_blend_t blend;
    double q_delay[4];   /* Unit Delay1 state                        */
    double q_out[4];     /* quaternion handed to the controller      */
    double r_accel;      /* last accel noise variance used           */
} tvc_est_t;

typedef struct {
    double gyro[3];      /* body rates, rad/s                          */
    double accel[3];     /* body specific force, any unit              */
    double thrust_n;     /* thrust-curve value for the noise schedule  */
    bool   has_gps;
    double gps_down_m;   /* NED down position from GNSS, m             */
    bool   has_baro;
    double baro_down_m;  /* NED down position from barometer, m        */
} tvc_est_input_t;

/* blend_init is the Accel_Noise_Blend Direct form II state
 * (TVC_ACCEL_BLEND_INIT reproduces the model's start at ignition). */
void tvc_est_init(tvc_est_t *e, const double q0[4], double blend_init);
/* One UKF sample (TVC_UKF_TS_S). Returns false on a numerical fault. */
bool tvc_est_step(tvc_est_t *e, const tvc_est_input_t *in);

/* ------------------------------------------------------------------ */
typedef struct {
    double kp, ki, kd, n;
    double integ;        /* integrator state (integral of Ki*e)  */
    double filt;         /* derivative filter state              */
} tvc_pid_t;

typedef struct {
    tvc_pid_t pitch, yaw;
    double out_pitch_deg;   /* rate limiter states = gimbal commands */
    double out_yaw_deg;
    /* intermediate signals, for logging and verification */
    double err_pitch_deg, err_yaw_deg;
    double pid_pitch_deg, pid_yaw_deg;
    double sum_pitch_deg, sum_yaw_deg;
} tvc_ctrl_t;

/* Zero integrators and rate limiters. If e_pitch/e_yaw are given, the
 * derivative filters start at that error (no derivative kick on the
 * first sample); pass 0 for the model's zero initial condition. */
void tvc_ctrl_reset(tvc_ctrl_t *c, double e_pitch_deg, double e_yaw_deg);

/* Attitude (NED->body) whose gravity direction in body axes is down_body
 * (unit vector, = -accel/|accel| at rest), choosing the smallest rotation
 * away from q_des. Roll about the nose is not observable from gravity and
 * is taken from q_des. Returns false if down_body is >= 90 deg from the
 * vertical (nose not pointing up). */
bool tvc_quat_from_gravity(const double down_body[3], double q[4]);

/* Nose tilt from vertical (deg) for an attitude quaternion. */
double tvc_tilt_deg(const double q[4]);

/* One PID sample: returns the clamped output (deg). */
double tvc_pid_step(tvc_pid_t *p, double e, double dt);

/* Attitude error angles (deg) of q_est relative to q_des. */
void tvc_attitude_error(const double q_est[4], double *e_pitch_deg, double *e_yaw_deg);

/* One controller step of length dt. rate_q/rate_r in rad/s. */
void tvc_ctrl_step(tvc_ctrl_t *c, const double q_est[4],
                   double rate_q, double rate_r, double dt);

#endif /* TVC_CONTROL_H */
