/*
 * tvc_control.c - see tvc_control.h. Pure C99, shared with the MEX build.
 */
#include "tvc_control.h"
#include "tvc_config.h"

#include <math.h>
#include <string.h>

#define RAD2DEG (180.0 / 3.14159265358979323846)

static double clamp(double v, double lo, double hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* ---------------------------------------------------------------------
 * Estimator
 * ------------------------------------------------------------------- */

void tvc_est_init(tvc_est_t *e, const double q0[4], double blend_init)
{
    memset(e, 0, sizeof *e);
    tvc_ukf_init(&e->ukf, q0);
    e->blend.w = blend_init;
    for (int i = 0; i < 4; i++) {
        e->q_delay[i] = q0[i];   /* Unit Delay1 IC = ukfX0(1:4) */
        e->q_out[i] = q0[i];
    }
}

bool tvc_est_step(tvc_est_t *e, const tvc_est_input_t *in)
{
    bool ok = true;

    /* Accel_Noise_Schedule -> Accel_Noise_Blend -> Accel_Noise_Covariance */
    double scale = tvc_accel_blend_step(&e->blend, tvc_accel_noise_schedule(in->thrust_n));
    e->r_accel = TVC_UKF_R_ACCEL * scale;

    /* Normalize_Accel, Accel_Frame_Correction (-I) */
    double an = sqrt(in->accel[0] * in->accel[0] + in->accel[1] * in->accel[1] +
                     in->accel[2] * in->accel[2]);
    if (an < 1e-6) an = 1e-6;
    double z[3] = { -in->accel[0] / an, -in->accel[1] / an, -in->accel[2] / an };

    ok &= tvc_ukf_correct_accel(&e->ukf, z, e->r_accel);
    if (in->has_gps) ok &= tvc_ukf_correct_down(&e->ukf, in->gps_down_m, TVC_UKF_R_GPS_DOWN);
    if (in->has_baro) ok &= tvc_ukf_correct_down(&e->ukf, in->baro_down_m, TVC_UKF_R_BARO);

    /* Normalize_Quaternion -> Unit Delay1 */
    const double *q = e->ukf.x;
    double qn = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    memcpy(e->q_out, e->q_delay, sizeof e->q_out);
    for (int i = 0; i < 4; i++) e->q_delay[i] = q[i] / qn;

    ok &= tvc_ukf_predict(&e->ukf, in->gyro, TVC_UKF_TS_S);
    return ok;
}

/* ---------------------------------------------------------------------
 * Controller
 * ------------------------------------------------------------------- */

void tvc_ctrl_reset(tvc_ctrl_t *c, double e_pitch_deg, double e_yaw_deg)
{
    memset(c, 0, sizeof *c);
    c->pitch.kp = TVC_PITCH_KP;
    c->pitch.ki = TVC_PITCH_KI;
    c->pitch.kd = TVC_PITCH_KD;
    c->pitch.n = TVC_PITCH_N;
    c->pitch.filt = e_pitch_deg;
    c->yaw.kp = TVC_YAW_KP;
    c->yaw.ki = TVC_YAW_KI;
    c->yaw.kd = TVC_YAW_KD;
    c->yaw.n = TVC_YAW_N;
    c->yaw.filt = e_yaw_deg;
}

void tvc_attitude_error(const double q_est[4], double *e_pitch_deg, double *e_yaw_deg)
{
    /* Quaternion Inverse: conj(q) / |q|^2 */
    double n2 = q_est[0] * q_est[0] + q_est[1] * q_est[1] +
                q_est[2] * q_est[2] + q_est[3] * q_est[3];
    double a0 = q_est[0] / n2, a1 = -q_est[1] / n2, a2 = -q_est[2] / n2, a3 = -q_est[3] / n2;

    /* Quaternion Multiplication: inv(q_est) (x) q_des */
    const double b0 = TVC_QDES_W, b1 = TVC_QDES_X, b2 = TVC_QDES_Y, b3 = TVC_QDES_Z;
    double w = a0 * b0 - a1 * b1 - a2 * b2 - a3 * b3;
    double y = a0 * b2 + b0 * a2 + (a3 * b1 - a1 * b3);
    double z = a0 * b3 + b0 * a3 + (a1 * b2 - a2 * b1);

    double s = (w > 0.0) ? 1.0 : ((w < 0.0) ? -1.0 : 0.0);   /* Sign */
    *e_pitch_deg = RAD2DEG * (2.0 * asin(clamp(s * y, -1.0, 1.0)));
    *e_yaw_deg = RAD2DEG * (-2.0 * asin(clamp(s * z, -1.0, 1.0)));
}

bool tvc_quat_from_gravity(const double down_body[3], double q[4])
{
    /* Gravity in body axes at q_des is gd = [-1 0 0]. With q = q_des (x) d,
     * h(q) = C(d) gd, so d is the frame rotation by -phi about
     * a = gd x g, where phi is the angle from gd to g. */
    const double gd[3] = { -1.0, 0.0, 0.0 };
    const double *g = down_body;
    double a[3] = { gd[1] * g[2] - gd[2] * g[1],
                    gd[2] * g[0] - gd[0] * g[2],
                    gd[0] * g[1] - gd[1] * g[0] };
    double s = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    double c = gd[0] * g[0] + gd[1] * g[1] + gd[2] * g[2];
    if (c <= 0.0) return false;

    double d[4] = { 1.0, 0.0, 0.0, 0.0 };
    if (s > 1e-12) {
        double phi = atan2(s, c);
        double k = -sin(0.5 * phi) / s;
        d[0] = cos(0.5 * phi);
        d[1] = k * a[0];
        d[2] = k * a[1];
        d[3] = k * a[2];
    }
    const double p0 = TVC_QDES_W, p1 = TVC_QDES_X, p2 = TVC_QDES_Y, p3 = TVC_QDES_Z;
    q[0] = p0 * d[0] - p1 * d[1] - p2 * d[2] - p3 * d[3];
    q[1] = p0 * d[1] + d[0] * p1 + (p2 * d[3] - p3 * d[2]);
    q[2] = p0 * d[2] + d[0] * p2 + (p3 * d[1] - p1 * d[3]);
    q[3] = p0 * d[3] + d[0] * p3 + (p1 * d[2] - p2 * d[1]);
    return true;
}

double tvc_tilt_deg(const double q[4])
{
    /* Nose (+X body) vs up: up in body axes is minus the gravity direction,
     * whose X component is 2 (qx qz - qw qy). */
    double up_x = -2.0 * (q[1] * q[3] - q[0] * q[2]);
    return RAD2DEG * acos(clamp(up_x, -1.0, 1.0));
}

/* Parallel PID with filtered derivative, output clamp and
 * back-calculation anti-windup (Simulink PID Controller block). */
double tvc_pid_step(tvc_pid_t *p, double e, double dt)
{
    const double lim = TVC_MAX_GIMBAL_DEG;
    const double decay = 1.0 - exp(-p->n * dt);

    double d = p->kd * p->n * (e - p->filt);
    double u_unsat = p->kp * e + p->integ + d;
    double u = clamp(u_unsat, -lim, lim);

    /* The derivative term decays as exp(-N t) within the sample (e is held),
     * so feed its exact average into the anti-windup path. Plain forward
     * Euler here drifts by ~0.3 deg after a large error step at the clamp. */
    double d_avg = p->kd * (e - p->filt) * decay / dt;
    double u_unsat_avg = p->kp * e + p->integ + d_avg;
    double u_avg = clamp(u_unsat_avg, -lim, lim);
    p->integ += dt * (p->ki * e + TVC_PID_KB * (u_avg - u_unsat_avg));

    p->filt += decay * (e - p->filt);
    return u;
}

void tvc_ctrl_step(tvc_ctrl_t *c, const double q_est[4],
                   double rate_q, double rate_r, double dt)
{
    tvc_attitude_error(q_est, &c->err_pitch_deg, &c->err_yaw_deg);

    c->pid_pitch_deg = tvc_pid_step(&c->pitch, c->err_pitch_deg, dt);
    c->pid_yaw_deg = tvc_pid_step(&c->yaw, c->err_yaw_deg, dt);

    c->sum_pitch_deg = c->pid_pitch_deg - TVC_PITCH_RATE_DAMPING * (RAD2DEG * rate_q);
    c->sum_yaw_deg = c->pid_yaw_deg + TVC_YAW_RATE_DAMPING * (RAD2DEG * rate_r);

    const double lim = TVC_MAX_GIMBAL_DEG;
    const double step = TVC_GIMBAL_SLEW_DEG_S * dt;
    double cp = clamp(c->sum_pitch_deg, -lim, lim);
    double cy = clamp(c->sum_yaw_deg, -lim, lim);
    c->out_pitch_deg += clamp(cp - c->out_pitch_deg, -step, step);
    c->out_yaw_deg += clamp(cy - c->out_yaw_deg, -step, step);
}
