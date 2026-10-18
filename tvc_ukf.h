/*
 * tvc_ukf.h - port of the Rocket_UKF block in thisistheend_v2.
 *
 * State x = [qw qx qy qz px py pz vx vy vz] (quaternion NED->body,
 * scalar-first; NED position and velocity relative to the pad).
 * Additive-noise UKF, alpha/beta/kappa and weights exactly as MATLAB's
 * matlabshared.tracking.internal implementation (calcUTParameters,
 * UTMeanCov*). MATLAB propagates a square-root covariance; this port
 * propagates the covariance itself, which is algebraically identical.
 *
 * Per step the block runs: correct(accel) -> correct(gps down) ->
 * correct(baro) -> output -> predict(gyro, Ts).
 */
#ifndef TVC_UKF_H
#define TVC_UKF_H

#include <stdbool.h>

#define UKF_N 10

typedef struct {
    double x[UKF_N];
    double P[UKF_N][UKF_N];
    double Q[UKF_N][UKF_N];
    double P0[UKF_N][UKF_N];
    unsigned fault_count;     /* times P lost positive definiteness and was reset */
} tvc_ukf_t;

/* Model ukfX0 / ukfP0 / ukfQ, with the given initial quaternion. */
void tvc_ukf_init(tvc_ukf_t *f, const double q0[4]);

/* Accel: z = -a/|a| (gravity direction in body axes), R = r * I3. */
bool tvc_ukf_correct_accel(tvc_ukf_t *f, const double z[3], double r);
/* GPS down and barometer both measure x(7) = NED down position. */
bool tvc_ukf_correct_down(tvc_ukf_t *f, double z, double r);
/* omega = body rates (rad/s), dt = sample time. */
bool tvc_ukf_predict(tvc_ukf_t *f, const double omega[3], double dt);

/* Accel noise covariance scale: Accel_Noise_Schedule (linear, end-clamped)
 * followed by the Accel_Noise_Blend first-order filter. */
typedef struct {
    double w;   /* Direct form II state (the block's "Initial states") */
} tvc_accel_blend_t;

double tvc_accel_noise_schedule(double thrust_n);
double tvc_accel_blend_step(tvc_accel_blend_t *b, double schedule);
/* Set the state so the filter sits in steady state at this output. */
void tvc_accel_blend_settle(tvc_accel_blend_t *b, double output);

/* Thrust curve lookup, 0 outside [0, burn]. */
double tvc_thrust_at(double t_s);

#endif /* TVC_UKF_H */
