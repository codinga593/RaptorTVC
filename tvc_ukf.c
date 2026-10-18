/*
 * tvc_ukf.c - see tvc_ukf.h. Pure C99, no hardware dependencies, so the
 * same file is compiled into the firmware and into the MATLAB MEX used by
 * verify/run_verification.m.
 */
#include "tvc_ukf.h"
#include "tvc_config.h"

#include <math.h>
#include <string.h>

#define NSIG (2 * UKF_N)
#define MMAX 3

/* ---------------------------------------------------------------------
 * Unscented transform parameters (calcUTParameters.m)
 * ------------------------------------------------------------------- */
typedef struct {
    double c;          /* alpha^2 (n + kappa)                      */
    double wm0, wm1;   /* mean weights, divided by OOM              */
    double wc0, wc1;   /* covariance weights, divided by OOM        */
    double oom;
} ut_params_t;

static ut_params_t ut_params(void)
{
    ut_params_t u;
    const double a = TVC_UKF_ALPHA;
    u.c = a * a * ((double)UKF_N + TVC_UKF_KAPPA);
    double wm0 = 1.0 - (double)UKF_N / u.c;
    double wm1 = 1.0 / (2.0 * u.c);
    double wc0 = wm0 + (1.0 - a * a + TVC_UKF_BETA);
    double wc1 = wm1;
    if (wm0 != 0.0) {
        u.oom = wm0;
        wm0 /= u.oom; wm1 /= u.oom; wc0 /= u.oom; wc1 /= u.oom;
    } else {
        u.oom = 1.0;
    }
    u.wm0 = wm0; u.wm1 = wm1; u.wc0 = wc0; u.wc1 = wc1;
    return u;
}

/* ---------------------------------------------------------------------
 * Small dense linear algebra
 * ------------------------------------------------------------------- */

/* Lower Cholesky of an n x n matrix stored with row stride ld. */
static bool chol_lower(const double *A, double *L, int n, int ld)
{
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) L[i * ld + j] = 0.0;
    }
    for (int j = 0; j < n; j++) {
        double s = A[j * ld + j];
        for (int k = 0; k < j; k++) s -= L[j * ld + k] * L[j * ld + k];
        if (!(s > 0.0)) return false;
        double d = sqrt(s);
        L[j * ld + j] = d;
        for (int i = j + 1; i < n; i++) {
            double t = A[i * ld + j];
            for (int k = 0; k < j; k++) t -= L[i * ld + k] * L[j * ld + k];
            L[i * ld + j] = t / d;
        }
    }
    return true;
}

static void symmetrize(double P[UKF_N][UKF_N])
{
    for (int i = 0; i < UKF_N; i++) {
        for (int j = i + 1; j < UKF_N; j++) {
            double m = 0.5 * (P[i][j] + P[j][i]);
            P[i][j] = m;
            P[j][i] = m;
        }
    }
}

/* Sigma points X2(:,k) = x +/- sqrt(c) * chol(P)(:,k)  (calcSigmaPoints.m) */
static bool sigma_points(tvc_ukf_t *f, const ut_params_t *u, double X2[NSIG][UKF_N])
{
    double L[UKF_N][UKF_N];
    if (!chol_lower(&f->P[0][0], &L[0][0], UKF_N, UKF_N)) {
        /* Should never happen with this model; recover instead of
         * freezing the estimator mid-flight. */
        memcpy(f->P, f->P0, sizeof f->P);
        f->fault_count++;
        if (!chol_lower(&f->P[0][0], &L[0][0], UKF_N, UKF_N)) return false;
    }
    const double sc = sqrt(u->c);
    for (int k = 0; k < UKF_N; k++) {
        for (int i = 0; i < UKF_N; i++) {
            double d = sc * L[i][k];
            X2[k][i] = f->x[i] + d;
            X2[k + UKF_N][i] = f->x[i] - d;
        }
    }
    return true;
}

/* ---------------------------------------------------------------------
 * Model functions (PID/rocketUKF*.m)
 * ------------------------------------------------------------------- */

/* rocketUKFStateFcn.m */
static void state_fcn(const double x[UKF_N], const double omega[3], double dt, double out[UKF_N])
{
    double q[4] = { x[0], x[1], x[2], x[3] };
    double qn = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (qn > 1.0e-12) {
        q[0] /= qn; q[1] /= qn; q[2] /= qn; q[3] /= qn;
    } else {
        q[0] = 1.0; q[1] = 0.0; q[2] = 0.0; q[3] = 0.0;
    }

    double wn = sqrt(omega[0] * omega[0] + omega[1] * omega[1] + omega[2] * omega[2]);
    double half = 0.5 * wn * dt;
    double d[4];
    if (wn > 1.0e-12) {
        double vs = sin(half) / wn;
        d[0] = cos(half);
        d[1] = vs * omega[0];
        d[2] = vs * omega[1];
        d[3] = vs * omega[2];
    } else {
        d[0] = 1.0;
        d[1] = 0.5 * dt * omega[0];
        d[2] = 0.5 * dt * omega[1];
        d[3] = 0.5 * dt * omega[2];
    }

    double n0 = q[0] * d[0] - q[1] * d[1] - q[2] * d[2] - q[3] * d[3];
    double n1 = q[0] * d[1] + q[1] * d[0] + q[2] * d[3] - q[3] * d[2];
    double n2 = q[0] * d[2] - q[1] * d[3] + q[2] * d[0] + q[3] * d[1];
    double n3 = q[0] * d[3] + q[1] * d[2] - q[2] * d[1] + q[3] * d[0];
    double nn = sqrt(n0 * n0 + n1 * n1 + n2 * n2 + n3 * n3);
    out[0] = n0 / nn;
    out[1] = n1 / nn;
    out[2] = n2 / nn;
    out[3] = n3 / nn;

    for (int i = 0; i < 3; i++) {
        out[4 + i] = x[4 + i] + x[7 + i] * dt;
        out[7 + i] = x[7 + i];
    }
}

/* rocketUKFAccelMeasFcn.m */
static void meas_accel(const double x[UKF_N], double z[MMAX])
{
    double q[4] = { x[0], x[1], x[2], x[3] };
    double qn = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (qn > 1.0e-12) {
        q[0] /= qn; q[1] /= qn; q[2] /= qn; q[3] /= qn;
    } else {
        q[0] = 1.0; q[1] = 0.0; q[2] = 0.0; q[3] = 0.0;
    }
    z[0] = 2.0 * (q[1] * q[3] - q[0] * q[2]);
    z[1] = 2.0 * (q[2] * q[3] + q[0] * q[1]);
    z[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
}

/* rocketUKFGpsDownMeasFcn.m and rocketUKFBaroMeasFcn.m */
static void meas_down(const double x[UKF_N], double z[MMAX])
{
    z[0] = x[6];
}

/* ---------------------------------------------------------------------
 * Filter
 * ------------------------------------------------------------------- */

void tvc_ukf_init(tvc_ukf_t *f, const double q0[4])
{
    memset(f, 0, sizeof *f);
    f->x[0] = q0[0];
    f->x[1] = q0[1];
    f->x[2] = q0[2];
    f->x[3] = q0[3];

    f->P0[0][0] = TVC_UKF_P0_QW;
    for (int i = 1; i < 4; i++) f->P0[i][i] = TVC_UKF_P0_QXYZ;
    for (int i = 4; i < 7; i++) f->P0[i][i] = TVC_UKF_P0_POS;
    for (int i = 7; i < 10; i++) f->P0[i][i] = TVC_UKF_P0_VEL;
    memcpy(f->P, f->P0, sizeof f->P);

    for (int i = 0; i < 4; i++) f->Q[i][i] = TVC_UKF_Q_QUAT;
    for (int i = 0; i < 3; i++) {
        f->Q[4 + i][4 + i] = TVC_UKF_Q_POS;
        f->Q[4 + i][7 + i] = TVC_UKF_Q_POSVEL;
        f->Q[7 + i][4 + i] = TVC_UKF_Q_POSVEL;
        f->Q[7 + i][7 + i] = TVC_UKF_Q_VEL;
    }
}

/* UKFCorrectorAdditive + UKFCorrector.correctStateAndSqrtCovariance,
 * with R = r * I (all measurements here have isotropic noise). */
static bool correct(tvc_ukf_t *f, void (*h)(const double *, double *), int m,
                    const double *z, double r)
{
    const ut_params_t u = ut_params();
    double X2[NSIG][UKF_N];
    if (!sigma_points(f, &u, X2)) return false;

    double Y1[MMAX], Y2[NSIG][MMAX];
    h(f->x, Y1);
    for (int k = 0; k < NSIG; k++) h(X2[k], Y2[k]);

    /* UTMeanCov.m */
    double ym[MMAX];
    for (int i = 0; i < m; i++) {
        double s = Y1[i] * u.wm0;
        for (int k = 0; k < NSIG; k++) s += Y2[k][i] * u.wm1;
        ym[i] = s * u.oom;
    }
    double d1[MMAX];
    for (int i = 0; i < m; i++) {
        d1[i] = Y1[i] - ym[i];
        for (int k = 0; k < NSIG; k++) Y2[k][i] -= ym[i];
    }
    double Pyy[MMAX][MMAX];
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < m; j++) {
            double s2 = 0.0;
            for (int k = 0; k < NSIG; k++) s2 += Y2[k][i] * Y2[k][j];
            Pyy[i][j] = u.oom * (u.wc0 * d1[i] * d1[j] + u.wc1 * s2);
        }
        Pyy[i][i] += r;
    }
    double Pxy[UKF_N][MMAX];
    for (int i = 0; i < UKF_N; i++) {
        for (int j = 0; j < m; j++) {
            double s = 0.0;
            for (int k = 0; k < NSIG; k++) s += (X2[k][i] - f->x[i]) * Y2[k][j];
            Pxy[i][j] = s * (u.wc1 * u.oom);
        }
    }

    /* K = Pxy / Pyy via Cholesky of Pyy */
    double L[MMAX][MMAX];
    if (!chol_lower(&Pyy[0][0], &L[0][0], m, MMAX)) {
        f->fault_count++;
        return false;
    }
    double K[UKF_N][MMAX];
    for (int i = 0; i < UKF_N; i++) {
        /* solve L * L' * k' = pxy' for row i */
        double t[MMAX];
        for (int j = 0; j < m; j++) {
            double s = Pxy[i][j];
            for (int l = 0; l < j; l++) s -= L[j][l] * t[l];
            t[j] = s / L[j][j];
        }
        for (int j = m - 1; j >= 0; j--) {
            double s = t[j];
            for (int l = j + 1; l < m; l++) s -= L[l][j] * K[i][l];
            K[i][j] = s / L[j][j];
        }
    }

    double res[MMAX];
    for (int j = 0; j < m; j++) res[j] = z[j] - ym[j];
    for (int i = 0; i < UKF_N; i++) {
        double s = 0.0;
        for (int j = 0; j < m; j++) s += K[i][j] * res[j];
        f->x[i] += s;
    }

    /* P -= K * Pyy * K'  (== P - Pxy * K') */
    for (int i = 0; i < UKF_N; i++) {
        for (int j = 0; j < UKF_N; j++) {
            double s = 0.0;
            for (int l = 0; l < m; l++) s += Pxy[i][l] * K[j][l];
            f->P[i][j] -= s;
        }
    }
    symmetrize(f->P);
    return true;
}

bool tvc_ukf_correct_accel(tvc_ukf_t *f, const double z[3], double r)
{
    return correct(f, meas_accel, 3, z, r);
}

bool tvc_ukf_correct_down(tvc_ukf_t *f, double z, double r)
{
    return correct(f, meas_down, 1, &z, r);
}

/* UKFPredictorAdditive.predict */
bool tvc_ukf_predict(tvc_ukf_t *f, const double omega[3], double dt)
{
    const ut_params_t u = ut_params();
    double X2[NSIG][UKF_N];
    if (!sigma_points(f, &u, X2)) return false;

    double Y1[UKF_N], Y2[NSIG][UKF_N];
    state_fcn(f->x, omega, dt, Y1);
    for (int k = 0; k < NSIG; k++) state_fcn(X2[k], omega, dt, Y2[k]);

    double ym[UKF_N];
    for (int i = 0; i < UKF_N; i++) {
        double s = Y1[i] * u.wm0;
        for (int k = 0; k < NSIG; k++) s += Y2[k][i] * u.wm1;
        ym[i] = s * u.oom;
    }
    double d1[UKF_N];
    for (int i = 0; i < UKF_N; i++) {
        d1[i] = Y1[i] - ym[i];
        for (int k = 0; k < NSIG; k++) Y2[k][i] -= ym[i];
    }
    for (int i = 0; i < UKF_N; i++) {
        for (int j = i; j < UKF_N; j++) {
            double s2 = 0.0;
            for (int k = 0; k < NSIG; k++) s2 += Y2[k][i] * Y2[k][j];
            double p = u.oom * (u.wc0 * d1[i] * d1[j] + u.wc1 * s2) + f->Q[i][j];
            f->P[i][j] = p;
            f->P[j][i] = p;
        }
    }
    memcpy(f->x, ym, sizeof ym);
    return true;
}

/* ---------------------------------------------------------------------
 * Thrust-dependent accelerometer noise
 * ------------------------------------------------------------------- */

static double interp1(const double *xs, const double *ys, int n, double x)
{
    if (x <= xs[0]) return ys[0];
    if (x >= xs[n - 1]) return ys[n - 1];
    int i = 1;
    while (x > xs[i]) i++;
    double t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
    return ys[i - 1] + t * (ys[i] - ys[i - 1]);
}

double tvc_accel_noise_schedule(double thrust_n)
{
    static const double bp[TVC_ACCEL_SCHED_N] = TVC_ACCEL_SCHED_THRUST_N;
    static const double sc[TVC_ACCEL_SCHED_N] = TVC_ACCEL_SCHED_SCALE;
    return interp1(bp, sc, TVC_ACCEL_SCHED_N, fabs(thrust_n));
}

/* Discrete Filter, Direct form II: num [b0], den [1 -a]
 *   w[k] = u[k] + a w[k-1],  y[k] = b0 w[k]                         */
#define BLEND_B0 (TVC_UKF_TS_S / (TVC_ACCEL_BLEND_TAU_S + TVC_UKF_TS_S))
#define BLEND_A  (TVC_ACCEL_BLEND_TAU_S / (TVC_ACCEL_BLEND_TAU_S + TVC_UKF_TS_S))

double tvc_accel_blend_step(tvc_accel_blend_t *b, double schedule)
{
    b->w = schedule + BLEND_A * b->w;
    return BLEND_B0 * b->w;
}

void tvc_accel_blend_settle(tvc_accel_blend_t *b, double output)
{
    b->w = output / BLEND_B0;
}

double tvc_thrust_at(double t_s)
{
    static const double tt[TVC_THRUST_N_POINTS] = TVC_THRUST_TIME_S;
    static const double ff[TVC_THRUST_N_POINTS] = TVC_THRUST_FORCE_N;
    if (t_s < 0.0 || t_s > tt[TVC_THRUST_N_POINTS - 1]) return 0.0;
    return interp1(tt, ff, TVC_THRUST_N_POINTS, t_s);
}
