/*
 * flight.c - see flight.h.
 */
#include "flight.h"
#include "hw.h"
#include "imu.h"
#include "tvc_config.h"
#include "tvc_control.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

#define G0          ((float)TVC_GRAVITY_MPS2)
#define RAD2DEG_F   57.29577951308232f
#define CTRL_HZ     ((uint32_t)(1.0 / TVC_CTRL_DT_S + 0.5))
#define SECS(s)     ((uint32_t)((s) * CTRL_HZ + 0.5))     /* seconds -> samples */

static const float R_body_imu[9] = TVC_R_BODY_IMU;

/* ---------------------------------------------------------------------
 * Shared with the main loop
 * ------------------------------------------------------------------- */
typedef struct {
    volatile uint32_t seq;
    float value;
} mailbox_t;

static mailbox_t mb_baro, mb_gps;
static volatile float vbat_v;
static volatile uint32_t cmd_bits;

static volatile struct {
    servo_manual_t mode;
    float a, b;
} manual;

static volatile uint32_t status_seq;
static flight_status_t status;

#define RECQ_LEN 64u
static log_record_t recq[RECQ_LEN];
static volatile uint32_t recq_head, recq_tail;

/* ---------------------------------------------------------------------
 * ISR-owned state
 * ------------------------------------------------------------------- */
static flight_state_t state = FS_IDLE;
static bool bench;
static bool imu_ok;
static bool est_ready;
static tvc_est_t est;
static tvc_ctrl_t ctrl;

static uint32_t tick;                 /* IMU samples since boot */
static uint32_t state_ticks;          /* samples since entering the state */
static uint32_t t_launch_us;
static float t_flight_s;
static uint32_t imu_errors, overruns, max_isr_us, last_isr_start;

static float gyro_bias[3];
static float imu_temp_c;

/* 100 Hz accumulators (UKF input) */
static float acc_gyro[3], acc_accel[3];
static unsigned acc_n;
static double held_gyro[3];           /* last UKF-rate gyro, used for rate damping */
static float held_accel[3];

/* calibration */
static bool cal_arm_after;
static bool cal_ok;
static double cal_gyro[3], cal_accel[3];
static float cal_max_rate_dps;
static float pad_accel[3], pad_tilt_deg;
typedef enum {
    MSG_NONE, MSG_BOOT_CAL, MSG_BOOT_NO_IMU, MSG_CAL_MOVING, MSG_CAL_ACCEL,
    MSG_ARM_TILT, MSG_CAL_OK, MSG_ARM_OK, MSG_DISARMED,
} msg_code_t;
static volatile msg_code_t msg_code;
static volatile float msg_value;

/* event detectors */
static uint32_t launch_count, burnout_count, still_count;

/* output */
static float out_cmd[2];
static uint16_t vbat_mv;

/* ---------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------- */
const char *flight_state_name(flight_state_t s)
{
    static const char *const n[FS_COUNT] = { "IDLE", "CAL", "ARMED", "POWERED", "COAST", "LANDED" };
    return (s < FS_COUNT) ? n[s] : "?";
}

uint16_t log_crc16(const void *data, uint32_t n)
{
    const uint8_t *p = data;
    uint16_t crc = 0xFFFF;
    while (n--) {
        crc ^= (uint16_t)(*p++) << 8;
        for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

static void rotate(const float R[9], const float v[3], float out[3])
{
    out[0] = R[0] * v[0] + R[1] * v[1] + R[2] * v[2];
    out[1] = R[3] * v[0] + R[4] * v[1] + R[5] * v[2];
    out[2] = R[6] * v[0] + R[7] * v[1] + R[8] * v[2];
}

/* The ISR only records a code and a value; the text is formatted in
 * flight_get_status() (main loop), because newlib's printf is not
 * re-entrant and must not run in an interrupt. */
static void set_msg(msg_code_t code, float value)
{
    msg_value = value;
    msg_code = code;
}

static void enter(flight_state_t s)
{
    state = s;
    state_ticks = 0;
}

static bool mailbox_read(mailbox_t *m, uint32_t *last_seq, float *v)
{
    uint32_t s = m->seq;
    if ((s & 1u) || s == *last_seq) return false;   /* being written, or nothing new */
    float val = m->value;
    __DMB();
    if (m->seq != s) return false;
    *last_seq = s;
    *v = val;
    return true;
}

static void mailbox_write(mailbox_t *m, float v)
{
    m->seq++;
    __DMB();
    m->value = v;
    __DMB();
    m->seq++;
}

void flight_set_baro(float alt) { mailbox_write(&mb_baro, alt); }
void flight_set_gps(float alt) { mailbox_write(&mb_gps, alt); }
void flight_set_vbat(float v) { vbat_v = v; }

void flight_servo_manual(servo_manual_t mode, float a, float b)
{
    __disable_irq();
    manual.mode = mode;
    manual.a = a;
    manual.b = b;
    __enable_irq();
}

void flight_command(flight_cmd_t c)
{
    __disable_irq();
    cmd_bits |= 1u << c;
    __enable_irq();
}

static uint32_t take_commands(void)
{
    uint32_t c = cmd_bits;     /* ISR context: main cannot run concurrently */
    cmd_bits = 0;
    return c;
}

bool flight_pop_record(log_record_t *r)
{
    uint32_t t = recq_tail;
    if (t == recq_head) return false;
    *r = recq[t % RECQ_LEN];
    __DMB();
    recq_tail = t + 1u;
    return true;
}

static void push_record(const log_record_t *r)
{
    uint32_t h = recq_head;
    if (h - recq_tail >= RECQ_LEN) return;     /* logger behind: drop */
    recq[h % RECQ_LEN] = *r;
    __DMB();
    recq_head = h + 1u;
}

/* ---------------------------------------------------------------------
 * Estimator / controller glue
 * ------------------------------------------------------------------- */
static void est_reset_from_gravity(const float accel_body[3])
{
    double an = sqrt((double)accel_body[0] * accel_body[0] + (double)accel_body[1] * accel_body[1] +
                     (double)accel_body[2] * accel_body[2]);
    double q0[4] = { TVC_QDES_W, TVC_QDES_X, TVC_QDES_Y, TVC_QDES_Z };
    if (an > 1e-3) {
        double down[3] = { -accel_body[0] / an, -accel_body[1] / an, -accel_body[2] / an };
        if (!tvc_quat_from_gravity(down, q0)) {
            q0[0] = TVC_QDES_W; q0[1] = TVC_QDES_X; q0[2] = TVC_QDES_Y; q0[3] = TVC_QDES_Z;
        }
    }
    tvc_est_init(&est, q0, 0.0);
    tvc_accel_blend_settle(&est.blend, tvc_accel_noise_schedule(0.0));   /* pad: trust accel */
    est_ready = true;
}

static void ctrl_reset_bumpless(void)
{
    double ep, ey;
    tvc_attitude_error(est.q_out, &ep, &ey);
    tvc_ctrl_reset(&ctrl, ep, ey);
}

static void slew_to_zero(void)
{
    const float step = (float)(TVC_GIMBAL_SLEW_DEG_S * TVC_CTRL_DT_S);
    for (int i = 0; i < 2; i++) {
        float c = out_cmd[i];
        out_cmd[i] = (c > step) ? c - step : (c < -step) ? c + step : 0.0f;
    }
}

static void ukf_step(void)
{
    static uint32_t baro_seq, gps_seq;
    tvc_est_input_t in;
    for (int i = 0; i < 3; i++) {
        in.gyro[i] = held_gyro[i];
        in.accel[i] = held_accel[i];
    }
    in.thrust_n = (state == FS_POWERED) ? tvc_thrust_at(t_flight_s) : 0.0;

    float v = 0.0f;
    bool use_ref = (state == FS_ARMED || state == FS_POWERED || state == FS_COAST || state == FS_LANDED);
    in.has_baro = mailbox_read(&mb_baro, &baro_seq, &v) && use_ref;
    in.baro_down_m = -(double)v;
    in.has_gps = mailbox_read(&mb_gps, &gps_seq, &v) && use_ref;
    in.gps_down_m = -(double)v;

    if (!tvc_est_step(&est, &in)) {
        /* fault_count is incremented inside; nothing else to do */
    }
}

/* ---------------------------------------------------------------------
 * Calibration
 * ------------------------------------------------------------------- */
static void cal_start(bool arm_after)
{
    cal_arm_after = arm_after;
    if (arm_after) t_flight_s = 0.0f;
    memset(cal_gyro, 0, sizeof cal_gyro);
    memset(cal_accel, 0, sizeof cal_accel);
    cal_max_rate_dps = 0.0f;
    enter(FS_CAL);
}

static void cal_sample(const float g_raw[3], const float a[3])
{
    for (int i = 0; i < 3; i++) {
        cal_gyro[i] += g_raw[i];
        cal_accel[i] += a[i];
    }
    float r = sqrtf(g_raw[0] * g_raw[0] + g_raw[1] * g_raw[1] + g_raw[2] * g_raw[2]) * RAD2DEG_F;
    if (r > cal_max_rate_dps) cal_max_rate_dps = r;
}

static void cal_finish(void)
{
    const double n = (double)state_ticks;
    float a[3];
    for (int i = 0; i < 3; i++) a[i] = (float)(cal_accel[i] / n);
    float an = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    float tilt = (an > 1e-3f) ? acosf(fmaxf(-1.0f, fminf(1.0f, a[0] / an))) * RAD2DEG_F : 180.0f;

    bool ok = true;
    if (cal_max_rate_dps > TVC_CAL_MAX_GYRO_DPS) {
        set_msg(MSG_CAL_MOVING, cal_max_rate_dps);
        ok = false;
    } else if (fabsf(an / G0 - 1.0f) > TVC_CAL_ACCEL_TOL_G) {
        set_msg(MSG_CAL_ACCEL, an / G0);
        ok = false;
    } else if (cal_arm_after && tilt > TVC_MAX_PAD_TILT_DEG) {
        set_msg(MSG_ARM_TILT, tilt);
        ok = false;
    } else {
        set_msg(cal_arm_after ? MSG_ARM_OK : MSG_CAL_OK, tilt);
    }

    if (ok) {
        for (int i = 0; i < 3; i++) {
            gyro_bias[i] = (float)(cal_gyro[i] / n);
            pad_accel[i] = a[i];
        }
        pad_tilt_deg = tilt;
        cal_ok = true;
        est_reset_from_gravity(a);
    }
    if (ok && cal_arm_after) {
        launch_count = 0;
        enter(FS_ARMED);
    } else {
        enter(FS_IDLE);
    }
}

/* ---------------------------------------------------------------------
 * Records and status (100 Hz)
 * ------------------------------------------------------------------- */
static void fill_record(log_record_t *r, uint8_t type)
{
    memset(r, 0, sizeof *r);
    r->magic = LOG_MAGIC;
    r->type = type;
    r->state = (uint8_t)state;
    r->t_ms = HAL_GetTick();
    for (int i = 0; i < 4; i++) r->q[i] = (float)est.q_out[i];
    if (type == LOG_TYPE_HEADER) {
        for (int i = 0; i < 3; i++) {
            r->gyro[i] = gyro_bias[i];
            r->accel[i] = pad_accel[i];
        }
        r->cmd[0] = (float)TVC_MAX_GIMBAL_DEG;
        r->cmd[1] = (float)TVC_BURN_TIME_S;
        r->err[0] = pad_tilt_deg;
    } else {
        for (int i = 0; i < 3; i++) {
            r->gyro[i] = (float)held_gyro[i];
            r->accel[i] = held_accel[i];
        }
        r->cmd[0] = out_cmd[0];
        r->cmd[1] = out_cmd[1];
        r->err[0] = (float)ctrl.err_pitch_deg;
        r->err[1] = (float)ctrl.err_yaw_deg;
        r->pos_d = (float)est.ukf.x[6];
        r->vel_d = (float)est.ukf.x[9];
        r->baro_alt = mb_baro.value;
    }
    r->vbat_mv = vbat_mv;
    r->crc = log_crc16(r, sizeof *r - 2);
}

static void publish_status(void)
{
    status_seq++;
    __DMB();
    status.state = state;
    status.bench = bench;
    status.imu_ok = imu_ok;
    status.cal_ok = cal_ok;
    status.t_ms = HAL_GetTick();
    status.t_flight_s = t_flight_s;
    for (int i = 0; i < 4; i++) status.q[i] = (float)est.q_out[i];
    status.tilt_deg = (float)tvc_tilt_deg(est.q_out);
    for (int i = 0; i < 3; i++) {
        status.gyro[i] = (float)held_gyro[i];
        status.accel[i] = held_accel[i];
        status.gyro_bias[i] = gyro_bias[i];
    }
    status.cmd[0] = out_cmd[0];
    status.cmd[1] = out_cmd[1];
    {
        double ep, ey;
        tvc_attitude_error(est.q_out, &ep, &ey);
        status.err[0] = (float)ep;
        status.err[1] = (float)ey;
    }
    status.pos_d = (float)est.ukf.x[6];
    status.vel_d = (float)est.ukf.x[9];
    status.imu_temp_c = imu_temp_c;
    status.imu_samples = tick;
    status.imu_errors = imu_errors;
    status.est_faults = est.ukf.fault_count;
    status.overruns = overruns;
    status.max_isr_us = max_isr_us;
    __DMB();
    status_seq++;
}

void flight_get_status(flight_status_t *out)
{
    uint32_t s;
    do {
        s = status_seq;
        __DMB();
        *out = status;
        __DMB();
    } while ((s & 1u) || s != status_seq);

    float v = msg_value;
    char *m = out->last_msg;
    const size_t n = sizeof out->last_msg;
    switch (msg_code) {
    case MSG_BOOT_CAL:    snprintf(m, n, "boot: calibrating"); break;
    case MSG_BOOT_NO_IMU: snprintf(m, n, "boot: IMU NOT FOUND"); break;
    case MSG_CAL_MOVING:  snprintf(m, n, "CAL FAIL: moving (%.1f dps)", (double)v); break;
    case MSG_CAL_ACCEL:   snprintf(m, n, "CAL FAIL: |a| = %.2f g", (double)v); break;
    case MSG_ARM_TILT:    snprintf(m, n, "ARM FAIL: tilt %.1f deg", (double)v); break;
    case MSG_CAL_OK:      snprintf(m, n, "CAL OK: tilt %.1f deg", (double)v); break;
    case MSG_ARM_OK:      snprintf(m, n, "ARM OK: tilt %.1f deg", (double)v); break;
    case MSG_DISARMED:    snprintf(m, n, "disarmed"); break;
    default:              m[0] = '\0'; break;
    }
}

/* ---------------------------------------------------------------------
 * Init and supervision
 * ------------------------------------------------------------------- */
void flight_init(bool ok)
{
    imu_ok = ok;
    set_msg(ok ? MSG_BOOT_CAL : MSG_BOOT_NO_IMU, 0.0f);
    servo_enable(TVC_SERVO_CH_PITCH, true);
    servo_enable(TVC_SERVO_CH_YAW, true);
    servo_set_gimbal(0.0f, 0.0f);
    publish_status();
    /* The IMU interrupt is already running: hand the boot calibration to
     * it as a command rather than touching its state from here. */
    if (ok) flight_command(FLIGHT_CMD_CAL);
}

void flight_supervise(void)
{
    /* If the IMU stops interrupting, nothing else will move the servos:
     * centre them from here. */
    static uint32_t last_tick, last_change_ms;
    uint32_t now = HAL_GetTick();
    if (tick != last_tick) {
        last_tick = tick;
        last_change_ms = now;
    } else if (now - last_change_ms > 20u) {
        imu_ok = false;
        servo_set_gimbal(0.0f, 0.0f);
    }
    vbat_mv = (uint16_t)(vbat_v * 1000.0f);
}

/* ---------------------------------------------------------------------
 * 1 kHz control loop
 * ------------------------------------------------------------------- */
void flight_imu_isr(void)
{
    uint32_t t0 = hw_micros();
    if (tick && (t0 - last_isr_start) > 2000u) overruns++;
    last_isr_start = t0;

    imu_sample_t s;
    if (!imu_read(&s)) {
        imu_errors++;
        return;
    }
    imu_ok = true;
    tick++;
    state_ticks++;
    imu_temp_c = s.temp_c;

    float g_raw[3], g[3], a[3];
    rotate(R_body_imu, s.gyro, g_raw);
    rotate(R_body_imu, s.accel, a);
    for (int i = 0; i < 3; i++) g[i] = g_raw[i] - gyro_bias[i];

    if (!est_ready) est_reset_from_gravity(a);

    /* ---- commands from the main loop ---- */
    uint32_t c = take_commands();
    if (c & (1u << FLIGHT_CMD_DISARM)) {
        if (state == FS_ARMED || state == FS_CAL || state == FS_LANDED) {
            enter(FS_IDLE);
            set_msg(MSG_DISARMED, 0.0f);
        }
        bench = false;
    }
    if (state == FS_IDLE) {
        if (c & (1u << FLIGHT_CMD_ARM)) {
            bench = false;
            cal_start(true);
        } else if (c & (1u << FLIGHT_CMD_CAL)) {
            cal_start(false);
        } else if (c & (1u << FLIGHT_CMD_BENCH_ON)) {
            bench = true;
            ctrl_reset_bumpless();
        }
    }
    if (c & (1u << FLIGHT_CMD_BENCH_OFF)) bench = false;

    /* ---- per-sample state logic ---- */
    switch (state) {
    case FS_CAL:
        cal_sample(g_raw, a);
        if (state_ticks >= SECS(TVC_CAL_TIME_S)) cal_finish();
        break;

    case FS_ARMED:
        launch_count = (a[0] > (float)(TVC_LAUNCH_ACCEL_G * TVC_GRAVITY_MPS2)) ? launch_count + 1 : 0;
        if (launch_count >= SECS(TVC_LAUNCH_HOLD_S)) {
            /* Ignition happened when the acceleration first crossed the
             * threshold; time the thrust curve from there. */
            t_launch_us = t0 - (uint32_t)(TVC_LAUNCH_HOLD_S * 1e6);
            est.blend.w = TVC_ACCEL_BLEND_INIT;      /* model state at ignition */
            ctrl_reset_bumpless();
            burnout_count = 0;
            enter(FS_POWERED);
            log_record_t h;
            fill_record(&h, LOG_TYPE_HEADER);
            h.t_ms = HAL_GetTick() - (uint32_t)(TVC_LAUNCH_HOLD_S * 1000.0);
            h.crc = log_crc16(&h, sizeof h - 2);
            push_record(&h);
        }
        break;

    case FS_POWERED:
        t_flight_s = (float)((uint32_t)(t0 - t_launch_us)) * 1e-6f;
        burnout_count = (t_flight_s > TVC_BURNOUT_MIN_T_S &&
                         a[0] < (float)(TVC_BURNOUT_ACCEL_G * TVC_GRAVITY_MPS2)) ? burnout_count + 1 : 0;
        if (t_flight_s >= TVC_BURN_TIME_S || burnout_count >= SECS(TVC_BURNOUT_HOLD_S)) {
            enter(FS_COAST);
            still_count = 0;
        }
        break;

    case FS_COAST:
    case FS_LANDED: {
        t_flight_s = (float)((uint32_t)(t0 - t_launch_us)) * 1e-6f;
        float an = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        float rate = sqrtf(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]) * RAD2DEG_F;
        bool still = fabsf(an - G0) < 0.2f * G0 && rate < 10.0f;
        still_count = still ? still_count + 1 : 0;
        if (state == FS_COAST && t_flight_s > TVC_LANDED_MIN_T_S && still_count >= SECS(TVC_LANDED_STILL_S)) {
            enter(FS_LANDED);
        }
        break;
    }

    case FS_IDLE:
    default:
        break;
    }

    /* ---- 100 Hz estimator ---- */
    for (int i = 0; i < 3; i++) {
        acc_gyro[i] += g[i];
        acc_accel[i] += a[i];
    }
    if (++acc_n >= TVC_UKF_DECIMATION) {
        for (int i = 0; i < 3; i++) {
            held_gyro[i] = (double)(acc_gyro[i] / (float)acc_n);
            held_accel[i] = acc_accel[i] / (float)acc_n;
            acc_gyro[i] = 0.0f;
            acc_accel[i] = 0.0f;
        }
        acc_n = 0;
        if (state != FS_CAL) ukf_step();

        if (state == FS_ARMED || state == FS_POWERED || state == FS_COAST || state == FS_LANDED) {
            log_record_t r;
            fill_record(&r, LOG_TYPE_DATA);
            push_record(&r);
        }
        publish_status();
    }

    /* ---- control ---- */
    if (state == FS_POWERED || (state == FS_IDLE && bench)) {
        tvc_ctrl_step(&ctrl, est.q_out, held_gyro[1], held_gyro[2], TVC_CTRL_DT_S);
        out_cmd[0] = (float)ctrl.out_pitch_deg;
        out_cmd[1] = (float)ctrl.out_yaw_deg;
    } else {
        slew_to_zero();
    }

    if (state == FS_IDLE && !bench && manual.mode != SERVO_MANUAL_OFF) {
        if (manual.mode == SERVO_MANUAL_DEG) {
            servo_set_gimbal(manual.a, manual.b);
        } else {
            servo_write_us(TVC_SERVO_CH_PITCH, manual.a);
            servo_write_us(TVC_SERVO_CH_YAW, manual.b);
        }
    } else {
        if (state != FS_IDLE) manual.mode = SERVO_MANUAL_OFF;
        servo_set_gimbal(out_cmd[0], out_cmd[1]);
    }

    uint32_t dt = hw_micros() - t0;
    if (dt > max_isr_us) max_isr_us = dt;
}
