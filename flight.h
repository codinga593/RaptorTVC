/*
 * flight.h - flight state machine and the 1 kHz control loop.
 *
 * flight_imu_isr() runs in the IMU data-ready interrupt (priority 1):
 *   every sample : IMU read, sensor->body rotation, gyro bias removal,
 *                  launch / burnout detection, PID + servo output
 *   every 10th   : UKF step (100 Hz, TVC_UKF_TS_S), log record, status
 * Everything else (USB, GPS, baro, flash, LoRa, Pi) runs in the main loop
 * and talks to the ISR through the mailboxes below.
 */
#ifndef FLIGHT_H
#define FLIGHT_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    FS_IDLE = 0,   /* disarmed, gimbal centred (or bench mode)         */
    FS_CAL,        /* averaging gyro bias / pad attitude                */
    FS_ARMED,      /* on the pad, waiting for launch acceleration       */
    FS_POWERED,    /* motor burning: TVC active                         */
    FS_COAST,      /* burnout: gimbal centred, still logging            */
    FS_LANDED,
    FS_COUNT
} flight_state_t;

const char *flight_state_name(flight_state_t s);

/* ---- Log record (80 bytes, little endian) -------------------------- */
#define LOG_MAGIC       0xA55Au
#define LOG_TYPE_HEADER 1u
#define LOG_TYPE_DATA   2u

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t type;
    uint8_t state;
    uint32_t t_ms;        /* ms since boot */
    float q[4];           /* attitude fed to the controller (NED->body)  */
    float gyro[3];        /* body rates, rad/s, bias removed             */
    float accel[3];       /* body specific force, m/s^2                  */
    float cmd[2];         /* gimbal command pitch, yaw (deg)             */
    float err[2];         /* attitude error pitch, yaw (deg)             */
    float pos_d;          /* UKF NED down position (m)                   */
    float vel_d;          /* UKF NED down velocity (m/s)                 */
    float baro_alt;       /* barometric altitude above pad (m)           */
    uint16_t vbat_mv;
    uint16_t crc;         /* CRC-16/CCITT of the preceding 78 bytes      */
} log_record_t;
/* Header record (type 1) reuses the layout:
 *   t_ms = launch time, q = attitude at launch, gyro = gyro bias,
 *   accel = mean pad accel, cmd = {max gimbal, burn time},
 *   err = {pad tilt deg, 0}, pos_d/vel_d/baro_alt = 0 */

_Static_assert(sizeof(log_record_t) == 80, "log record must be 80 bytes");

uint16_t log_crc16(const void *data, uint32_t n);

/* ---- Status snapshot for console / telemetry ----------------------- */
typedef struct {
    flight_state_t state;
    bool bench;
    bool imu_ok;
    bool cal_ok;
    uint32_t t_ms;
    float t_flight_s;     /* since launch, 0 before */
    float q[4];
    float tilt_deg;       /* angle of the nose from vertical */
    float gyro[3];
    float accel[3];
    float cmd[2];
    float err[2];
    float pos_d, vel_d;
    float gyro_bias[3];
    float imu_temp_c;
    uint32_t imu_samples;
    uint32_t imu_errors;
    uint32_t est_faults;
    uint32_t overruns;    /* control ISR took longer than one period */
    uint32_t max_isr_us;
    char last_msg[48];    /* result of the last ARM / CAL attempt */
} flight_status_t;

void flight_get_status(flight_status_t *out);

/* ---- Main loop -> ISR inputs --------------------------------------- */
void flight_set_baro(float alt_above_pad_m);     /* only after ARM */
void flight_set_gps(float alt_above_pad_m);      /* only after ARM, 3D fix */
void flight_set_vbat(float v);

typedef enum {
    FLIGHT_CMD_ARM,
    FLIGHT_CMD_DISARM,
    FLIGHT_CMD_CAL,
    FLIGHT_CMD_BENCH_ON,
    FLIGHT_CMD_BENCH_OFF,
} flight_cmd_t;
void flight_command(flight_cmd_t c);

/* Bench servo override, honoured only in IDLE with bench mode off:
 *   SERVO_MANUAL_OFF   normal (centred)
 *   SERVO_MANUAL_DEG   a, b = pitch, yaw gimbal angle in deg (calibrated)
 *   SERVO_MANUAL_US    a, b = raw pitch, yaw servo pulse in us          */
typedef enum { SERVO_MANUAL_OFF = 0, SERVO_MANUAL_DEG, SERVO_MANUAL_US } servo_manual_t;
void flight_servo_manual(servo_manual_t mode, float a, float b);

/* Records produced by the ISR at 100 Hz, consumed by the logger. */
bool flight_pop_record(log_record_t *r);

/* Startup: call after imu_init(); begins the boot-time calibration. */
void flight_init(bool imu_ok);
/* IMU data-ready interrupt body. */
void flight_imu_isr(void);
/* Main-loop supervision (IMU dropout -> centre the gimbal). */
void flight_supervise(void);

#endif /* FLIGHT_H */
