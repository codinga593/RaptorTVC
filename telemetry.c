/*
 * telemetry.c - see telemetry.h.
 */
#include "telemetry.h"
#include "app.h"
#include "flight.h"
#include "serial.h"
#include "stm32h7xx_hal.h"
#include "sx1262.h"
#include "tvc_config.h"

#include <stdio.h>
#include <string.h>

typedef struct __attribute__((packed)) {
    uint8_t magic;        /* 0x54 'T' */
    uint8_t seq;
    uint8_t state;
    uint8_t gps_fix_sv;   /* fix type << 6 | min(sv, 63) */
    uint32_t t_ms;
    int16_t tilt_cdeg;    /* 0.01 deg */
    int16_t err_cdeg[2];
    int16_t cmd_cdeg[2];
    int16_t vel_d_cm;     /* cm/s */
    int32_t alt_cm;       /* baro altitude above pad */
    int32_t lat_e7, lon_e7;
    int32_t h_msl_cm;
    uint16_t vbat_mv;
} lora_packet_t;
_Static_assert(sizeof(lora_packet_t) == 38, "LoRa packet size");

static bool lora_ok;

bool telemetry_init(void)
{
    serial_init(SERIAL_PI, TVC_PI_BAUD);
    lora_ok = sx1262_init(TVC_LORA_FREQ_HZ, TVC_LORA_TX_DBM, TVC_LORA_SF);
    return lora_ok;
}

static int16_t c16(float v)
{
    v *= 100.0f;
    if (v > 32767.0f) v = 32767.0f;
    if (v < -32768.0f) v = -32768.0f;
    return (int16_t)v;
}

static void send_pi(const flight_status_t *s, const app_sensors_t *a)
{
    char b[320];
    int n = snprintf(b, sizeof b,
                     "$TVC,%lu,%s,%.3f,%.5f,%.5f,%.5f,%.5f,%.2f,%.2f,%.2f,%.2f,%.2f,"
                     "%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%u,%u,%.7f,%.7f,%.1f,%.2f\r\n",
                     (unsigned long)s->t_ms, flight_state_name(s->state), (double)s->t_flight_s,
                     (double)s->q[0], (double)s->q[1], (double)s->q[2], (double)s->q[3],
                     (double)s->tilt_deg, (double)s->err[0], (double)s->err[1],
                     (double)s->cmd[0], (double)s->cmd[1],
                     (double)s->gyro[0], (double)s->gyro[1], (double)s->gyro[2],
                     (double)s->pos_d, (double)s->vel_d, (double)a->baro_alt_m,
                     a->gps_valid ? a->gps.fix_type : 0, a->gps_valid ? a->gps.num_sv : 0,
                     a->gps_valid ? a->gps.lat_deg : 0.0, a->gps_valid ? a->gps.lon_deg : 0.0,
                     a->gps_valid ? (double)a->gps.h_msl_m : 0.0, (double)a->vbat_v);
    if (n > 0) serial_write(SERIAL_PI, b, (size_t)(n < (int)sizeof b ? n : (int)sizeof b - 1));
}

static void send_lora(const flight_status_t *s, const app_sensors_t *a)
{
    static uint8_t seq;
    lora_packet_t p;
    memset(&p, 0, sizeof p);
    p.magic = 0x54;
    p.seq = seq++;
    p.state = (uint8_t)s->state;
    if (a->gps_valid) {
        uint8_t sv = a->gps.num_sv > 63 ? 63 : a->gps.num_sv;
        p.gps_fix_sv = (uint8_t)(((a->gps.fix_type & 3u) << 6) | sv);
        p.lat_e7 = (int32_t)(a->gps.lat_deg * 1e7);
        p.lon_e7 = (int32_t)(a->gps.lon_deg * 1e7);
        p.h_msl_cm = (int32_t)(a->gps.h_msl_m * 100.0f);
    }
    p.t_ms = s->t_ms;
    p.tilt_cdeg = c16(s->tilt_deg);
    p.err_cdeg[0] = c16(s->err[0]);
    p.err_cdeg[1] = c16(s->err[1]);
    p.cmd_cdeg[0] = c16(s->cmd[0]);
    p.cmd_cdeg[1] = c16(s->cmd[1]);
    p.vel_d_cm = c16(s->vel_d);
    p.alt_cm = (int32_t)(a->baro_alt_m * 100.0f);
    p.vbat_mv = (uint16_t)(a->vbat_v * 1000.0f);
    sx1262_send((const uint8_t *)&p, sizeof p);
}

void telemetry_task(void)
{
    static uint32_t last_pi, last_lora;
    uint32_t now = HAL_GetTick();
    flight_status_t s;
    const app_sensors_t *a = app_sensors();

    if (now - last_pi >= TVC_PI_PERIOD_MS) {
        last_pi = now;
        flight_get_status(&s);
        send_pi(&s, a);
    }
    if (lora_ok) {
        sx1262_tx_busy();   /* service TxDone */
        if (now - last_lora >= TVC_LORA_PERIOD_MS) {
            last_lora = now;
            flight_get_status(&s);
            send_lora(&s, a);
        }
    }
}
