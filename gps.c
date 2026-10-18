/*
 * gps.c - UBX protocol: CFG-VALSET at boot, NAV-PVT parser.
 */
#include "gps.h"
#include "hw.h"
#include "serial.h"

#include <string.h>

#define GPS_BAUD          38400u      /* MAX-M10S factory default */

#define UBX_SYNC1         0xB5
#define UBX_SYNC2         0x62
#define UBX_CLASS_NAV     0x01
#define UBX_ID_NAV_PVT    0x07
#define UBX_CLASS_CFG     0x06
#define UBX_ID_CFG_VALSET 0x8A

/* Configuration keys (u-blox M10 interface description) */
#define CFG_UART1OUTPROT_NMEA        0x10740002u  /* L  */
#define CFG_UART1OUTPROT_UBX         0x10740001u  /* L  */
#define CFG_MSGOUT_UBX_NAV_PVT_UART1 0x20910007u  /* U1 */
#define CFG_RATE_MEAS                0x30210001u  /* U2, ms */
#define CFG_NAVSPG_DYNMODEL          0x20110021u  /* E1, 8 = airborne <4 g */

static uint32_t msg_count;

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

static void send_ubx(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len)
{
    uint8_t hdr[6] = { UBX_SYNC1, UBX_SYNC2, cls, id, (uint8_t)len, (uint8_t)(len >> 8) };
    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 6; i++) { ck_a += hdr[i]; ck_b += ck_a; }
    for (uint16_t i = 0; i < len; i++) { ck_a += payload[i]; ck_b += ck_a; }
    uint8_t ck[2] = { ck_a, ck_b };
    serial_write(SERIAL_GPS, hdr, sizeof hdr);
    serial_write(SERIAL_GPS, payload, len);
    serial_write(SERIAL_GPS, ck, sizeof ck);
}

static uint32_t last_cfg_us;

static void send_config(void)
{
    uint8_t p[4 + 5 * 8];
    size_t n = 0;
    p[n++] = 0x00;          /* version */
    p[n++] = 0x01;          /* layer: RAM */
    p[n++] = 0x00;
    p[n++] = 0x00;
    put_u32(&p[n], CFG_UART1OUTPROT_NMEA); n += 4; p[n++] = 0;
    put_u32(&p[n], CFG_UART1OUTPROT_UBX); n += 4; p[n++] = 1;
    put_u32(&p[n], CFG_MSGOUT_UBX_NAV_PVT_UART1); n += 4; p[n++] = 1;
    put_u32(&p[n], CFG_NAVSPG_DYNMODEL); n += 4; p[n++] = 8;
    put_u32(&p[n], CFG_RATE_MEAS); n += 4; p[n++] = 100; p[n++] = 0;
    send_ubx(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, p, (uint16_t)n);
    last_cfg_us = hw_micros();
}

void gps_init(void)
{
    serial_init(SERIAL_GPS, GPS_BAUD);
    send_config();
}

/* --------------------------------------------------------------------- */
static uint8_t frame[100];
static uint16_t frame_len, frame_pos;
static uint8_t parse_state;

static int32_t le_i32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

static bool decode_pvt(const uint8_t *m, gps_fix_t *o)
{
    o->t_us = hw_micros();
    o->fix_type = m[20];
    o->gnss_fix_ok = (m[21] & 0x01) != 0;
    o->num_sv = m[23];
    o->lon_deg = le_i32(&m[24]) * 1e-7;
    o->lat_deg = le_i32(&m[28]) * 1e-7;
    o->h_msl_m = (float)le_i32(&m[36]) * 1e-3f;
    o->h_acc_m = (float)(uint32_t)le_i32(&m[40]) * 1e-3f;
    o->v_acc_m = (float)(uint32_t)le_i32(&m[44]) * 1e-3f;
    o->vel_ned_mps[0] = (float)le_i32(&m[48]) * 1e-3f;
    o->vel_ned_mps[1] = (float)le_i32(&m[52]) * 1e-3f;
    o->vel_ned_mps[2] = (float)le_i32(&m[56]) * 1e-3f;
    return true;
}

bool gps_poll(gps_fix_t *out)
{
    /* The receiver may still be booting when gps_init() runs; keep
     * re-sending the configuration until NAV-PVT starts arriving. */
    if (msg_count == 0 && hw_micros() - last_cfg_us > 2000000u) send_config();

    uint8_t c;
    bool got = false;
    while (serial_getc(SERIAL_GPS, &c)) {
        switch (parse_state) {
        case 0: if (c == UBX_SYNC1) parse_state = 1; break;
        case 1: parse_state = (c == UBX_SYNC2) ? 2 : 0; break;
        case 2: frame[0] = c; parse_state = 3; break;                      /* class */
        case 3: frame[1] = c; parse_state = 4; break;                      /* id    */
        case 4: frame_len = c; parse_state = 5; break;
        case 5:
            frame_len |= (uint16_t)c << 8;
            frame_pos = 0;
            /* Only NAV-PVT (92 bytes) is kept; everything else is skipped. */
            parse_state = (frame_len <= sizeof frame - 4) ? 6 : 0;
            if (parse_state == 6 && frame_len == 0) parse_state = 7;
            break;
        case 6:
            frame[4 + frame_pos++] = c;
            if (frame_pos >= frame_len) parse_state = 7;
            break;
        case 7: frame[2] = c; parse_state = 8; break;                      /* ck_a */
        case 8: {
            uint8_t a = 0, b = 0;
            a += frame[0]; b += a;
            a += frame[1]; b += a;
            a += (uint8_t)frame_len; b += a;
            a += (uint8_t)(frame_len >> 8); b += a;
            for (uint16_t i = 0; i < frame_len; i++) { a += frame[4 + i]; b += a; }
            if (a == frame[2] && b == c && frame[0] == UBX_CLASS_NAV &&
                frame[1] == UBX_ID_NAV_PVT && frame_len == 92) {
                got = decode_pvt(&frame[4], out);
                msg_count++;
            }
            parse_state = 0;
            break;
        }
        default: parse_state = 0; break;
        }
    }
    return got;
}

uint32_t gps_message_count(void)
{
    return msg_count;
}
