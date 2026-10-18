/*
 * console.c - see console.h.
 */
#include "console.h"
#include "app.h"
#include "flight.h"
#include "logger.h"
#include "serial.h"
#include "stm32h7xx_hal.h"
#include "tvc_config.h"
#include "usbd_cdc_if.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FW_VERSION "TVC_Raptor_FC 1.0 (" __DATE__ ")"

typedef enum { SRC_USB, SRC_PI } src_t;

static uint8_t usb_rx_buf[512];
static uint8_t usb_tx_buf[8192];
static ring_t usb_rx, usb_tx;
static src_t reply = SRC_USB;

typedef struct {
    char buf[96];
    unsigned len;
} line_t;
static line_t line_usb, line_pi;

void console_init(void)
{
    ring_init(&usb_rx, usb_rx_buf, sizeof usb_rx_buf);
    ring_init(&usb_tx, usb_tx_buf, sizeof usb_tx_buf);
}

void console_usb_rx(const uint8_t *data, uint32_t n)
{
    ring_write(&usb_rx, data, n);
}

size_t console_write(const void *data, size_t n)
{
    if (reply == SRC_PI) return serial_write(SERIAL_PI, data, n);
    return ring_write(&usb_tx, data, n);
}

size_t console_tx_free(void)
{
    return sizeof usb_tx_buf - ring_count(&usb_tx);
}

int console_printf(const char *fmt, ...)
{
    char b[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    if (n < 0) return n;
    if (n > (int)sizeof b - 1) n = sizeof b - 1;
    console_write(b, (size_t)n);
    return n;
}

void console_usb_flush(void)
{
    static uint8_t pkt[512];
    if (!cdc_ready()) {
        /* Nobody listening: don't let stale output pile up. */
        if (!cdc_open()) ring_consume(&usb_tx, ring_count(&usb_tx));
        return;
    }
    const uint8_t *p;
    size_t n = ring_peek_block(&usb_tx, &p);
    if (n == 0) return;
    if (n > sizeof pkt) n = sizeof pkt;
    memcpy(pkt, p, n);
    if (cdc_transmit(pkt, (uint16_t)n)) ring_consume(&usb_tx, n);
}

/* ---------------------------------------------------------------------
 * Commands
 * ------------------------------------------------------------------- */
static void cmd_help(void)
{
    console_printf(
        "commands:\r\n"
        "  status                 one-shot status\r\n"
        "  stream on|off          10 Hz status lines\r\n"
        "  arm                    calibrate on the pad and arm (USB only)\r\n"
        "  disarm                 back to IDLE (not during powered flight/coast)\r\n"
        "  cal                    re-run gyro bias / attitude calibration\r\n"
        "  bench on|off           run the controller live without launch (no motor!)\r\n"
        "  servo center|off       release manual servo control\r\n"
        "  servo deg <p> <y>      set gimbal angles (uses calibration)\r\n"
        "  servo us <p> <y>       set raw pulse widths for pitch/yaw servos\r\n"
        "  sensors                IMU, baro, GPS, battery details\r\n"
        "  log info|dump|erase    flight log (dump prints CSV)\r\n"
        "  version, reboot\r\n");
}

static void print_status(void)
{
    flight_status_t s;
    flight_get_status(&s);
    const app_sensors_t *a = app_sensors();
    console_printf("%s%s t=%.2f tilt=%.2f err=%.2f,%.2f cmd=%.2f,%.2f rate=%.1f,%.1f,%.1f "
                   "alt=%.1f vbat=%.2f\r\n",
                   flight_state_name(s.state), s.bench ? "(bench)" : "", (double)s.t_flight_s,
                   (double)s.tilt_deg, (double)s.err[0], (double)s.err[1],
                   (double)s.cmd[0], (double)s.cmd[1],
                   (double)(s.gyro[0] * 57.2958f), (double)(s.gyro[1] * 57.2958f),
                   (double)(s.gyro[2] * 57.2958f), (double)a->baro_alt_m, (double)a->vbat_v);
}

static void cmd_status(void)
{
    flight_status_t s;
    flight_get_status(&s);
    print_status();
    console_printf("  last: %s\r\n", s.last_msg);
    console_printf("  imu %s, samples %lu, errors %lu, overruns %lu, max isr %lu us, ukf faults %lu\r\n",
                   s.imu_ok ? "ok" : "FAIL", (unsigned long)s.imu_samples, (unsigned long)s.imu_errors,
                   (unsigned long)s.overruns, (unsigned long)s.max_isr_us, (unsigned long)s.est_faults);
    console_printf("  q = [%.5f %.5f %.5f %.5f], gyro bias = [%.3f %.3f %.3f] dps\r\n",
                   (double)s.q[0], (double)s.q[1], (double)s.q[2], (double)s.q[3],
                   (double)(s.gyro_bias[0] * 57.2958f), (double)(s.gyro_bias[1] * 57.2958f),
                   (double)(s.gyro_bias[2] * 57.2958f));
    console_printf("  log: %s, used %lu kB, free %lu kB%s%s\r\n",
                   logger_ok() ? "ok" : "NO FLASH", (unsigned long)(logger_used_bytes() / 1024),
                   (unsigned long)(logger_free_bytes() / 1024),
                   logger_recording() ? ", RECORDING" : "", logger_erasing() ? ", ERASING" : "");
}

static void cmd_sensors(void)
{
    flight_status_t s;
    flight_get_status(&s);
    const app_sensors_t *a = app_sensors();
    console_printf("accel body = [%.3f %.3f %.3f] m/s^2, imu temp %.1f C\r\n",
                   (double)s.accel[0], (double)s.accel[1], (double)s.accel[2], (double)s.imu_temp_c);
    console_printf("gyro body  = [%.3f %.3f %.3f] dps (bias removed)\r\n",
                   (double)(s.gyro[0] * 57.2958f), (double)(s.gyro[1] * 57.2958f), (double)(s.gyro[2] * 57.2958f));
    if (a->baro_ok) {
        console_printf("baro %.1f Pa, %.2f C, alt above pad %.2f m\r\n",
                       (double)a->baro.pressure_pa, (double)a->baro.temp_c, (double)a->baro_alt_m);
    } else {
        console_printf("baro NOT FOUND\r\n");
    }
    if (a->gps_valid) {
        console_printf("gps fix %u, sv %u, lat %.7f lon %.7f hMSL %.1f m (vAcc %.1f m), msgs %lu\r\n",
                       a->gps.fix_type, a->gps.num_sv, a->gps.lat_deg, a->gps.lon_deg,
                       (double)a->gps.h_msl_m, (double)a->gps.v_acc_m, (unsigned long)gps_message_count());
    } else {
        console_printf("gps: no NAV-PVT yet\r\n");
    }
    console_printf("battery %.2f V, lora %s\r\n", (double)a->vbat_v,
                   TVC_LORA_FREQ_HZ == 0 ? "disabled (set TVC_LORA_FREQ_HZ)" : (a->lora_ok ? "ok" : "NOT FOUND"));
}

static void cmd_arm(src_t src)
{
    flight_status_t s;
    flight_get_status(&s);
    const app_sensors_t *a = app_sensors();
    const char *why = NULL;
    if (src != SRC_USB) why = "arm is only accepted over USB";
    else if (s.state != FS_IDLE) why = "not in IDLE";
    else if (!s.imu_ok) why = "IMU not working";
    else if (!logger_ok()) why = "log flash not found";
    else if (logger_erasing() || logger_dumping()) why = "log erase/dump in progress";
    else if (logger_free_bytes() < TVC_LOG_MIN_FREE_BYTES) why = "log flash full - run 'log erase'";
    else if (a->vbat_v < TVC_VBAT_MIN_ARM_V) why = "battery low";
    if (why) {
        console_printf("ARM REFUSED: %s\r\n", why);
        return;
    }
    flight_servo_manual(SERVO_MANUAL_OFF, 0, 0);
    flight_command(FLIGHT_CMD_ARM);
    console_printf("calibrating for %.1f s - keep the rocket still on the pad...\r\n", TVC_CAL_TIME_S);
}

static void cmd_servo(int argc, char **argv)
{
    flight_status_t s;
    flight_get_status(&s);
    if (s.state != FS_IDLE || s.bench) {
        console_printf("servo commands only in IDLE with bench off\r\n");
        return;
    }
    if (argc >= 2 && (!strcmp(argv[1], "center") || !strcmp(argv[1], "off"))) {
        flight_servo_manual(SERVO_MANUAL_OFF, 0, 0);
        console_printf("servos centred\r\n");
    } else if (argc >= 4 && !strcmp(argv[1], "deg")) {
        float p = strtof(argv[2], NULL), y = strtof(argv[3], NULL);
        if (p > TVC_MAX_GIMBAL_DEG) p = TVC_MAX_GIMBAL_DEG;
        if (p < -TVC_MAX_GIMBAL_DEG) p = -TVC_MAX_GIMBAL_DEG;
        if (y > TVC_MAX_GIMBAL_DEG) y = TVC_MAX_GIMBAL_DEG;
        if (y < -TVC_MAX_GIMBAL_DEG) y = -TVC_MAX_GIMBAL_DEG;
        flight_servo_manual(SERVO_MANUAL_DEG, p, y);
        console_printf("gimbal pitch %.2f deg, yaw %.2f deg\r\n", (double)p, (double)y);
    } else if (argc >= 4 && !strcmp(argv[1], "us")) {
        float p = strtof(argv[2], NULL), y = strtof(argv[3], NULL);
        flight_servo_manual(SERVO_MANUAL_US, p, y);
        console_printf("pitch servo %.0f us, yaw servo %.0f us (clamped to %.0f..%.0f)\r\n",
                       (double)p, (double)y, (double)TVC_SERVO_MIN_US, (double)TVC_SERVO_MAX_US);
    } else {
        console_printf("usage: servo center | servo deg <p> <y> | servo us <p> <y>\r\n");
    }
}

static void cmd_log(int argc, char **argv, src_t src)
{
    const char *sub = argc >= 2 ? argv[1] : "info";
    flight_status_t s;
    flight_get_status(&s);
    if (!strcmp(sub, "info")) {
        console_printf("log flash %s (JEDEC ok), used %lu bytes (%lu records max), free %lu kB, dropped %lu\r\n",
                       logger_ok() ? "ok" : "NOT FOUND", (unsigned long)logger_used_bytes(),
                       (unsigned long)(logger_used_bytes() / sizeof(log_record_t)),
                       (unsigned long)(logger_free_bytes() / 1024), (unsigned long)logger_dropped());
    } else if (!strcmp(sub, "dump")) {
        if (src != SRC_USB) { console_printf("dump only over USB\r\n"); return; }
        if (s.state != FS_IDLE) { console_printf("dump only in IDLE\r\n"); return; }
        if (!logger_start_dump()) console_printf("cannot dump now\r\n");
    } else if (!strcmp(sub, "erase")) {
        if (s.state != FS_IDLE) { console_printf("erase only in IDLE\r\n"); return; }
        if (logger_start_erase()) console_printf("erasing 16 MB, takes up to ~3 minutes...\r\n");
        else console_printf("cannot erase now\r\n");
    } else {
        console_printf("usage: log info|dump|erase\r\n");
    }
}

static void execute(char *ln, src_t src)
{
    char *argv[6];
    int argc = 0;
    for (char *t = strtok(ln, " \t"); t && argc < 6; t = strtok(NULL, " \t")) argv[argc++] = t;
    if (argc == 0) return;

    reply = src;
    const char *c = argv[0];
    if (!strcmp(c, "help") || !strcmp(c, "?")) cmd_help();
    else if (!strcmp(c, "status")) cmd_status();
    else if (!strcmp(c, "stream")) {
        bool on = argc >= 2 && !strcmp(argv[1], "on");
        app_set_stream(on);
        console_printf("stream %s\r\n", on ? "on" : "off");
    }
    else if (!strcmp(c, "arm")) cmd_arm(src);
    else if (!strcmp(c, "disarm")) { flight_command(FLIGHT_CMD_DISARM); console_printf("disarm requested\r\n"); }
    else if (!strcmp(c, "cal")) { flight_command(FLIGHT_CMD_CAL); console_printf("calibrating, keep still...\r\n"); }
    else if (!strcmp(c, "bench")) {
        bool on = argc >= 2 && !strcmp(argv[1], "on");
        if (on) flight_servo_manual(SERVO_MANUAL_OFF, 0, 0);
        flight_command(on ? FLIGHT_CMD_BENCH_ON : FLIGHT_CMD_BENCH_OFF);
        console_printf("bench %s%s\r\n", on ? "on" : "off",
                       on ? " - tilt the rocket: the gimbal should push the nose back to vertical" : "");
    }
    else if (!strcmp(c, "servo")) cmd_servo(argc, argv);
    else if (!strcmp(c, "sensors")) cmd_sensors();
    else if (!strcmp(c, "log")) cmd_log(argc, argv, src);
    else if (!strcmp(c, "version")) console_printf("%s\r\n", FW_VERSION);
    else if (!strcmp(c, "reboot")) {
        flight_status_t s;
        flight_get_status(&s);
        if (s.state == FS_IDLE || s.state == FS_LANDED) NVIC_SystemReset();
        console_printf("refused: not in IDLE/LANDED\r\n");
    }
    else console_printf("unknown command '%s' - try help\r\n", c);
    reply = SRC_USB;
}

static void feed(line_t *l, uint8_t ch, src_t src)
{
    if (ch == '\r' || ch == '\n') {
        if (l->len) {
            l->buf[l->len] = '\0';
            execute(l->buf, src);
            l->len = 0;
        }
    } else if (ch == 0x03) {                 /* Ctrl-C stops a dump */
        logger_cancel_dump();
    } else if (l->len < sizeof l->buf - 1 && ch >= 0x20 && ch < 0x7F) {
        l->buf[l->len++] = (char)ch;
    }
}

void console_task(void)
{
    uint8_t ch;
    while (ring_get(&usb_rx, &ch)) feed(&line_usb, ch, SRC_USB);
    while (serial_getc(SERIAL_PI, &ch)) feed(&line_pi, ch, SRC_PI);

    static uint32_t last_stream;
    if (app_stream() && HAL_GetTick() - last_stream >= 100u) {
        last_stream = HAL_GetTick();
        if (!logger_dumping()) print_status();
    }
    console_usb_flush();
}
