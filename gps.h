/*
 * gps.h - u-blox MAX-M10S on UART4. Configured at boot (RAM layer, the
 * module has no backup supply) for UBX-NAV-PVT only, 10 Hz, airborne <4 g.
 */
#ifndef GPS_H
#define GPS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t t_us;        /* local receive time */
    uint8_t fix_type;     /* 0 none, 2 = 2D, 3 = 3D */
    uint8_t num_sv;
    bool gnss_fix_ok;
    double lat_deg, lon_deg;
    float h_msl_m;
    float h_acc_m, v_acc_m;
    float vel_ned_mps[3];
} gps_fix_t;

void gps_init(void);
/* Parse received bytes; returns true when *out holds a new NAV-PVT. */
bool gps_poll(gps_fix_t *out);
uint32_t gps_message_count(void);

#endif /* GPS_H */
