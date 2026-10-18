/*
 * app.h - main-loop sensor state shared between console and telemetry.
 */
#ifndef APP_H
#define APP_H

#include <stdbool.h>
#include "baro.h"
#include "gps.h"

typedef struct {
    bool baro_ok;
    baro_sample_t baro;
    float baro_alt_m;       /* above the pad reference (0 before ARM) */
    bool gps_valid;         /* at least one NAV-PVT received */
    gps_fix_t gps;
    float vbat_v;
    bool lora_ok;
    bool usb_connected;
} app_sensors_t;

const app_sensors_t *app_sensors(void);
void app_set_stream(bool on);
bool app_stream(void);

#endif /* APP_H */
