/*
 * baro.h - MS5611-01BA on I2C4 (0x77), polled from the main loop.
 * Runs D1 (pressure) conversions back to back at OSR 4096, with a D2
 * (temperature) conversion every 10th cycle: ~90 pressure readings/s.
 */
#ifndef BARO_H
#define BARO_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float pressure_pa;
    float temp_c;
    uint32_t t_us;
} baro_sample_t;

bool baro_init(void);
/* Advance the conversion state machine; returns true when *out is new. */
bool baro_poll(baro_sample_t *out);
/* Barometric altitude of p relative to reference pressure p0 (m, up). */
float baro_altitude_m(float p_pa, float p0_pa);

#endif /* BARO_H */
