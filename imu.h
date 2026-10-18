/*
 * imu.h - TDK ICM-42688-P on SPI2, 1 kHz data-ready on INT1 (PD8).
 */
#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float gyro[3];    /* rad/s, sensor axes  */
    float accel[3];   /* m/s^2, sensor axes (specific force) */
    float temp_c;
} imu_sample_t;

/* Configures +-2000 dps / +-16 g, 1 kHz ODR, INT1 data-ready, and enables
 * the EXTI interrupt. Returns false if WHO_AM_I is wrong. */
bool imu_init(void);
/* Burst read of temperature, accel and gyro. Called from the EXTI ISR. */
bool imu_read(imu_sample_t *s);

#endif /* IMU_H */
