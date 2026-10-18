/*
 * hw.h - board-level drivers (timebase, servos, battery, watchdog).
 */
#ifndef HW_H
#define HW_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"

/* Free-running 1 MHz counter on TIM2 (32 bit, wraps after ~71 min). */
void hw_time_init(void);
static inline uint32_t hw_micros(void) { return TIM2->CNT; }

/* Servos: TIM1 CH1..CH4 = J1, J9, J10, J11. Channels start disabled
 * (no pulses) until servo_enable() is called for them. */
void servo_init(void);
void servo_enable(unsigned ch, bool on);
void servo_write_us(unsigned ch, float us);
/* Apply tvc_config.h calibration and write both gimbal servos. */
void servo_set_gimbal(float pitch_deg, float yaw_deg);

/* Battery voltage (V), blocking ~20 us conversion. */
void battery_init(void);
float battery_read_v(void);

/* Independent watchdog, ~250 ms. */
void watchdog_init(void);
void watchdog_kick(void);

#endif /* HW_H */
