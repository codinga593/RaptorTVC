/*
 * telemetry.h - LoRa downlink (binary, TVC_LORA_PERIOD_MS) and a CSV
 * stream to the Raspberry Pi on UART7 (TVC_PI_PERIOD_MS).
 *
 * Pi line format:
 *   $TVC,t_ms,state,t_flight,qw,qx,qy,qz,tilt,err_p,err_y,cmd_p,cmd_y,
 *        p,q,r,pos_d,vel_d,baro_alt,gps_fix,gps_sv,lat,lon,h_msl,vbat\r\n
 * LoRa packet (little endian, 38 bytes): see telemetry.c.
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>

bool telemetry_init(void);      /* returns LoRa radio status */
void telemetry_task(void);

#endif /* TELEMETRY_H */
