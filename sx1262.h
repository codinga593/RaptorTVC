/*
 * sx1262.h - minimal Semtech SX1262 LoRa transmitter (Seeed Wio-SX1262).
 * TX only. The module's TCXO is powered from DIO3; RF_SW (PC7) is the
 * module's receive-enable input and is held low.
 */
#ifndef SX1262_H
#define SX1262_H

#include <stdbool.h>
#include <stdint.h>

bool sx1262_init(uint32_t freq_hz, int8_t tx_dbm, uint8_t sf);
/* Start sending a packet (<= 255 bytes). Returns false if still busy. */
bool sx1262_send(const uint8_t *data, uint8_t len);
bool sx1262_tx_busy(void);

#endif /* SX1262_H */
