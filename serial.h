/*
 * serial.h - interrupt-driven UART with RX/TX ring buffers (UART4 = GPS,
 * UART7 = Raspberry Pi), plus a generic byte ring used by the USB console.
 */
#ifndef SERIAL_H
#define SERIAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Single-producer / single-consumer byte ring, size must be a power of 2. */
typedef struct {
    uint8_t *buf;
    uint32_t mask;
    volatile uint32_t head;   /* written by producer */
    volatile uint32_t tail;   /* written by consumer */
} ring_t;

void ring_init(ring_t *r, uint8_t *buf, uint32_t size);
bool ring_put(ring_t *r, uint8_t c);
size_t ring_write(ring_t *r, const void *data, size_t n);   /* drops what doesn't fit */
bool ring_get(ring_t *r, uint8_t *c);
size_t ring_count(const ring_t *r);
/* Longest contiguous readable block (for zero-copy sends). */
size_t ring_peek_block(const ring_t *r, const uint8_t **p);
void ring_consume(ring_t *r, size_t n);

typedef enum { SERIAL_GPS = 0, SERIAL_PI = 1, SERIAL_COUNT } serial_port_t;

void serial_init(serial_port_t port, uint32_t baud);
void serial_set_baud(serial_port_t port, uint32_t baud);
size_t serial_write(serial_port_t port, const void *data, size_t n);
bool serial_getc(serial_port_t port, uint8_t *c);
void serial_irq(serial_port_t port);        /* call from the USART IRQ handler */

#endif /* SERIAL_H */
