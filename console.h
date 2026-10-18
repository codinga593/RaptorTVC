/*
 * console.h - text command console on USB CDC and the Pi UART.
 * Type "help" for the command list.
 */
#ifndef CONSOLE_H
#define CONSOLE_H

#include <stddef.h>
#include <stdint.h>

void console_init(void);
void console_task(void);

/* Output goes to whichever port issued the command being handled
 * (USB outside of command handling). */
int console_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
size_t console_write(const void *data, size_t n);
size_t console_tx_free(void);

/* Called from the USB CDC receive callback (interrupt context). */
void console_usb_rx(const uint8_t *data, uint32_t n);
/* Pull bytes from the USB TX ring into the CDC endpoint (main loop). */
void console_usb_flush(void);

#endif /* CONSOLE_H */
