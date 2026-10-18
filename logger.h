/*
 * logger.h - flight log on the W25Q128 (main loop only).
 *
 * Records from the control ISR are kept in a 2 s RAM ring while ARMED.
 * At launch the ring and every following record are appended to flash,
 * starting at the first erased page; recording stops 3 s after LANDED or
 * on disarm. Page programs are started and polled, never waited on.
 */
#ifndef LOGGER_H
#define LOGGER_H

#include <stdbool.h>
#include <stdint.h>

bool logger_init(void);           /* finds the end of the existing log */
void logger_task(void);           /* call every main-loop pass */

bool logger_ok(void);
uint32_t logger_used_bytes(void);
uint32_t logger_free_bytes(void);
bool logger_recording(void);
uint32_t logger_dropped(void);

/* Console helpers. Output goes through console_write(). */
bool logger_start_erase(void);    /* refused while recording */
bool logger_erasing(void);
bool logger_start_dump(void);     /* CSV of every record */
bool logger_dumping(void);
void logger_cancel_dump(void);

#endif /* LOGGER_H */
