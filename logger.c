/*
 * logger.c - see logger.h.
 */
#include "logger.h"
#include "console.h"
#include "flight.h"
#include "spiflash.h"

#include <stdio.h>
#include <string.h>

#define PRETRIGGER_RECORDS 200u                  /* 2 s at 100 Hz */
#define PAGE_QUEUE         64u                   /* 16 kB = 2 s of data */
#define STOP_AFTER_LANDED_MS 3000u

static bool flash_ok;
static uint32_t write_addr;                      /* next page to program */

static log_record_t pre[PRETRIGGER_RECORDS];
static uint32_t pre_head, pre_count;

static uint8_t pages[PAGE_QUEUE][SPIFLASH_PAGE_SIZE] __attribute__((section(".axi_bss")));
static uint32_t pq_head, pq_tail;                /* full pages waiting for flash */
static uint32_t fill;                            /* bytes in pages[pq_head] */

static bool recording;
static uint32_t landed_since_ms;
static uint32_t dropped;

static bool erasing;
static bool dumping;
static uint32_t dump_addr, dump_flight, dump_records;

/* ---------------------------------------------------------------------
 * Initialisation: binary search for the first erased page.
 * The log is written contiguously from address 0.
 * ------------------------------------------------------------------- */
static bool page_erased(uint32_t page)
{
    uint8_t b[16];
    spiflash_read(page * SPIFLASH_PAGE_SIZE, b, sizeof b);
    for (unsigned i = 0; i < sizeof b; i++) {
        if (b[i] != 0xFF) return false;
    }
    return true;
}

bool logger_init(void)
{
    flash_ok = spiflash_init();
    if (!flash_ok) return false;
    uint32_t lo = 0, hi = SPIFLASH_SIZE / SPIFLASH_PAGE_SIZE;   /* first erased page in [lo, hi] */
    while (lo < hi) {
        uint32_t mid = (lo + hi) / 2;
        if (page_erased(mid)) hi = mid; else lo = mid + 1;
    }
    write_addr = lo * SPIFLASH_PAGE_SIZE;
    return true;
}

bool logger_ok(void) { return flash_ok; }
uint32_t logger_used_bytes(void) { return write_addr; }
uint32_t logger_free_bytes(void) { return flash_ok ? SPIFLASH_SIZE - write_addr : 0; }
bool logger_recording(void) { return recording; }
uint32_t logger_dropped(void) { return dropped; }
bool logger_erasing(void) { return erasing; }
bool logger_dumping(void) { return dumping; }

/* ---------------------------------------------------------------------
 * Writing
 * ------------------------------------------------------------------- */
static void append(const void *data, uint32_t n)
{
    const uint8_t *p = data;
    while (n) {
        if (pq_head - pq_tail >= PAGE_QUEUE) {     /* flash can't keep up */
            dropped++;
            return;
        }
        uint32_t room = SPIFLASH_PAGE_SIZE - fill;
        uint32_t k = n < room ? n : room;
        memcpy(&pages[pq_head % PAGE_QUEUE][fill], p, k);
        fill += k;
        p += k;
        n -= k;
        if (fill == SPIFLASH_PAGE_SIZE) {
            pq_head++;
            fill = 0;
        }
    }
}

static void flush_partial_page(void)
{
    if (fill == 0) return;
    memset(&pages[pq_head % PAGE_QUEUE][fill], 0xFF, SPIFLASH_PAGE_SIZE - fill);
    pq_head++;
    fill = 0;
}

static void write_pages(void)
{
    if (pq_tail == pq_head || spiflash_busy()) return;
    if (write_addr + SPIFLASH_PAGE_SIZE > SPIFLASH_SIZE) {      /* full */
        dropped += pq_head - pq_tail;
        pq_tail = pq_head;
        return;
    }
    if (spiflash_program_page_start(write_addr, pages[pq_tail % PAGE_QUEUE], SPIFLASH_PAGE_SIZE)) {
        write_addr += SPIFLASH_PAGE_SIZE;
        pq_tail++;
    }
}

static void handle_record(const log_record_t *r)
{
    if (r->type == LOG_TYPE_HEADER) {
        /* Launch: header, then the pre-trigger history, then live data. */
        if (!flash_ok || erasing) return;
        recording = true;
        landed_since_ms = 0;
        append(r, sizeof *r);
        uint32_t start = (pre_head + PRETRIGGER_RECORDS - pre_count) % PRETRIGGER_RECORDS;
        for (uint32_t i = 0; i < pre_count; i++) append(&pre[(start + i) % PRETRIGGER_RECORDS], sizeof *r);
        pre_count = 0;
        return;
    }

    if (!recording) {
        if (r->state == FS_ARMED) {
            pre[pre_head] = *r;
            pre_head = (pre_head + 1) % PRETRIGGER_RECORDS;
            if (pre_count < PRETRIGGER_RECORDS) pre_count++;
        } else {
            pre_count = 0;
        }
        return;
    }

    append(r, sizeof *r);
    if (r->state == FS_LANDED) {
        if (landed_since_ms == 0) landed_since_ms = r->t_ms ? r->t_ms : 1;
        if (r->t_ms - landed_since_ms > STOP_AFTER_LANDED_MS) {
            flush_partial_page();
            recording = false;
        }
    }
}

/* ---------------------------------------------------------------------
 * Erase / dump
 * ------------------------------------------------------------------- */
bool logger_start_erase(void)
{
    if (!flash_ok || recording || dumping || erasing) return false;
    if (!spiflash_chip_erase_start()) return false;
    erasing = true;
    return true;
}

bool logger_start_dump(void)
{
    if (!flash_ok || recording || erasing || dumping) return false;
    dumping = true;
    dump_addr = 0;
    dump_flight = 0;
    dump_records = 0;
    console_printf("flight,type,state,t_ms,qw,qx,qy,qz,p,q,r,ax,ay,az,"
                   "cmd_pitch,cmd_yaw,err_pitch,err_yaw,pos_d,vel_d,baro_alt,vbat_mv\r\n");
    return true;
}

void logger_cancel_dump(void) { dumping = false; }

static void dump_step(void)
{
    /* Emit a few records per pass, only while the console has room. */
    for (int n = 0; n < 8 && console_tx_free() > 400; n++) {
        if (dump_addr + sizeof(log_record_t) > write_addr) {
            console_printf("# end of log: %lu flights, %lu records\r\n",
                           (unsigned long)dump_flight, (unsigned long)dump_records);
            dumping = false;
            return;
        }
        log_record_t r;
        spiflash_read(dump_addr, &r, sizeof r);
        if (r.magic != LOG_MAGIC || log_crc16(&r, sizeof r - 2) != r.crc) {
            if (r.magic == 0xFFFFu) {
                /* 0xFF padding at the end of a flight: next page. */
                dump_addr = (dump_addr / SPIFLASH_PAGE_SIZE + 1) * SPIFLASH_PAGE_SIZE;
            } else {
                dump_addr++;          /* corrupt record: resynchronise */
            }
            continue;
        }
        dump_addr += sizeof r;
        if (r.type == LOG_TYPE_HEADER) dump_flight++;
        dump_records++;
        console_printf("%lu,%u,%s,%lu,%.6f,%.6f,%.6f,%.6f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,"
                       "%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%u\r\n",
                       (unsigned long)dump_flight, r.type, flight_state_name((flight_state_t)r.state),
                       (unsigned long)r.t_ms,
                       (double)r.q[0], (double)r.q[1], (double)r.q[2], (double)r.q[3],
                       (double)r.gyro[0], (double)r.gyro[1], (double)r.gyro[2],
                       (double)r.accel[0], (double)r.accel[1], (double)r.accel[2],
                       (double)r.cmd[0], (double)r.cmd[1], (double)r.err[0], (double)r.err[1],
                       (double)r.pos_d, (double)r.vel_d, (double)r.baro_alt, r.vbat_mv);
    }
}

/* --------------------------------------------------------------------- */
void logger_task(void)
{
    log_record_t r;
    while (flight_pop_record(&r)) handle_record(&r);

    /* No records are produced in IDLE, so a disarm (e.g. right after
     * landing) has to be noticed here. */
    if (recording) {
        flight_status_t s;
        flight_get_status(&s);
        if (s.state == FS_IDLE) {
            flush_partial_page();
            recording = false;
        }
    }

    if (erasing) {
        if (!spiflash_busy()) {
            erasing = false;
            write_addr = 0;
            console_printf("log erase complete\r\n");
        }
        return;
    }
    write_pages();
    if (dumping && pq_tail == pq_head) dump_step();
}
