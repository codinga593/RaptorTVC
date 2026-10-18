/*
 * spiflash.h - Winbond W25Q128JV (16 MB) on SPI3, single-SPI mode.
 */
#ifndef SPIFLASH_H
#define SPIFLASH_H

#include <stdbool.h>
#include <stdint.h>

#define SPIFLASH_SIZE       (16u * 1024u * 1024u)
#define SPIFLASH_PAGE_SIZE  256u

bool spiflash_init(void);             /* true if the JEDEC ID matches */
uint32_t spiflash_jedec_id(void);
bool spiflash_busy(void);
void spiflash_read(uint32_t addr, void *dst, uint32_t n);
/* Starts a page program and returns; poll spiflash_busy() before the next
 * write/erase. n <= 256 and must not cross a page boundary. */
bool spiflash_program_page_start(uint32_t addr, const void *src, uint32_t n);
bool spiflash_chip_erase_start(void);  /* 40-200 s; poll spiflash_busy() */

#endif /* SPIFLASH_H */
