/*
 * spiflash.c - W25Q128JV driver.
 */
#include "spiflash.h"
#include "board.h"

#define CMD_WRITE_ENABLE   0x06
#define CMD_READ_STATUS1   0x05
#define CMD_READ_DATA      0x03
#define CMD_PAGE_PROGRAM   0x02
#define CMD_CHIP_ERASE     0xC7
#define CMD_JEDEC_ID       0x9F
#define CMD_RELEASE_PD     0xAB
#define CMD_ENABLE_RESET   0x66
#define CMD_RESET          0x99
#define W25Q128JV_ID       0xEF4018u

static SPI_HandleTypeDef hspi3;
static uint32_t jedec_id;

static inline void cs_low(void)  { FLASH_CS_PORT->BSRR = (uint32_t)FLASH_CS_PIN << 16; }
static inline void cs_high(void) { FLASH_CS_PORT->BSRR = FLASH_CS_PIN; }

static void tx(const uint8_t *p, uint32_t n)
{
    HAL_SPI_Transmit(&hspi3, (uint8_t *)p, (uint16_t)n, 10);
}

static void command(uint8_t c)
{
    cs_low();
    tx(&c, 1);
    cs_high();
}

static uint8_t status1(void)
{
    uint8_t b[2] = { CMD_READ_STATUS1, 0 };
    cs_low();
    HAL_SPI_TransmitReceive(&hspi3, b, b, 2, 10);
    cs_high();
    return b[1];
}

bool spiflash_init(void)
{
    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef g = { 0 };
    g.Pin = FLASH_CS_PIN;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    cs_high();
    HAL_GPIO_Init(FLASH_CS_PORT, &g);

    g.Pin = FLASH_SCK_PIN | FLASH_MISO_PIN | FLASH_MOSI_PIN;
    g.Mode = GPIO_MODE_AF_PP;
    g.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &g);

    hspi3.Instance = FLASH_SPI;
    hspi3.Init.Mode = SPI_MODE_MASTER;
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;          /* mode 0 */
    hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi3.Init.NSS = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;   /* 12.5 MHz */
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
    HAL_SPI_Init(&hspi3);

    command(CMD_RELEASE_PD);
    command(CMD_ENABLE_RESET);
    command(CMD_RESET);
    HAL_Delay(1);

    uint8_t b[4] = { CMD_JEDEC_ID, 0, 0, 0 };
    cs_low();
    HAL_SPI_TransmitReceive(&hspi3, b, b, 4, 10);
    cs_high();
    jedec_id = ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
    return jedec_id == W25Q128JV_ID;
}

uint32_t spiflash_jedec_id(void) { return jedec_id; }

bool spiflash_busy(void) { return (status1() & 0x01) != 0; }

void spiflash_read(uint32_t addr, void *dst, uint32_t n)
{
    uint8_t c[4] = { CMD_READ_DATA, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr };
    cs_low();
    tx(c, 4);
    uint8_t *d = dst;
    while (n) {
        uint16_t chunk = n > 0xFFFF ? 0xFFFF : (uint16_t)n;
        HAL_SPI_Receive(&hspi3, d, chunk, 100);
        d += chunk;
        n -= chunk;
    }
    cs_high();
}

bool spiflash_program_page_start(uint32_t addr, const void *src, uint32_t n)
{
    if (n == 0 || n > SPIFLASH_PAGE_SIZE) return false;
    if ((addr % SPIFLASH_PAGE_SIZE) + n > SPIFLASH_PAGE_SIZE) return false;
    if (spiflash_busy()) return false;
    command(CMD_WRITE_ENABLE);
    uint8_t c[4] = { CMD_PAGE_PROGRAM, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr };
    cs_low();
    tx(c, 4);
    tx(src, n);
    cs_high();
    return true;
}

bool spiflash_chip_erase_start(void)
{
    if (spiflash_busy()) return false;
    command(CMD_WRITE_ENABLE);
    command(CMD_CHIP_ERASE);
    return true;
}
