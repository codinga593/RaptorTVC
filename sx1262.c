/*
 * sx1262.c - see sx1262.h. Command set from the SX1261/2 datasheet rev 2.1.
 */
#include "sx1262.h"
#include "board.h"

#define OP_SET_STANDBY            0x80
#define OP_SET_PACKET_TYPE        0x8A
#define OP_SET_RF_FREQUENCY       0x86
#define OP_SET_PA_CONFIG          0x95
#define OP_SET_TX_PARAMS          0x8E
#define OP_SET_BUFFER_BASE        0x8F
#define OP_SET_MODULATION_PARAMS  0x8B
#define OP_SET_PACKET_PARAMS      0x8C
#define OP_SET_DIO_IRQ_PARAMS     0x08
#define OP_CLEAR_IRQ              0x02
#define OP_GET_IRQ_STATUS         0x12
#define OP_WRITE_BUFFER           0x0E
#define OP_SET_TX                 0x83
#define OP_SET_DIO3_AS_TCXO       0x97
#define OP_SET_DIO2_AS_RF_SWITCH  0x9D
#define OP_CALIBRATE              0x89
#define OP_CALIBRATE_IMAGE        0x98
#define OP_SET_REGULATOR_MODE     0x96
#define OP_GET_STATUS             0xC0
#define OP_WRITE_REGISTER         0x0D

#define IRQ_TX_DONE               0x0001
#define IRQ_TIMEOUT               0x0200

static SPI_HandleTypeDef hspi1;
static bool ready;
static bool tx_active;
static uint32_t tx_start_ms;

static inline void cs_low(void)  { LORA_CS_PORT->BSRR = (uint32_t)LORA_CS_PIN << 16; }
static inline void cs_high(void) { LORA_CS_PORT->BSRR = LORA_CS_PIN; }

static bool wait_busy(uint32_t timeout_ms)
{
    uint32_t t = HAL_GetTick();
    while (HAL_GPIO_ReadPin(LORA_BUSY_PORT, LORA_BUSY_PIN) == GPIO_PIN_SET) {
        if (HAL_GetTick() - t > timeout_ms) return false;
    }
    return true;
}

static bool command(uint8_t op, const uint8_t *args, uint8_t n)
{
    if (!wait_busy(20)) return false;
    cs_low();
    HAL_SPI_Transmit(&hspi1, &op, 1, 5);
    if (n) HAL_SPI_Transmit(&hspi1, (uint8_t *)args, n, 5);
    cs_high();
    return true;
}

static bool read_cmd(uint8_t op, uint8_t *out, uint8_t n)
{
    if (!wait_busy(20)) return false;
    uint8_t b[8] = { op, 0 };
    cs_low();
    HAL_SPI_TransmitReceive(&hspi1, b, b, (uint16_t)(2 + n), 5);
    cs_high();
    for (uint8_t i = 0; i < n; i++) out[i] = b[2 + i];
    return true;
}

static void gpio_spi_init(void)
{
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef g = { 0 };
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Speed = GPIO_SPEED_FREQ_MEDIUM;
    g.Pin = LORA_CS_PIN;
    cs_high();
    HAL_GPIO_Init(LORA_CS_PORT, &g);
    g.Pin = LORA_RST_PIN | LORA_RFSW_PIN;
    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LORA_RFSW_PORT, LORA_RFSW_PIN, GPIO_PIN_RESET);   /* RX enable off */
    HAL_GPIO_Init(GPIOC, &g);

    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;
    g.Pin = LORA_BUSY_PIN | LORA_DIO1_PIN;
    HAL_GPIO_Init(GPIOC, &g);

    g.Pin = LORA_SCK_PIN | LORA_MISO_PIN | LORA_MOSI_PIN;
    g.Mode = GPIO_MODE_AF_PP;
    g.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &g);

    hspi1.Instance = LORA_SPI;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;          /* mode 0 */
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;   /* 6.25 MHz */
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
    HAL_SPI_Init(&hspi1);
}

bool sx1262_init(uint32_t freq_hz, int8_t tx_dbm, uint8_t sf)
{
    ready = false;
    gpio_spi_init();
    if (freq_hz == 0) return false;

    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
    if (!wait_busy(50)) return false;

    uint8_t a[8];
    a[0] = 0x00;                                   /* STDBY_RC */
    if (!command(OP_SET_STANDBY, a, 1)) return false;
    uint8_t st;
    if (!read_cmd(OP_GET_STATUS, &st, 0)) return false;

    /* TCXO on DIO3 at 1.8 V, 5 ms start-up (320 x 15.625 us) */
    a[0] = 0x02; a[1] = 0x00; a[2] = 0x01; a[3] = 0x40;
    command(OP_SET_DIO3_AS_TCXO, a, 4);
    a[0] = 0x7F;                                   /* calibrate all blocks */
    command(OP_CALIBRATE, a, 1);
    HAL_Delay(5);
    if (!wait_busy(50)) return false;
    a[0] = 0x01;                                   /* DC-DC regulator */
    command(OP_SET_REGULATOR_MODE, a, 1);
    a[0] = 0x01;
    command(OP_SET_DIO2_AS_RF_SWITCH, a, 1);
    a[0] = 0x01;                                   /* LoRa */
    command(OP_SET_PACKET_TYPE, a, 1);

    /* Image calibration for the band */
    if (freq_hz > 900000000u)      { a[0] = 0xE1; a[1] = 0xE9; }
    else if (freq_hz > 850000000u) { a[0] = 0xD7; a[1] = 0xDB; }
    else if (freq_hz > 770000000u) { a[0] = 0xC1; a[1] = 0xC5; }
    else if (freq_hz > 460000000u) { a[0] = 0x75; a[1] = 0x81; }
    else                           { a[0] = 0x6B; a[1] = 0x6F; }
    command(OP_CALIBRATE_IMAGE, a, 2);

    uint32_t frf = (uint32_t)(((uint64_t)freq_hz << 25) / 32000000u);
    a[0] = (uint8_t)(frf >> 24); a[1] = (uint8_t)(frf >> 16); a[2] = (uint8_t)(frf >> 8); a[3] = (uint8_t)frf;
    command(OP_SET_RF_FREQUENCY, a, 4);

    a[0] = 0x04; a[1] = 0x07; a[2] = 0x00; a[3] = 0x01;   /* SX1262 PA, up to +22 dBm */
    command(OP_SET_PA_CONFIG, a, 4);
    if (tx_dbm > 22) tx_dbm = 22;
    if (tx_dbm < -9) tx_dbm = -9;
    a[0] = (uint8_t)tx_dbm; a[1] = 0x04;                  /* 200 us ramp */
    command(OP_SET_TX_PARAMS, a, 2);

    a[0] = 0x00; a[1] = 0x00;
    command(OP_SET_BUFFER_BASE, a, 2);

    a[0] = sf; a[1] = 0x04; a[2] = 0x01;                  /* BW 125 kHz, CR 4/5 */
    a[3] = (sf >= 11) ? 0x01 : 0x00;                      /* low data rate optimise */
    command(OP_SET_MODULATION_PARAMS, a, 4);

    /* TxDone | Timeout -> DIO1 */
    a[0] = 0x02; a[1] = 0x01; a[2] = 0x02; a[3] = 0x01; a[4] = 0; a[5] = 0; a[6] = 0; a[7] = 0;
    command(OP_SET_DIO_IRQ_PARAMS, a, 8);

    ready = true;
    return true;
}

bool sx1262_tx_busy(void)
{
    if (!tx_active) return false;
    uint8_t irq[2];
    if (HAL_GPIO_ReadPin(LORA_DIO1_PORT, LORA_DIO1_PIN) == GPIO_PIN_SET ||
        HAL_GetTick() - tx_start_ms > 3000u) {
        read_cmd(OP_GET_IRQ_STATUS, irq, 2);
        uint8_t c[2] = { 0xFF, 0xFF };
        command(OP_CLEAR_IRQ, c, 2);
        tx_active = false;
    }
    return tx_active;
}

bool sx1262_send(const uint8_t *data, uint8_t len)
{
    if (!ready || sx1262_tx_busy()) return false;

    uint8_t a[6] = { 0x00, 0x08,   /* preamble 8 */
                     0x00,         /* explicit header */
                     len,
                     0x01,         /* CRC on */
                     0x00 };       /* standard IQ */
    command(OP_SET_PACKET_PARAMS, a, 6);

    if (!wait_busy(20)) return false;
    uint8_t hdr[2] = { OP_WRITE_BUFFER, 0x00 };
    cs_low();
    HAL_SPI_Transmit(&hspi1, hdr, 2, 5);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)data, len, 10);
    cs_high();

    uint8_t t[3] = { 0x00, 0x00, 0x00 };   /* no timeout */
    command(OP_SET_TX, t, 3);
    tx_active = true;
    tx_start_ms = HAL_GetTick();
    return true;
}
