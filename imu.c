/*
 * imu.c - ICM-42688-P driver (register bank 0 only).
 */
#include "imu.h"
#include "board.h"
#include "tvc_config.h"

#define REG_DEVICE_CONFIG   0x11
#define REG_INT_CONFIG      0x14
#define REG_TEMP_DATA1      0x1D
#define REG_PWR_MGMT0       0x4E
#define REG_GYRO_CONFIG0    0x4F
#define REG_ACCEL_CONFIG0   0x50
#define REG_INT_CONFIG1     0x64
#define REG_INT_SOURCE0     0x65
#define REG_WHO_AM_I        0x75
#define REG_BANK_SEL        0x76
#define WHO_AM_I_VALUE      0x47

#define GYRO_LSB_PER_DPS    16.4f     /* +-2000 dps */
#define ACCEL_LSB_PER_G     2048.0f   /* +-16 g     */
#define DEG2RAD             0.017453292519943295f

static SPI_HandleTypeDef hspi2;

static inline void cs_low(void)  { IMU_CS_PORT->BSRR = (uint32_t)IMU_CS_PIN << 16; }
static inline void cs_high(void) { IMU_CS_PORT->BSRR = IMU_CS_PIN; }

static bool xfer(uint8_t *buf, uint16_t n)
{
    cs_low();
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi2, buf, buf, n, 2);
    cs_high();
    return st == HAL_OK;
}

static bool write_reg(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg & 0x7F, val };
    return xfer(b, 2);
}

static uint8_t read_reg(uint8_t reg)
{
    uint8_t b[2] = { reg | 0x80, 0 };
    xfer(b, 2);
    return b[1];
}

static void spi_init(void)
{
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef g = { 0 };
    g.Pin = IMU_CS_PIN;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    cs_high();
    HAL_GPIO_Init(IMU_CS_PORT, &g);

    g.Pin = IMU_SCK_PIN | IMU_MISO_PIN | IMU_MOSI_PIN;
    g.Mode = GPIO_MODE_AF_PP;
    g.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &g);

    hspi2.Instance = IMU_SPI;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;          /* mode 3 */
    hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;   /* 100 MHz / 8 = 12.5 MHz */
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
    HAL_SPI_Init(&hspi2);
}

bool imu_init(void)
{
    spi_init();
    HAL_Delay(10);

    write_reg(REG_BANK_SEL, 0);
    write_reg(REG_DEVICE_CONFIG, 0x01);      /* soft reset */
    HAL_Delay(2);
    write_reg(REG_BANK_SEL, 0);
    if (read_reg(REG_WHO_AM_I) != WHO_AM_I_VALUE) return false;

    write_reg(REG_INT_CONFIG, 0x03);         /* INT1 pulsed, push-pull, active high */
    write_reg(REG_INT_CONFIG1, 0x00);        /* 100 us pulse, INT_ASYNC_RESET = 0 (required) */
    write_reg(REG_GYRO_CONFIG0, 0x06);       /* +-2000 dps, 1 kHz */
    write_reg(REG_ACCEL_CONFIG0, 0x06);      /* +-16 g, 1 kHz */
    write_reg(REG_INT_SOURCE0, 0x08);        /* UI data ready -> INT1 */
    write_reg(REG_PWR_MGMT0, 0x0F);          /* gyro + accel low-noise mode */
    HAL_Delay(50);                           /* gyro start-up */

    GPIO_InitTypeDef g = { 0 };
    g.Pin = IMU_INT_PIN;
    g.Mode = GPIO_MODE_IT_RISING;
    g.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(IMU_INT_PORT, &g);
    HAL_NVIC_SetPriority(IMU_INT_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(IMU_INT_IRQn);
    return true;
}

static inline int16_t be16(const uint8_t *p) { return (int16_t)((p[0] << 8) | p[1]); }

bool imu_read(imu_sample_t *s)
{
    uint8_t b[1 + 14] = { REG_TEMP_DATA1 | 0x80 };
    if (!xfer(b, sizeof b)) return false;
    const uint8_t *d = &b[1];
    s->temp_c = (float)be16(&d[0]) / 132.48f + 25.0f;
    for (int i = 0; i < 3; i++) {
        s->accel[i] = (float)be16(&d[2 + 2 * i]) * ((float)TVC_GRAVITY_MPS2 / ACCEL_LSB_PER_G);
        s->gyro[i] = (float)be16(&d[8 + 2 * i]) * (DEG2RAD / GYRO_LSB_PER_DPS);
    }
    /* A disconnected sensor reads all ones or all zeros. */
    return !(d[2] == 0xFF && d[3] == 0xFF && d[8] == 0xFF && d[9] == 0xFF);
}
