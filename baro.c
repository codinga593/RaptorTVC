/*
 * baro.c - MS5611 driver (datasheet first + second order compensation).
 */
#include "baro.h"
#include "board.h"
#include "hw.h"

#include <math.h>

#define MS5611_ADDR      (0x77u << 1)
#define CMD_RESET        0x1E
#define CMD_CONV_D1_4096 0x48
#define CMD_CONV_D2_4096 0x58
#define CMD_ADC_READ     0x00
#define CMD_PROM_READ    0xA0
#define CONV_TIME_US     9500u     /* 9.04 ms max at OSR 4096 */

/* I2C4 timing for 400 kHz from a 100 MHz kernel clock (PRESC=2, SCLDEL=13,
 * SDADEL=3, SCLH=26, SCLL=46): tLOW 1.41 us, tHIGH 0.81 us. */
#define I2C4_TIMING_400K 0x20D31A2Eu

static I2C_HandleTypeDef hi2c4;
static uint16_t prom[8];

typedef enum { ST_IDLE, ST_WAIT_D1, ST_WAIT_D2 } baro_state_t;
static baro_state_t state;
static uint32_t t_start;
static uint32_t d1, d2;
static bool have_d2;

static bool cmd(uint8_t c)
{
    return HAL_I2C_Master_Transmit(&hi2c4, MS5611_ADDR, &c, 1, 3) == HAL_OK;
}

static bool read_adc(uint32_t *v)
{
    uint8_t b[3];
    if (!cmd(CMD_ADC_READ)) return false;
    if (HAL_I2C_Master_Receive(&hi2c4, MS5611_ADDR, b, 3, 3) != HAL_OK) return false;
    *v = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
    return *v != 0;      /* 0 = read before conversion finished */
}

/* CRC-4 from AN520 */
static uint8_t prom_crc4(const uint16_t *p)
{
    uint16_t n[8];
    for (int i = 0; i < 8; i++) n[i] = p[i];
    uint16_t crc_read = n[7] & 0x000F;
    n[7] &= 0xFF00;
    uint16_t rem = 0;
    for (int cnt = 0; cnt < 16; cnt++) {
        rem ^= (cnt & 1) ? (n[cnt >> 1] & 0x00FF) : (n[cnt >> 1] >> 8);
        for (int b = 8; b > 0; b--) {
            rem = (rem & 0x8000) ? (uint16_t)((rem << 1) ^ 0x3000) : (uint16_t)(rem << 1);
        }
    }
    rem = (rem >> 12) & 0x000F;
    return (uint8_t)(rem ^ crc_read);    /* 0 when the CRC matches */
}

bool baro_init(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_I2C4_CLK_ENABLE();

    GPIO_InitTypeDef g = { 0 };
    g.Pin = BARO_SCL_PIN | BARO_SDA_PIN;
    g.Mode = GPIO_MODE_AF_OD;
    g.Pull = GPIO_NOPULL;                 /* R14/R17 4.7k on board */
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Alternate = GPIO_AF4_I2C4;
    HAL_GPIO_Init(GPIOD, &g);

    hi2c4.Instance = BARO_I2C;
    hi2c4.Init.Timing = I2C4_TIMING_400K;
    hi2c4.Init.OwnAddress1 = 0;
    hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c4) != HAL_OK) return false;
    HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE);

    if (!cmd(CMD_RESET)) return false;
    HAL_Delay(4);
    for (int i = 0; i < 8; i++) {
        uint8_t b[2];
        if (!cmd((uint8_t)(CMD_PROM_READ + 2 * i))) return false;
        if (HAL_I2C_Master_Receive(&hi2c4, MS5611_ADDR, b, 2, 3) != HAL_OK) return false;
        prom[i] = (uint16_t)((b[0] << 8) | b[1]);
    }
    if (prom_crc4(prom) != 0) return false;
    state = ST_IDLE;
    have_d2 = false;
    return true;
}

static void compensate(uint32_t D1, uint32_t D2, baro_sample_t *o)
{
    int64_t dT = (int64_t)D2 - ((int64_t)prom[5] << 8);
    int64_t TEMP = 2000 + ((dT * prom[6]) >> 23);
    int64_t OFF = ((int64_t)prom[2] << 16) + (((int64_t)prom[4] * dT) >> 7);
    int64_t SENS = ((int64_t)prom[1] << 15) + (((int64_t)prom[3] * dT) >> 8);

    if (TEMP < 2000) {
        int64_t T2 = (dT * dT) >> 31;
        int64_t t = TEMP - 2000;
        int64_t OFF2 = 5 * t * t / 2;
        int64_t SENS2 = 5 * t * t / 4;
        if (TEMP < -1500) {
            int64_t u = TEMP + 1500;
            OFF2 += 7 * u * u;
            SENS2 += 11 * u * u / 2;
        }
        TEMP -= T2;
        OFF -= OFF2;
        SENS -= SENS2;
    }
    int64_t P = ((((int64_t)D1 * SENS) >> 21) - OFF) >> 15;
    o->pressure_pa = (float)P;          /* 0.01 mbar = 1 Pa */
    o->temp_c = (float)TEMP / 100.0f;
}

bool baro_poll(baro_sample_t *out)
{
    uint32_t now = hw_micros();
    switch (state) {
    case ST_IDLE:
        /* Temperature every other cycle is plenty (it changes slowly). */
        if (!have_d2) {
            if (cmd(CMD_CONV_D2_4096)) { state = ST_WAIT_D2; t_start = now; }
        } else {
            if (cmd(CMD_CONV_D1_4096)) { state = ST_WAIT_D1; t_start = now; }
        }
        return false;

    case ST_WAIT_D2:
        if (now - t_start < CONV_TIME_US) return false;
        state = ST_IDLE;
        if (read_adc(&d2)) have_d2 = true;
        return false;

    case ST_WAIT_D1:
        if (now - t_start < CONV_TIME_US) return false;
        state = ST_IDLE;
        if (!read_adc(&d1)) return false;
        compensate(d1, d2, out);
        out->t_us = t_start + CONV_TIME_US / 2;
        static uint8_t n;
        if (++n >= 10) { n = 0; have_d2 = false; }   /* refresh temperature at ~5 Hz */
        return true;
    }
    return false;
}

float baro_altitude_m(float p_pa, float p0_pa)
{
    return 44330.0f * (1.0f - powf(p_pa / p0_pa, 0.190295f));
}
