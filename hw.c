/*
 * hw.c - timebase, servo PWM, battery ADC and watchdog.
 *
 * Clocks (see main.c): TIM1/TIM2 kernel = 200 MHz, ADC kernel = per_ck
 * (HSI 64 MHz), IWDG on LSI (32 kHz).
 */
#include "hw.h"
#include "board.h"
#include "tvc_config.h"

static TIM_HandleTypeDef htim1;
static ADC_HandleTypeDef hadc1;
static IWDG_HandleTypeDef hiwdg;

/* ---------------------------------------------------------------------
 * Timebase
 * ------------------------------------------------------------------- */
void hw_time_init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM2->CR1 = 0;
    TIM2->PSC = 200 - 1;          /* 200 MHz / 200 = 1 MHz */
    TIM2->ARR = 0xFFFFFFFFu;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 = TIM_CR1_CEN;
}

/* ---------------------------------------------------------------------
 * Servos
 * ------------------------------------------------------------------- */
static const uint32_t servo_channels[4] = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4 };
static volatile uint32_t *const servo_ccr[4] = { &TIM1->CCR1, &TIM1->CCR2, &TIM1->CCR3, &TIM1->CCR4 };

void servo_init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitTypeDef g = { 0 };
    g.Pin = SERVO_CH1_PIN | SERVO_CH2_PIN | SERVO_CH3_PIN | SERVO_CH4_PIN;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLDOWN;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(SERVO_PORT, &g);

    htim1.Instance = SERVO_TIM;
    htim1.Init.Prescaler = 200 - 1;                        /* 1 us tick */
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = (1000000u / TVC_SERVO_PWM_HZ) - 1u;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim1);

    TIM_OC_InitTypeDef oc = { 0 };
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0;                                          /* no pulse until enabled */
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    for (unsigned i = 0; i < 4; i++) {
        HAL_TIM_PWM_ConfigChannel(&htim1, &oc, servo_channels[i]);
        HAL_TIM_PWM_Start(&htim1, servo_channels[i]);      /* also sets MOE */
    }
}

static bool servo_on[4];

void servo_enable(unsigned ch, bool on)
{
    if (ch >= 4) return;
    servo_on[ch] = on;
    if (!on) *servo_ccr[ch] = 0;
}

void servo_write_us(unsigned ch, float us)
{
    if (ch >= 4 || !servo_on[ch]) return;
    if (us < TVC_SERVO_MIN_US) us = TVC_SERVO_MIN_US;
    if (us > TVC_SERVO_MAX_US) us = TVC_SERVO_MAX_US;
    *servo_ccr[ch] = (uint32_t)(us + 0.5f);   /* preloaded, applied next frame */
}

void servo_set_gimbal(float pitch_deg, float yaw_deg)
{
    servo_write_us(TVC_SERVO_CH_PITCH, TVC_SERVO_PITCH_CENTER_US + TVC_SERVO_PITCH_US_PER_DEG * pitch_deg);
    servo_write_us(TVC_SERVO_CH_YAW, TVC_SERVO_YAW_CENTER_US + TVC_SERVO_YAW_US_PER_DEG * yaw_deg);
}

/* ---------------------------------------------------------------------
 * Battery
 * ------------------------------------------------------------------- */
void battery_init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_ADC12_CLK_ENABLE();

    RCC_PeriphCLKInitTypeDef pc = { 0 };
    pc.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    pc.AdcClockSelection = RCC_ADCCLKSOURCE_CLKP;          /* per_ck = HSI 64 MHz */
    HAL_RCCEx_PeriphCLKConfig(&pc);

    GPIO_InitTypeDef g = { 0 };
    g.Pin = VBAT_PIN;
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(VBAT_PORT, &g);

    hadc1.Instance = VBAT_ADC;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;      /* 16 MHz */
    hadc1.Init.Resolution = ADC_RESOLUTION_16B;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
    hadc1.Init.OversamplingMode = DISABLE;
    HAL_ADC_Init(&hadc1);
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

    ADC_ChannelConfTypeDef ch = { 0 };
    ch.Channel = VBAT_ADC_CHANNEL;
    ch.Rank = ADC_REGULAR_RANK_1;
    ch.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
    ch.SingleDiff = ADC_SINGLE_ENDED;
    ch.OffsetNumber = ADC_OFFSET_NONE;
    ch.Offset = 0;
    HAL_ADC_ConfigChannel(&hadc1, &ch);
}

float battery_read_v(void)
{
    if (HAL_ADC_Start(&hadc1) != HAL_OK) return 0.0f;
    if (HAL_ADC_PollForConversion(&hadc1, 2) != HAL_OK) return 0.0f;
    uint32_t raw = HAL_ADC_GetValue(&hadc1);
    return ((float)raw / 65535.0f) * 3.3f / TVC_VBAT_DIVIDER;
}

/* ---------------------------------------------------------------------
 * Watchdog
 * ------------------------------------------------------------------- */
void watchdog_init(void)
{
    hiwdg.Instance = IWDG1;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32;              /* 1 kHz */
    hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
    hiwdg.Init.Reload = 250;                               /* 250 ms */
    HAL_IWDG_Init(&hiwdg);
}

void watchdog_kick(void)
{
    HAL_IWDG_Refresh(&hiwdg);
}
