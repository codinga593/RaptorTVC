/*
 * serial.c - see serial.h. UART registers are driven directly in the IRQ;
 * HAL is only used for GPIO/clock setup and baud configuration.
 */
#include "serial.h"
#include "board.h"

#include <string.h>

/* ---------------------------------------------------------------------
 * Ring buffer
 * ------------------------------------------------------------------- */
void ring_init(ring_t *r, uint8_t *buf, uint32_t size)
{
    r->buf = buf;
    r->mask = size - 1u;
    r->head = 0;
    r->tail = 0;
}

bool ring_put(ring_t *r, uint8_t c)
{
    uint32_t h = r->head;
    if (h - r->tail > r->mask) return false;
    r->buf[h & r->mask] = c;
    __DMB();
    r->head = h + 1u;
    return true;
}

size_t ring_write(ring_t *r, const void *data, size_t n)
{
    const uint8_t *p = data;
    size_t i = 0;
    while (i < n && ring_put(r, p[i])) i++;
    return i;
}

bool ring_get(ring_t *r, uint8_t *c)
{
    uint32_t t = r->tail;
    if (t == r->head) return false;
    *c = r->buf[t & r->mask];
    __DMB();
    r->tail = t + 1u;
    return true;
}

size_t ring_count(const ring_t *r)
{
    return (size_t)(r->head - r->tail);
}

size_t ring_peek_block(const ring_t *r, const uint8_t **p)
{
    uint32_t t = r->tail;
    size_t n = (size_t)(r->head - t);
    size_t to_end = (size_t)(r->mask + 1u - (t & r->mask));
    *p = &r->buf[t & r->mask];
    return n < to_end ? n : to_end;
}

void ring_consume(ring_t *r, size_t n)
{
    __DMB();
    r->tail += (uint32_t)n;
}

/* ---------------------------------------------------------------------
 * UARTs
 * ------------------------------------------------------------------- */
typedef struct {
    UART_HandleTypeDef h;
    ring_t rx, tx;
    uint8_t rx_buf[1024];
    uint8_t tx_buf[2048];
} port_t;

static port_t ports[SERIAL_COUNT];

static void gpio_init(serial_port_t port)
{
    GPIO_InitTypeDef g = { 0 };
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    if (port == SERIAL_GPS) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_UART4_CLK_ENABLE();
        g.Pin = GPS_TX_PIN | GPS_RX_PIN;
        g.Alternate = GPIO_AF8_UART4;
        HAL_GPIO_Init(GPIOA, &g);
    } else {
        __HAL_RCC_GPIOE_CLK_ENABLE();
        __HAL_RCC_UART7_CLK_ENABLE();
        g.Pin = PI_TX_PIN | PI_RX_PIN;
        g.Alternate = GPIO_AF7_UART7;
        HAL_GPIO_Init(GPIOE, &g);
    }
}

void serial_set_baud(serial_port_t port, uint32_t baud)
{
    port_t *p = &ports[port];
    USART_TypeDef *u = p->h.Instance;
    u->CR1 &= ~(USART_CR1_RXNEIE_RXFNEIE | USART_CR1_TXEIE_TXFNFIE);
    p->h.Init.BaudRate = baud;
    p->h.Init.WordLength = UART_WORDLENGTH_8B;
    p->h.Init.StopBits = UART_STOPBITS_1;
    p->h.Init.Parity = UART_PARITY_NONE;
    p->h.Init.Mode = UART_MODE_TX_RX;
    p->h.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    p->h.Init.OverSampling = UART_OVERSAMPLING_16;
    p->h.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    p->h.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    HAL_UART_Init(&p->h);
    u->CR1 |= USART_CR1_RXNEIE_RXFNEIE;
    if (ring_count(&p->tx)) u->CR1 |= USART_CR1_TXEIE_TXFNFIE;
}

void serial_init(serial_port_t port, uint32_t baud)
{
    port_t *p = &ports[port];
    memset(&p->h, 0, sizeof p->h);
    ring_init(&p->rx, p->rx_buf, sizeof p->rx_buf);
    ring_init(&p->tx, p->tx_buf, sizeof p->tx_buf);
    gpio_init(port);
    p->h.Instance = (port == SERIAL_GPS) ? GPS_UART : PI_UART;
    serial_set_baud(port, baud);

    IRQn_Type irq = (port == SERIAL_GPS) ? GPS_UART_IRQn : PI_UART_IRQn;
    HAL_NVIC_SetPriority(irq, 5, 0);
    HAL_NVIC_EnableIRQ(irq);
}

size_t serial_write(serial_port_t port, const void *data, size_t n)
{
    port_t *p = &ports[port];
    size_t w = ring_write(&p->tx, data, n);
    p->h.Instance->CR1 |= USART_CR1_TXEIE_TXFNFIE;
    return w;
}

bool serial_getc(serial_port_t port, uint8_t *c)
{
    return ring_get(&ports[port].rx, c);
}

void serial_irq(serial_port_t port)
{
    port_t *p = &ports[port];
    USART_TypeDef *u = p->h.Instance;
    uint32_t isr = u->ISR;

    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) {
        u->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
    }
    if (isr & USART_ISR_RXNE_RXFNE) {
        ring_put(&p->rx, (uint8_t)u->RDR);    /* drop on overflow */
    }
    if ((u->CR1 & USART_CR1_TXEIE_TXFNFIE) && (isr & USART_ISR_TXE_TXFNF)) {
        uint8_t c;
        if (ring_get(&p->tx, &c)) {
            u->TDR = c;
        } else {
            u->CR1 &= ~USART_CR1_TXEIE_TXFNFIE;
        }
    }
}
