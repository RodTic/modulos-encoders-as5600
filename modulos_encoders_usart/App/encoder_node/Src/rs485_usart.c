#include "rs485_usart.h"
#include "enc_config.h"
#include "mb_slave.h"          /* MB_FRAME_MAX */
#include "stm32f0xx_hal.h"

/* BRR del USART1 con sobremuestreo x16: fck / baud (redondeado). */
#define BRR_VALUE   ((ENC_SYSCLK_HZ + ENC_BAUD / 2u) / ENC_BAUD)
#if BRR_VALUE < 16u
#error "ENC_BAUD demasiado alto para ENC_SYSCLK_HZ"
#endif

/* TIM17 a ~1 MHz; el silencio de fin de trama se redondea hacia arriba. */
#define GAP_PSC     (ENC_SYSCLK_HZ / 1000000u - 1u)
#define GAP_TICKS   (((ENC_GAP_US * (ENC_SYSCLK_HZ / (GAP_PSC + 1u))) + 999999u) / 1000000u)

#define DE_ON()     (GPIOA->BSRR = (1u << 8))
#define DE_OFF()    (GPIOA->BSRR = (1u << (8 + 16)))
#define RX_ERR_FLAGS (USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NCF)

static struct {
    /* rx */
    uint8_t  rx_buf[MB_FRAME_MAX];
    volatile uint16_t rx_len;
    volatile uint8_t rx_err, frame_ready;
    /* tx */
    const uint8_t *tx_buf;
    uint16_t tx_len, tx_idx;
    volatile uint8_t tx_busy;
} s;

static int clock_ok(void)
{
    /* RM0360: CFGR.SWS=PLL, PLLSRC=HSE/PREDIV, PLLMUL=x4, HPRE=1, PPRE=1; CFGR2.PREDIV=/1.
     * El USART1 toma PCLK por defecto, asi que fck = 44.2368 MHz. */
    uint32_t cfgr = RCC->CFGR;
    return ((cfgr >> 2) & 3u) == 2u        /* SWS = PLL */
        && (cfgr & (1u << 16))             /* PLLSRC = HSE/PREDIV */
        && ((cfgr >> 18) & 0xFu) == 2u     /* PLLMUL = x4 */
        && ((cfgr >> 4) & 0xFu) == 0u      /* HPRE = /1 */
        && ((cfgr >> 8) & 7u) == 0u        /* PPRE = /1 */
        && (RCC->CFGR2 & 0xFu) == 0u;      /* PREDIV = /1 */
}

int rs485_init(void)
{
    if (!clock_ok())
        return -1;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_TIM17_CLK_ENABLE();

    /* PB9 y PB10 comparten red con PA9/PA10 (cables soldados): alta impedancia. */
    GPIOB->MODER |= (3u << 18) | (3u << 20);

    /* PA8 = DE+RE#, sale en recepcion (0). */
    DE_OFF();
    GPIOA->MODER = (GPIOA->MODER & ~(3u << 16)) | (1u << 16);

    /* PA9 = USART1_TX, PA10 = USART1_RX (AF1). Rx con pull-up: reposo en alto
     * mientras el SP3485 tiene RO en alta impedancia (durante nuestra transmision). */
    GPIOA->AFR[1]   = (GPIOA->AFR[1] & ~((0xFu << 4) | (0xFu << 8))) | (1u << 4) | (1u << 8);
    GPIOA->OSPEEDR |= (3u << 18);
    GPIOA->PUPDR    = (GPIOA->PUPDR & ~(3u << 20)) | (1u << 20);
    GPIOA->MODER    = (GPIOA->MODER & ~((3u << 18) | (3u << 20))) | (2u << 18) | (2u << 20);

    /* USART1: 8N1, sobremuestreo x16. */
    USART1->CR1 = 0;
    USART1->CR2 = 0;
    USART1->CR3 = 0;
    USART1->BRR = BRR_VALUE;
    USART1->ICR = RX_ERR_FLAGS | USART_ICR_TCCF;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

    /* TIM17: una cuenta de ENC_GAP_US y se detiene (fin de trama). */
    TIM17->PSC = GAP_PSC;
    TIM17->ARR = GAP_TICKS - 1u;
    TIM17->CR1 = TIM_CR1_OPM;
    TIM17->EGR = TIM_EGR_UG;
    TIM17->SR = 0;
    TIM17->DIER = TIM_DIER_UIE;

    s.rx_len = 0; s.rx_err = 0; s.frame_ready = 0; s.tx_busy = 0;

    NVIC_SetPriority(USART1_IRQn, 0);
    NVIC_SetPriority(TIM17_IRQn, 1);
    NVIC_ClearPendingIRQ(USART1_IRQn);
    NVIC_ClearPendingIRQ(TIM17_IRQn);
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(TIM17_IRQn);
    return 0;
}

int rs485_frame_get(const uint8_t **buf, uint16_t *len)
{
    if (!s.frame_ready)
        return 0;
    __asm volatile ("" ::: "memory");   /* leer rx_buf despues de ver frame_ready */
    if (s.rx_err) {
        rs485_frame_release();
        return 0;
    }
    *buf = s.rx_buf;
    *len = s.rx_len;
    return 1;
}

void rs485_frame_release(void)
{
    s.rx_len = 0;
    s.rx_err = 0;
    s.frame_ready = 0;
}

int rs485_tx_busy(void)
{
    return s.tx_busy;
}

int rs485_send(const uint8_t *buf, uint16_t len)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (s.tx_busy || len == 0) {
        __set_PRIMASK(primask);
        return -1;
    }
    s.tx_busy = 1;
    __set_PRIMASK(primask);

    s.tx_buf = buf; s.tx_len = len; s.tx_idx = 0;
    DE_ON();
    /* Sin recepcion mientras se transmite; TXE dispara de inmediato y la ISR carga los bytes. */
    USART1->CR1 = (USART1->CR1 & ~USART_CR1_RXNEIE) | USART_CR1_TXEIE;
    return 0;
}

static void rx_restart_gap(void)
{
    TIM17->CNT = 0;                       /* cada byte reinicia el silencio de fin de trama */
    TIM17->CR1 |= TIM_CR1_CEN;
}

void rs485_usart1_isr(void)
{
    uint32_t isr = USART1->ISR;
    uint32_t cr1 = USART1->CR1;
    uint8_t err = 0;

    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) {
        USART1->ICR = RX_ERR_FLAGS;
        err = 1;
    }

    if (cr1 & USART_CR1_RXNEIE) {
        if (isr & USART_ISR_RXNE) {
            uint8_t b = (uint8_t)USART1->RDR;            /* leer RDR limpia RXNE */
            if (err || s.frame_ready || s.rx_len >= MB_FRAME_MAX)
                s.rx_err = 1;                            /* error o desbordado: la trama se pierde */
            if (!s.frame_ready && s.rx_len < MB_FRAME_MAX)
                s.rx_buf[s.rx_len++] = b;
            rx_restart_gap();
        } else if (err) {                                /* overrun sin byte nuevo */
            s.rx_err = 1;
            rx_restart_gap();
        }
    }

    if ((cr1 & USART_CR1_TXEIE) && (isr & USART_ISR_TXE)) {
        if (s.tx_idx < s.tx_len)
            USART1->TDR = s.tx_buf[s.tx_idx++];
        else                                             /* ultimo byte cargado: esperar a que salga */
            USART1->CR1 = (USART1->CR1 & ~USART_CR1_TXEIE) | USART_CR1_TCIE;
    }

    if ((cr1 & USART_CR1_TCIE) && (isr & USART_ISR_TC)) {
        USART1->ICR = USART_ICR_TCCF;
        DE_OFF();                                        /* ya salio el bit de parada */
        (void)USART1->RDR;                               /* descartar lo que haya quedado */
        USART1->ICR = RX_ERR_FLAGS;
        USART1->CR1 = (USART1->CR1 & ~USART_CR1_TCIE) | USART_CR1_RXNEIE;
        s.tx_busy = 0;
    }
}

void rs485_tim17_isr(void)
{
    TIM17->SR = 0;
    if (s.rx_len || s.rx_err)
        s.frame_ready = 1;
}

#if ENC_DEFINE_IRQ_HANDLERS
void USART1_IRQHandler(void)  { rs485_usart1_isr(); }
void TIM17_IRQHandler(void)   { rs485_tim17_isr(); }
#endif
