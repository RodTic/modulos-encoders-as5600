#include "swuart.h"
#include "enc_config.h"
#include "mb_slave.h"          /* MB_FRAME_MAX */
#include "stm32f0xx_hal.h"

#define BIT_TICKS     (ENC_SYSCLK_HZ / ENC_BAUD)
#if (ENC_SYSCLK_HZ % ENC_BAUD) != 0
#error "ENC_BAUD no divide exacto a ENC_SYSCLK_HZ"
#endif
#if BIT_TICKS < 96
#error "ENC_BAUD demasiado alto para un UART por software"
#endif

/* Primer muestreo: mitad del bit 0 = 1.5 bits tras el flanco de arranque. */
#define FIRST_SAMPLE_TICKS ((BIT_TICKS * 3u) / 2u - ENC_RX_ENTRY_COMP_CYCLES)

/* TIM17 a ~1 MHz; el gap se redondea hacia arriba. */
#define GAP_PSC       (ENC_SYSCLK_HZ / 1000000u - 1u)
#define GAP_TICKS     (((ENC_GAP_US * (ENC_SYSCLK_HZ / (GAP_PSC + 1u))) + 999999u) / 1000000u)

/* Las ISR de bit tienen que ser rapidas (a 230400 el bit dura 192 ciclos) aunque el proyecto
 * compile con -O0, como el Debug de CubeIDE: a -O0 son ~50 % mas largas y ya no caben. */
#define ISR_RAPIDA __attribute__((optimize("Os")))

#define TX_HIGH()  (GPIOB->BSRR = (1u << 9))
#define TX_LOW()   (GPIOB->BSRR = (1u << (9 + 16)))
#define DE_ON()    (GPIOA->BSRR = (1u << 8))
#define DE_OFF()   (GPIOA->BSRR = (1u << (8 + 16)))
#define RX_LEVEL() ((GPIOB->IDR >> 10) & 1u)
#define RX_EXTI    (1u << 10)

enum { ST_IDLE, ST_RX, ST_TX };

static struct {
    volatile uint8_t  state;
    /* rx */
    uint8_t  rx_buf[MB_FRAME_MAX];
    volatile uint16_t rx_len;
    uint8_t  rx_shift, rx_nbit;
    volatile uint8_t rx_err, frame_ready;
    /* tx */
    const uint8_t *tx_buf;
    uint16_t tx_len, tx_idx;
    uint16_t tx_frame;
    uint8_t  tx_nbits;
} s;

static int clock_ok(void)
{
    /* RM0360: CFGR.SWS=PLL, PLLSRC=HSE/PREDIV, PLLMUL=x4, HPRE=1, PPRE=1; CFGR2.PREDIV=/1 */
    uint32_t cfgr = RCC->CFGR;
    return ((cfgr >> 2) & 3u) == 2u        /* SWS = PLL */
        && (cfgr & (1u << 16))             /* PLLSRC = HSE/PREDIV */
        && ((cfgr >> 18) & 0xFu) == 2u     /* PLLMUL = x4 */
        && ((cfgr >> 4) & 0xFu) == 0u      /* HPRE = /1 */
        && ((cfgr >> 8) & 7u) == 0u        /* PPRE = /1 */
        && (RCC->CFGR2 & 0xFu) == 0u;      /* PREDIV = /1 */
}

int swuart_init(void)
{
    if (!clock_ok())
        return -1;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_TIM16_CLK_ENABLE();
    __HAL_RCC_TIM17_CLK_ENABLE();

    TX_HIGH();
    DE_OFF();
    GPIOB->MODER   = (GPIOB->MODER & ~((3u << 18) | (3u << 20))) | (1u << 18); /* PB9 out, PB10 in */
    GPIOB->OTYPER &= ~(1u << 9);
    GPIOB->OSPEEDR |= (3u << 18);
    GPIOB->PUPDR   = (GPIOB->PUPDR & ~(3u << 20)) | (1u << 20);               /* PB10 pull-up */
    GPIOA->MODER   = (GPIOA->MODER & ~(3u << 16)) | (1u << 16);               /* PA8 out */

    TIM16->PSC = 0;
    TIM16->ARR = BIT_TICKS - 1u;
    TIM16->CR1 = 0;
    TIM16->DIER = 0;
    TIM16->EGR = TIM_EGR_UG;
    TIM16->SR = 0;

    TIM17->PSC = GAP_PSC;
    TIM17->ARR = GAP_TICKS - 1u;
    TIM17->CR1 = TIM_CR1_OPM;
    TIM17->EGR = TIM_EGR_UG;
    TIM17->SR = 0;
    TIM17->DIER = TIM_DIER_UIE;

    SYSCFG->EXTICR[2] = (SYSCFG->EXTICR[2] & ~(0xFu << 8)) | (1u << 8);       /* EXTI10 <- PB10 */
    EXTI->FTSR |= RX_EXTI;
    EXTI->RTSR &= ~RX_EXTI;
    EXTI->PR = RX_EXTI;
    EXTI->IMR |= RX_EXTI;

    s.state = ST_IDLE;
    s.rx_len = 0; s.rx_err = 0; s.frame_ready = 0;

    NVIC_SetPriority(TIM16_IRQn, 0);
    NVIC_SetPriority(EXTI4_15_IRQn, 0);
    NVIC_SetPriority(TIM17_IRQn, 1);
    NVIC_ClearPendingIRQ(TIM16_IRQn);
    NVIC_ClearPendingIRQ(TIM17_IRQn);
    NVIC_ClearPendingIRQ(EXTI4_15_IRQn);
    NVIC_EnableIRQ(TIM16_IRQn);
    NVIC_EnableIRQ(TIM17_IRQn);
    NVIC_EnableIRQ(EXTI4_15_IRQn);
    return 0;
}

int swuart_frame_get(const uint8_t **buf, uint16_t *len)
{
    if (!s.frame_ready)
        return 0;
    __asm volatile ("" ::: "memory");   /* leer rx_buf despues de ver frame_ready */
    if (s.rx_err) {
        swuart_frame_release();
        return 0;
    }
    *buf = s.rx_buf;
    *len = s.rx_len;
    return 1;
}

void swuart_frame_release(void)
{
    s.rx_len = 0;
    s.rx_err = 0;
    s.frame_ready = 0;
}

int swuart_tx_busy(void)
{
    return s.state == ST_TX;
}

int swuart_send(const uint8_t *buf, uint16_t len)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (s.state != ST_IDLE || len == 0) {
        __set_PRIMASK(primask);
        return -1;
    }
    s.state = ST_TX;
    __set_PRIMASK(primask);

    EXTI->IMR &= ~RX_EXTI;
    s.tx_buf = buf; s.tx_len = len; s.tx_idx = 0; s.tx_nbits = 0;
    DE_ON();
    TIM16->ARR = BIT_TICKS - 1u;
    TIM16->CNT = 0;
    TIM16->SR = 0;
    TIM16->DIER = TIM_DIER_UIE;
    TIM16->CR1 |= TIM_CR1_CEN;
    TIM16->EGR = TIM_EGR_UG;          /* primer tick ya: bit de arranque */
    return 0;
}

ISR_RAPIDA static void rx_byte_done(void)
{
    if (s.frame_ready || s.rx_len >= MB_FRAME_MAX)
        s.rx_err = 1;                              /* desbordado: la trama se pierde */
    else
        s.rx_buf[s.rx_len++] = s.rx_shift;
    TIM17->CNT = 0;                                /* reinicia el silencio de fin de trama */
    TIM17->CR1 |= TIM_CR1_CEN;
}

ISR_RAPIDA void swuart_tim16_isr(void)
{
    TIM16->SR = 0;

    if (s.state == ST_RX) {
        uint32_t level = RX_LEVEL();
        if (s.rx_nbit == 0)
            TIM16->ARR = BIT_TICKS - 1u;           /* de ahi en mas, un bit por tick */
        s.rx_shift = (uint8_t)((s.rx_shift >> 1) | (level << 7));
        if (++s.rx_nbit == 8u) {
            /* Ultimo bit de datos: el byte esta completo y se cierra AQUI, no en el bit de
             * parada. El siguiente byte puede empezar justo al terminar la parada (1.5 bits
             * despues de este muestreo) y esta ISR tarda ~0.5 bit a 230400: rearmar el EXTI
             * despues de la parada llegaba tarde y se perdia el flanco de arranque, con lo que
             * todos los bytes desde el segundo salian desalineados. El bit de parada no se
             * comprueba; la CRC de la trama protege. */
            TIM16->CR1 &= ~TIM_CR1_CEN;
            TIM16->DIER = 0;
            rx_byte_done();
            s.state = ST_IDLE;
            EXTI->PR = RX_EXTI;
            EXTI->IMR |= RX_EXTI;
        }
    } else if (s.state == ST_TX) {
        if (s.tx_nbits == 0) {                     /* termino el bit de parada anterior */
            if (s.tx_idx >= s.tx_len) {
                TIM16->CR1 &= ~TIM_CR1_CEN;
                TIM16->DIER = 0;
                DE_OFF();
                s.state = ST_IDLE;
                EXTI->PR = RX_EXTI;
                EXTI->IMR |= RX_EXTI;
                return;
            }
            s.tx_frame = (uint16_t)(0x200u | ((uint16_t)s.tx_buf[s.tx_idx++] << 1)); /* stop|datos|start */
            s.tx_nbits = 10;
        }
        if (s.tx_frame & 1u) TX_HIGH(); else TX_LOW();
        s.tx_frame >>= 1;
        s.tx_nbits--;
    }
}

ISR_RAPIDA void swuart_exti_isr(void)
{
    if (EXTI->PR & RX_EXTI) {
        EXTI->PR = RX_EXTI;
        if (s.state == ST_IDLE) {
            EXTI->IMR &= ~RX_EXTI;
            s.state = ST_RX;
            s.rx_nbit = 0;
            TIM16->ARR = FIRST_SAMPLE_TICKS - 1u;
            TIM16->CNT = 0;
            TIM16->SR = 0;
            TIM16->DIER = TIM_DIER_UIE;
            TIM16->CR1 |= TIM_CR1_CEN;
        }
    }
}

ISR_RAPIDA void swuart_tim17_isr(void)
{
    TIM17->SR = 0;
    if (s.rx_len)
        s.frame_ready = 1;
}

#if ENC_DEFINE_IRQ_HANDLERS
/* Alias, no funciones envoltorio: la interrupcion entra directo a la ISR sin una llamada de mas. */
void TIM16_IRQHandler(void)      __attribute__((alias("swuart_tim16_isr")));
void TIM17_IRQHandler(void)      __attribute__((alias("swuart_tim17_isr")));
void EXTI4_15_IRQHandler(void)   __attribute__((alias("swuart_exti_isr")));
#endif
