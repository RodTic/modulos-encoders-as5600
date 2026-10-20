/* enc_config.h - todos los parametros ajustables del modulo encoder_node.
 * Es el unico archivo que normalmente se toca al integrar en otro proyecto. */
#ifndef ENC_CONFIG_H
#define ENC_CONFIG_H

/* ---- Identidad ---------------------------------------------------------- */
#define ENC_FW_VERSION      0x0100u   /* 1.00, se lee en input reg 12 */
#define ENC_DEFAULT_ID      247u      /* ID de una placa sin configurar */
#define ENC_ID_MIN          1u
#define ENC_ID_MAX          247u
#define ENC_COUNTS_PER_REV  4096u     /* AS5600: 12 bits */

/* ---- Bus serie (UART por software en PB9 Tx / PB10 Rx, DE+RE en PA8) ---- */
#define ENC_BAUD            115200u
#define ENC_SYSCLK_HZ       44236800u /* HSE 11.0592 MHz x4 (PLL). Debe dividir
                                         exacto por ENC_BAUD */
/* Silencio entre tramas Modbus RTU: 1.75 ms fijo si baud > 19200, si no 3.5
 * caracteres de 11 bits. */
#define ENC_GAP_US          ((ENC_BAUD > 19200u) ? 1750u : (38500000u / ENC_BAUD))
/* Ciclos de CPU que tarda la ISR del flanco de arranque en iniciar el timer.
 * Se resta del primer muestreo. Ajustar solo si hay errores de trama. */
#define ENC_RX_ENTRY_COMP_CYCLES 30u

/* 1: swuart.c define TIM16_IRQHandler, TIM17_IRQHandler y EXTI4_15_IRQHandler.
 * 0: el proyecto grande ya los define; que llame a swuart_tim16_isr(),
 * swuart_tim17_isr() y swuart_exti_isr() desde ellos. */
#define ENC_DEFINE_IRQ_HANDLERS 1

/* ---- Sensor AS5600 ------------------------------------------------------ */
#define ENC_AS5600_ADDR     0x36u     /* direccion I2C de 7 bits */
#define ENC_I2C_TIMEOUT_MS  5u
#define ENC_STATUS_EVERY_N  16u       /* lee STATUS (iman) cada N muestras */
#define ENC_I2C_RETRY_MS    50u       /* espera tras 3 fallos seguidos */

/* ---- Flash: ultima pagina de 1 KB del STM32F030C8 (64 KB) --------------- */
#define ENC_STORE_ADDR      0x0800FC00u

/* ---- Modbus: registros >= esta direccion los atiende el proyecto grande
 * implementando enc_user_reg_read / enc_user_reg_write (ver enc_regs.h) ---- */
#define ENC_USER_REG_BASE   0x0100u

#endif
