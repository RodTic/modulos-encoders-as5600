/* enc_config.h - todos los parametros ajustables del modulo encoder_node.
 * Es el unico archivo que normalmente se toca al integrar en otro proyecto. */
#ifndef ENC_CONFIG_H
#define ENC_CONFIG_H

/* ---- Identidad ---------------------------------------------------------- */
#define ENC_FW_VERSION      0x0110u   /* 1.10 = 1.00 + modo rafaga, se lee en input reg 12 */
#define ENC_DEFAULT_ID      247u      /* ID de una placa sin configurar */
#define ENC_ID_MIN          1u
#define ENC_ID_MAX          247u
#define ENC_COUNTS_PER_REV  4096u     /* AS5600: 12 bits */

/* ---- Bus serie (UART por software en PB9 Tx / PB10 Rx, DE+RE en PA8) ---- */
#define ENC_BAUD            230400u   /* maximo estimado del UART por software (la ISR de
                                         bit cuesta ~100 ciclos de 192). Si hay errores
                                         de CRC bajar a 115200u */
#define ENC_SYSCLK_HZ       44236800u /* HSE 11.0592 MHz x4 (PLL). Debe dividir
                                         exacto por ENC_BAUD */
/* Silencio que el nodo espera para dar por terminada una trama. Modbus estandar
 * pide 1.75 ms fijo (baud > 19200) o 3.5 caracteres (baud <= 19200). A 230400 o mas
 * se usa 500 us: el nodo responde antes y la lectura de 6 nodos va mas rapida. El
 * adaptador USB entrega la trama seguida, asi que no hay pausas internas de ese tamano. */
#define ENC_GAP_US          ((ENC_BAUD >= 230400u) ? 500u : ((ENC_BAUD > 19200u) ? 1750u : (38500000u / ENC_BAUD)))
/* Ciclos de CPU entre el flanco de arranque y el arranque del timer (entrada de la
 * interrupcion ~19 + codigo de swuart_exti_isr ~50 + esperas de flash, medido en el
 * desensamblado). Se resta del primer muestreo para caer a mitad del bit 0. */
#define ENC_RX_ENTRY_COMP_CYCLES 80u

/* 1: swuart.c define TIM16_IRQHandler, TIM17_IRQHandler y EXTI4_15_IRQHandler.
 * 0: el proyecto grande ya los define; que llame a swuart_tim16_isr(),
 * swuart_tim17_isr() y swuart_exti_isr() desde ellos. */
#define ENC_DEFINE_IRQ_HANDLERS 1

/* ---- Modo rafaga (piloto) ------------------------------------------------
 * Un solo broadcast (id 0, FC 06, registro COMMAND = ENC_CMD_BURST) hace que cada nodo con
 * ID 1..ENC_BURST_MAX_ID tome su snapshot y conteste SOLO, sin que el maestro le pregunte, en
 * su turno: el nodo N empieza ENC_BURST_FIRST_US + (N-1) * ENC_BURST_SLOT_US despues de que
 * detecta el fin de la peticion (los 500 us de silencio ya cuentan). La respuesta es la misma
 * trama FC 04 que daria leer los registros 13..15 (11 bytes), asi que el maestro la lee igual.
 * Los nodos con otro ID (por ejemplo una placa nueva, 247) no participan: siguen contestando
 * Modbus normal. */
#define ENC_BURST_MAX_ID        6u
#define ENC_BURST_FIRST_US      2300u   /* el snapshot necesita ~2.1 ms (medido); de ahi sale este margen */
#define ENC_BURST_SLOT_US       650u    /* la trama dura 477 us a 230400; 650 deja ~170 us de guarda */
#define ENC_BURST_LATE_TOL_US   150u    /* si el nodo llega mas tarde que esto a su turno, no contesta */
#define ENC_BURST_FRAME_BYTES   11u

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
