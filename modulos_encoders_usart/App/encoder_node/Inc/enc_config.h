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

/* ---- Bus serie (USART1 de hardware en PA9 Tx / PA10 Rx, DE+RE en PA8) ---- */
#define ENC_BAUD            921600u   /* exacto en el STM32 (BRR = 48) y en el adaptador USB.
                                         Con el USART de hardware el limite es el cable:
                                         ver README para subir a 1843200 o bajar si hay errores */
#define ENC_SYSCLK_HZ       44236800u /* HSE 11.0592 MHz x4 (PLL). Con otro reloj,
                                         cambiar este valor al real (el baudrate
                                         sale de dividir este valor por ENC_BAUD) */
/* Silencio que el nodo espera para dar por terminada una trama. Modbus estandar
 * pide 1.75 ms fijo (baud > 19200) o 3.5 caracteres (baud <= 19200). A 230400 o mas
 * se usa 500 us: el nodo responde antes y la lectura de 6 nodos va mas rapida. El
 * adaptador USB entrega la trama seguida, asi que no hay pausas internas de ese tamano. */
#define ENC_GAP_US          ((ENC_BAUD >= 230400u) ? 500u : ((ENC_BAUD > 19200u) ? 1750u : (38500000u / ENC_BAUD)))

/* 1: rs485_usart.c define USART1_IRQHandler y TIM17_IRQHandler.
 * 0: el proyecto grande ya los define; que llame a rs485_usart1_isr() y
 * rs485_tim17_isr() desde ellos. */
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
