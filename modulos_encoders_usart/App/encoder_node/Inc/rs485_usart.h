/* rs485_usart.h - puerto RS-485 half-duplex con el USART1 de hardware.
 *   PA9  = Tx (USART1_TX, AF1)  -> soldar un cable de PA9 (pin 30) a la red Tx
 *   PA10 = Rx (USART1_RX, AF1)  -> soldar un cable de PA10 (pin 31) a la red Rx
 *   PA8  = DE+RE# (1 = transmitir), ya cableado en la placa
 *   PB9 y PB10 comparten esas redes: quedan en alta impedancia (analogico).
 *   TIM17 = silencio de fin de trama. 8N1. */
#ifndef RS485_USART_H
#define RS485_USART_H

#include <stdint.h>

/* 0 = ok; -1 = el reloj no es HSE*4=44.2368 MHz sin divisores, no se inicia nada. */
int rs485_init(void);

/* 1 si hay una trama completa y sin errores (buf/len validos hasta llamar a
 * rs485_frame_release). Las tramas con error de trama, ruido o desbordadas se
 * descartan solas. */
int  rs485_frame_get(const uint8_t **buf, uint16_t *len);
void rs485_frame_release(void);

/* Inicia el envio y vuelve de inmediato; buf debe seguir valido hasta que
 * rs485_tx_busy() sea 0. 0 = ok, -1 = ocupado. */
int rs485_send(const uint8_t *buf, uint16_t len);
int rs485_tx_busy(void);

/* ISRs; con ENC_DEFINE_IRQ_HANDLERS=0 llamarlas desde los handlers propios
 * (USART1_IRQHandler y TIM17_IRQHandler). */
void rs485_usart1_isr(void);
void rs485_tim17_isr(void);

#endif
