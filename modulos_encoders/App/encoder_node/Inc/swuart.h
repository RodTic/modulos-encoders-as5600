/* swuart.h - UART half-duplex por software para Modbus RTU.
 *   PB9  = Tx (hacia DI del SP3485)      PB10 = Rx (desde RO, con EXTI)
 *   PA8  = DE+RE# (1 = transmitir)
 *   TIM16 = reloj de bit, TIM17 = silencio de fin de trama, EXTI linea 10.
 * 8N1. Cambiar a un USART de hardware = reemplazar solo este archivo. */
#ifndef SWUART_H
#define SWUART_H

#include <stdint.h>

/* 0 = ok; -1 = el reloj no es HSE*4=44.2368 MHz sin divisores (los tiempos
 * saldrian mal), no se inicia nada. */
int swuart_init(void);

/* 1 si hay una trama completa y sin errores (buf/len validos hasta llamar a
 * swuart_frame_release). Las tramas con error de trama o desbordadas se
 * descartan solas. */
int  swuart_frame_get(const uint8_t **buf, uint16_t *len);
void swuart_frame_release(void);

/* Inicia el envio y vuelve de inmediato; buf debe seguir valido hasta que
 * swuart_tx_busy() sea 0. 0 = ok, -1 = ocupado. */
int swuart_send(const uint8_t *buf, uint16_t len);
int swuart_tx_busy(void);

/* ISRs; con ENC_DEFINE_IRQ_HANDLERS=0 llamarlas desde los handlers propios. */
void swuart_tim16_isr(void);
void swuart_tim17_isr(void);
void swuart_exti_isr(void);

#endif
