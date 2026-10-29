/* swuart.h - UART half-duplex por software para Modbus RTU.
 *   PB9  = Tx (hacia DI del SP3485)      PB10 = Rx (desde RO, con EXTI)
 *   PA8  = DE+RE# (1 = transmitir)
 *   TIM16 = reloj de bit, TIM17 = silencio de fin de trama, EXTI linea 10.
 * 8N1. Cambiar a un USART de hardware = reemplazar solo este archivo. */
#ifndef SWUART_H
#define SWUART_H

#include <stdint.h>
#include "enc_config.h"

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

/* Reloj libre de 16 bits (TIM14, SYSCLK/44 = ~1 MHz, vuelve a 0 cada ~65 ms) para programar el
 * turno de la rafaga. Los intervalos se comparan con resta en int16_t, asi que valen hasta ~32 ms. */
#define SWUART_CLOCK_DIV          44u
#define SWUART_US_TO_TICKS(us)    (((us) * (ENC_SYSCLK_HZ / SWUART_CLOCK_DIV / 1000u)) / 1000u)
uint16_t swuart_clock(void);
/* Valor del reloj cuando se dio por terminada la trama que devuelve swuart_frame_get (fin del
 * ultimo byte + el silencio de fin de trama). Es la referencia comun de todos los nodos. */
uint16_t swuart_frame_clock(void);

/* ISRs; con ENC_DEFINE_IRQ_HANDLERS=0 llamarlas desde los handlers propios. */
void swuart_tim16_isr(void);
void swuart_tim17_isr(void);
void swuart_exti_isr(void);

#endif
