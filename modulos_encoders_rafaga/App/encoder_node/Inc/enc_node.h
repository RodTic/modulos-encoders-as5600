/* enc_node.h - API publica del modulo encoder_node.
 *
 *   enc_node_init(&hi2c1);   // una vez, antes del bucle principal
 *   for (;;) { enc_node_poll(); ... }   // lo demas de tu aplicacion
 *
 * enc_node_poll() no usa HAL_Delay; bloquea solo durante una lectura I2C
 * (~0.3 ms) y durante un SAVE a flash (~40 ms). */
#ifndef ENC_NODE_H
#define ENC_NODE_H

#include <stdint.h>
#include "stm32f0xx_hal.h"

/* 0 = ok; -1 = reloj mal configurado (ver swuart_init): el nodo no arranca. */
int enc_node_init(I2C_HandleTypeDef *hi2c);
void enc_node_poll(void);

/* Datos para el resto de la aplicacion (0..4095 por vuelta). */
uint16_t enc_node_angle_cal(void);
uint16_t enc_node_angle_raw(void);
uint16_t enc_node_status(void);        /* ENC_STATUS_* de enc_regs.h */
uint16_t enc_node_sample_count(void);  /* cambia con cada lectura nueva */
uint8_t  enc_node_id(void);

#endif
