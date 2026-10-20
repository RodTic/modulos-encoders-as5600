/* as5600.h - driver minimo del AS5600 por I2C (HAL). */
#ifndef AS5600_H
#define AS5600_H

#include <stdint.h>
#include "stm32f0xx_hal.h"

/* Ambas devuelven 0 si OK, -1 si falla el I2C. */
int as5600_read_raw(I2C_HandleTypeDef *h, uint16_t *raw);     /* 0..4095, reg 0x0C/0x0D */
int as5600_read_status(I2C_HandleTypeDef *h, uint8_t *status); /* reg 0x0B: MD bit5, ML bit4, MH bit3 */

#endif
