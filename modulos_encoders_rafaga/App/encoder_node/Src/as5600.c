#include "as5600.h"
#include "enc_config.h"

#define AS5600_REG_STATUS  0x0Bu
#define AS5600_REG_RAW_ANGLE 0x0Cu

int as5600_read_raw(I2C_HandleTypeDef *h, uint16_t *raw)
{
    uint8_t b[2];
    if (HAL_I2C_Mem_Read(h, ENC_AS5600_ADDR << 1, AS5600_REG_RAW_ANGLE,
                         I2C_MEMADD_SIZE_8BIT, b, 2, ENC_I2C_TIMEOUT_MS) != HAL_OK)
        return -1;
    *raw = (uint16_t)(((b[0] & 0x0Fu) << 8) | b[1]);
    return 0;
}

int as5600_read_status(I2C_HandleTypeDef *h, uint8_t *status)
{
    return HAL_I2C_Mem_Read(h, ENC_AS5600_ADDR << 1, AS5600_REG_STATUS,
                            I2C_MEMADD_SIZE_8BIT, status, 1, ENC_I2C_TIMEOUT_MS) == HAL_OK ? 0 : -1;
}
