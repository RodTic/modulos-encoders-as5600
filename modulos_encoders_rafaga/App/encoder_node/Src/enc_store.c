#include "enc_store.h"
#include "enc_config.h"
#include "mb_slave.h"          /* mb_crc16 */
#include "stm32f0xx_hal.h"
#include <string.h>

#define REC_MAGIC 0xEC01u
#define REC_HALFWORDS 5u       /* magic, id, offset, direction, crc */

/* Simbolos del linker script de CubeIDE: fin de la imagen en flash. */
extern uint32_t _sidata, _sdata, _edata;

static void pack(uint16_t rec[REC_HALFWORDS], const enc_cfg_t *c)
{
    rec[0] = REC_MAGIC;
    rec[1] = c->id;
    rec[2] = c->offset;
    rec[3] = c->direction;
    rec[4] = mb_crc16((const uint8_t *)rec, 4 * sizeof(uint16_t));
}

int enc_store_load(enc_cfg_t *c)
{
    uint16_t rec[REC_HALFWORDS];
    memcpy(rec, (const void *)ENC_STORE_ADDR, sizeof rec);

    if (rec[0] != REC_MAGIC) return -1;
    if (rec[4] != mb_crc16((const uint8_t *)rec, 4 * sizeof(uint16_t))) return -1;
    if (rec[1] < ENC_ID_MIN || rec[1] > ENC_ID_MAX) return -1;
    if (rec[2] >= ENC_COUNTS_PER_REV || rec[3] > 1u) return -1;

    c->id = rec[1]; c->offset = rec[2]; c->direction = rec[3];
    return 0;
}

int enc_store_save(const enc_cfg_t *c)
{
    uint32_t image_end = (uint32_t)&_sidata + ((uint32_t)&_edata - (uint32_t)&_sdata);
    if (image_end > ENC_STORE_ADDR)
        return -2;

    uint16_t rec[REC_HALFWORDS];
    pack(rec, c);

    FLASH_EraseInitTypeDef e = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .PageAddress = ENC_STORE_ADDR,
        .NbPages = 1,
    };
    uint32_t page_error = 0;
    int rc = 0;

    HAL_FLASH_Unlock();
    if (HAL_FLASHEx_Erase(&e, &page_error) != HAL_OK)
        rc = -1;
    for (uint32_t i = 0; i < REC_HALFWORDS && rc == 0; i++)
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, ENC_STORE_ADDR + 2u * i, rec[i]) != HAL_OK)
            rc = -1;
    HAL_FLASH_Lock();

    if (rc == 0 && memcmp((const void *)ENC_STORE_ADDR, rec, sizeof rec) != 0)
        rc = -1;
    return rc;
}
