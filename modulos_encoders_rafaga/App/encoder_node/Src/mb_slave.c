#include "mb_slave.h"

uint16_t mb_crc16(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFFu;
    while (len--) {
        crc ^= *buf++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0xA001u) : (uint16_t)(crc >> 1);
    }
    return crc;
}

static uint16_t be16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }

static size_t finish(uint8_t *resp, size_t n)
{
    uint16_t crc = mb_crc16(resp, n);
    resp[n++] = (uint8_t)(crc & 0xFFu);
    resp[n++] = (uint8_t)(crc >> 8);
    return n;
}

static size_t exception(uint8_t *resp, uint8_t id, uint8_t fc, uint8_t ex)
{
    resp[0] = id;
    resp[1] = (uint8_t)(fc | 0x80u);
    resp[2] = ex;
    return finish(resp, 3);
}

size_t mb_process(uint8_t my_id, const mb_regs_t *r,
                  const uint8_t *req, size_t len, uint8_t *resp)
{
    if (len < 4 || len > MB_FRAME_MAX)
        return 0;
    if ((uint16_t)(req[len - 2] | (req[len - 1] << 8)) != mb_crc16(req, len - 2))
        return 0;

    uint8_t id = req[0], fc = req[1];
    int bcast = (id == 0);
    if (!bcast && id != my_id)
        return 0;

    uint8_t ex = MB_EX_NONE;
    size_t n = 0;

    switch (fc) {
    case MB_FC_READ_HOLDING:
    case MB_FC_READ_INPUT: {
        if (bcast) return 0;                 /* leer por broadcast no tiene sentido */
        if (len != 8) { ex = MB_EX_ILLEGAL_VALUE; break; }
        uint16_t addr = be16(req + 2), qty = be16(req + 4);
        if (qty < 1 || qty > 125) { ex = MB_EX_ILLEGAL_VALUE; break; }
        if ((uint32_t)addr + qty > 0x10000u) { ex = MB_EX_ILLEGAL_ADDRESS; break; }
        resp[0] = id; resp[1] = fc; resp[2] = (uint8_t)(qty * 2u);
        n = 3;
        for (uint16_t i = 0; i < qty && ex == MB_EX_NONE; i++) {
            uint16_t v = 0;
            ex = r->read_reg(r->ctx, fc == MB_FC_READ_INPUT, (uint16_t)(addr + i), &v);
            resp[n++] = (uint8_t)(v >> 8);
            resp[n++] = (uint8_t)(v & 0xFFu);
        }
        break;
    }
    case MB_FC_WRITE_SINGLE: {
        if (len != 8) { ex = MB_EX_ILLEGAL_VALUE; break; }
        ex = r->write_reg(r->ctx, be16(req + 2), be16(req + 4), 1);
        if (ex == MB_EX_NONE) {
            for (n = 0; n < 6; n++) resp[n] = req[n];   /* eco */
            resp[0] = id;
        }
        break;
    }
    case MB_FC_WRITE_MULTIPLE: {
        if (len < 9) { ex = MB_EX_ILLEGAL_VALUE; break; }
        uint16_t addr = be16(req + 2), qty = be16(req + 4);
        uint8_t bc = req[6];
        if (qty < 1 || qty > 123 || bc != qty * 2u || len != (size_t)9 + bc) {
            ex = MB_EX_ILLEGAL_VALUE; break;
        }
        if ((uint32_t)addr + qty > 0x10000u) { ex = MB_EX_ILLEGAL_ADDRESS; break; }
        for (uint16_t i = 0; i < qty && ex == MB_EX_NONE; i++)      /* validar todo */
            ex = r->write_reg(r->ctx, (uint16_t)(addr + i), be16(req + 7 + 2u * i), 0);
        for (uint16_t i = 0; i < qty && ex == MB_EX_NONE; i++)      /* y recien aplicar */
            ex = r->write_reg(r->ctx, (uint16_t)(addr + i), be16(req + 7 + 2u * i), 1);
        if (ex == MB_EX_NONE) {
            resp[0] = id; resp[1] = fc;
            resp[2] = req[2]; resp[3] = req[3]; resp[4] = req[4]; resp[5] = req[5];
            n = 6;
        }
        break;
    }
    default:
        ex = MB_EX_ILLEGAL_FUNCTION;
        break;
    }

    if (bcast)
        return 0;
    if (ex != MB_EX_NONE)
        return exception(resp, id, fc, ex);
    return finish(resp, n);
}
