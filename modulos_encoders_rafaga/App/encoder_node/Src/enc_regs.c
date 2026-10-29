#include "enc_regs.h"

uint16_t enc_calibrate(uint16_t raw, uint16_t direction, uint16_t offset)
{
    uint16_t d = direction ? (uint16_t)((ENC_COUNTS_PER_REV - raw) & (ENC_COUNTS_PER_REV - 1u)) : raw;
    return (uint16_t)((d + ENC_COUNTS_PER_REV - offset) & (ENC_COUNTS_PER_REV - 1u));
}

static void recompute(enc_regs_t *r)
{
    r->cal = enc_calibrate(r->raw, r->direction, r->offset);
}

void enc_regs_init(enc_regs_t *r, const uint16_t uid[6])
{
    *r = (enc_regs_t){0};
    r->id = ENC_DEFAULT_ID;
    r->status = ENC_STATUS_DEFAULTS;
    for (int i = 0; i < 6; i++) r->uid[i] = uid[i];
}

void enc_regs_set_cfg(enc_regs_t *r, uint16_t id, uint16_t offset, uint16_t direction)
{
    r->id = id;
    r->offset = offset;
    r->direction = direction;
    recompute(r);
}

void enc_regs_update(enc_regs_t *r, uint16_t raw)
{
    r->raw = raw;
    r->sample_count++;
    recompute(r);
}

void enc_regs_set_magnet(enc_regs_t *r, uint8_t s)
{
    /* AS5600 STATUS: bit5 MD, bit4 ML, bit3 MH */
    uint16_t m = (uint16_t)(((s >> 5) & 1u) ? ENC_STATUS_MAGNET : 0u)
               | (uint16_t)(((s >> 4) & 1u) ? ENC_STATUS_TOO_WEAK : 0u)
               | (uint16_t)(((s >> 3) & 1u) ? ENC_STATUS_TOO_STRONG : 0u);
    r->status = (uint16_t)((r->status & ~(ENC_STATUS_MAGNET | ENC_STATUS_TOO_WEAK | ENC_STATUS_TOO_STRONG)) | m);
}

void enc_regs_set_flag(enc_regs_t *r, uint16_t flag, int on)
{
    if (on) r->status |= flag; else r->status &= (uint16_t)~flag;
}

void enc_regs_snapshot(enc_regs_t *r)
{
    r->snap_cal = r->cal;
    r->snap_raw = r->raw;
    r->snap_status = r->status;
    r->snap_seq++;
}

void enc_regs_apply_id(enc_regs_t *r)
{
    if (r->pending_id) { r->id = r->pending_id; r->pending_id = 0; }
}

uint8_t enc_regs_take_actions(enc_regs_t *r, uint8_t mask)
{
    uint8_t a = r->actions & mask;
    r->actions &= (uint8_t)~mask;
    return a;
}

__attribute__((weak)) uint8_t enc_user_reg_read(uint16_t addr, uint8_t is_input, uint16_t *value)
{
    (void)addr; (void)is_input; (void)value;
    return MB_EX_ILLEGAL_ADDRESS;
}

__attribute__((weak)) uint8_t enc_user_reg_write(uint16_t addr, uint16_t value, uint8_t commit)
{
    (void)addr; (void)value; (void)commit;
    return MB_EX_ILLEGAL_ADDRESS;
}

static uint8_t read_reg(void *ctx, uint8_t is_input, uint16_t addr, uint16_t *v)
{
    enc_regs_t *r = ctx;
    if (addr >= ENC_USER_REG_BASE)
        return enc_user_reg_read(addr, is_input, v);
    if (!is_input) {
        switch (addr) {
        case ENC_HREG_OFFSET:    *v = r->offset;    return MB_EX_NONE;
        case ENC_HREG_DIRECTION: *v = r->direction; return MB_EX_NONE;
        case ENC_HREG_SLAVE_ID:  *v = r->id;        return MB_EX_NONE;
        case ENC_HREG_COMMAND:   *v = 0;            return MB_EX_NONE;
        }
        return MB_EX_ILLEGAL_ADDRESS;
    }
    switch (addr) {
    case ENC_IREG_ANGLE_CAL:    *v = r->cal;          return MB_EX_NONE;
    case ENC_IREG_ANGLE_RAW:    *v = r->raw;          return MB_EX_NONE;
    case ENC_IREG_STATUS:       *v = r->status;       return MB_EX_NONE;
    case ENC_IREG_SAMPLE_COUNT: *v = r->sample_count; return MB_EX_NONE;
    case ENC_IREG_SNAP_ANGLE:   *v = r->snap_cal;     return MB_EX_NONE;
    case ENC_IREG_SNAP_SEQ:     *v = r->snap_seq;     return MB_EX_NONE;
    case ENC_IREG_FW_VERSION:   *v = ENC_FW_VERSION;  return MB_EX_NONE;
    case ENC_IREG_SNAP_RAW:     *v = r->snap_raw;     return MB_EX_NONE;
    case ENC_IREG_SNAP_STATUS:  *v = r->snap_status;  return MB_EX_NONE;
    case ENC_IREG_SNAP_SEQ2:    *v = r->snap_seq;     return MB_EX_NONE;
    case ENC_IREG_BURST_LATE:   *v = r->burst_late;   return MB_EX_NONE;
    }
    if (addr >= ENC_IREG_UID0 && addr < ENC_IREG_UID0 + 6u) {
        *v = r->uid[addr - ENC_IREG_UID0];
        return MB_EX_NONE;
    }
    return MB_EX_ILLEGAL_ADDRESS;
}

static void command(enc_regs_t *r, uint16_t cmd)
{
    switch (cmd) {
    case ENC_CMD_SAVE:    r->actions |= ENC_ACT_SAVE; break;
    case ENC_CMD_RESTORE:
        r->offset = 0; r->direction = 0;
        r->pending_id = ENC_DEFAULT_ID; r->actions |= ENC_ACT_ID;
        recompute(r);
        break;
    case ENC_CMD_SET_ZERO:
        r->offset = r->direction
            ? (uint16_t)((ENC_COUNTS_PER_REV - r->raw) & (ENC_COUNTS_PER_REV - 1u))
            : r->raw;
        recompute(r);
        break;
    case ENC_CMD_RESET:    r->actions |= ENC_ACT_RESET; break;
    case ENC_CMD_SNAPSHOT: r->actions |= ENC_ACT_SNAPSHOT; break;
    case ENC_CMD_BURST:    r->actions |= ENC_ACT_SNAPSHOT | ENC_ACT_BURST; break;
    }
}

static uint8_t write_reg(void *ctx, uint16_t addr, uint16_t v, uint8_t commit)
{
    enc_regs_t *r = ctx;
    switch (addr) {
    case ENC_HREG_OFFSET:
        if (v >= ENC_COUNTS_PER_REV) return MB_EX_ILLEGAL_VALUE;
        if (commit) { r->offset = v; recompute(r); }
        return MB_EX_NONE;
    case ENC_HREG_DIRECTION:
        if (v > 1) return MB_EX_ILLEGAL_VALUE;
        if (commit) { r->direction = v; recompute(r); }
        return MB_EX_NONE;
    case ENC_HREG_SLAVE_ID:
        if (v < ENC_ID_MIN || v > ENC_ID_MAX) return MB_EX_ILLEGAL_VALUE;
        if (commit) { r->pending_id = v; r->actions |= ENC_ACT_ID; }
        return MB_EX_NONE;
    case ENC_HREG_COMMAND:
        if (v < ENC_CMD_SAVE || v > ENC_CMD_BURST) return MB_EX_ILLEGAL_VALUE;
        if (commit) command(r, v);
        return MB_EX_NONE;
    }
    if (addr >= ENC_USER_REG_BASE)
        return enc_user_reg_write(addr, v, commit);
    return MB_EX_ILLEGAL_ADDRESS;   /* incluye los input registers (solo lectura) */
}

mb_regs_t enc_regs_mb(enc_regs_t *r)
{
    mb_regs_t m = { read_reg, write_reg, r };
    return m;
}

size_t enc_regs_burst_frame(enc_regs_t *r, uint8_t *resp)
{
    uint8_t rq[8] = { (uint8_t)r->id, MB_FC_READ_INPUT, 0, ENC_IREG_SNAP_RAW, 0, 3, 0, 0 };
    uint16_t crc = mb_crc16(rq, 6);
    rq[6] = (uint8_t)(crc & 0xFFu);
    rq[7] = (uint8_t)(crc >> 8);
    mb_regs_t m = enc_regs_mb(r);
    return mb_process((uint8_t)r->id, &m, rq, 8, resp);
}
