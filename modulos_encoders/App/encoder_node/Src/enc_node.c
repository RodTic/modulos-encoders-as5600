#include "enc_node.h"
#include "enc_regs.h"
#include "enc_store.h"
#include "as5600.h"
#include "swuart.h"
#include "mb_slave.h"

static I2C_HandleTypeDef *s_i2c;
static enc_regs_t s_regs;
static mb_regs_t s_mb;
static uint8_t s_resp[MB_FRAME_MAX];      /* swuart_send lo lee en segundo plano */
static uint8_t s_running;
static uint32_t s_last_tick, s_hold_until;
static uint8_t s_i2c_fails, s_status_div;

static void i2c_fail(void)
{
    enc_regs_set_flag(&s_regs, ENC_STATUS_I2C_ERROR, 1);
    if (++s_i2c_fails >= 3) {
        s_i2c_fails = 0;
        HAL_I2C_DeInit(s_i2c);
        HAL_I2C_Init(s_i2c);
        s_hold_until = HAL_GetTick() + ENC_I2C_RETRY_MS;
    }
}

static void sample(void)
{
    uint16_t raw;
    if (as5600_read_raw(s_i2c, &raw) != 0) { i2c_fail(); return; }
    enc_regs_update(&s_regs, raw);

    if (++s_status_div >= ENC_STATUS_EVERY_N) {
        uint8_t st;
        s_status_div = 0;
        if (as5600_read_status(s_i2c, &st) != 0) { i2c_fail(); return; }
        enc_regs_set_magnet(&s_regs, st);
    }
    s_i2c_fails = 0;
    enc_regs_set_flag(&s_regs, ENC_STATUS_I2C_ERROR, 0);
}

static void after_reply_actions(void)
{
    if (enc_regs_take_actions(&s_regs, ENC_ACT_ID))
        enc_regs_apply_id(&s_regs);

    if (enc_regs_take_actions(&s_regs, ENC_ACT_SAVE)) {
        enc_cfg_t c = { s_regs.id, s_regs.offset, s_regs.direction };
        int rc = enc_store_save(&c);
        enc_regs_set_flag(&s_regs, ENC_STATUS_SAVE_FAIL, rc != 0);
        if (rc == 0)
            enc_regs_set_flag(&s_regs, ENC_STATUS_DEFAULTS, 0);
        swuart_frame_release();           /* lo recibido durante la pausa es basura */
    }

    if (enc_regs_take_actions(&s_regs, ENC_ACT_RESET))
        NVIC_SystemReset();
}

int enc_node_init(I2C_HandleTypeDef *hi2c)
{
    uint16_t uid[6];
    uint32_t w[3] = { HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2() };
    for (int i = 0; i < 3; i++) {
        uid[2 * i]     = (uint16_t)(w[i] >> 16);
        uid[2 * i + 1] = (uint16_t)(w[i] & 0xFFFFu);
    }

    s_i2c = hi2c;
    enc_regs_init(&s_regs, uid);

    enc_cfg_t c;
    if (enc_store_load(&c) == 0) {
        enc_regs_set_cfg(&s_regs, c.id, c.offset, c.direction);
        enc_regs_set_flag(&s_regs, ENC_STATUS_DEFAULTS, 0);
    }
    s_mb = enc_regs_mb(&s_regs);

    if (swuart_init() != 0)
        return -1;

    s_status_div = ENC_STATUS_EVERY_N - 1;   /* la primera muestra tambien lee el estado del iman */
    sample();
    s_last_tick = HAL_GetTick();
    s_running = 1;
    return 0;
}

void enc_node_poll(void)
{
    if (!s_running)
        return;

    const uint8_t *f;
    uint16_t n;
    if (swuart_frame_get(&f, &n)) {
        size_t rl = mb_process((uint8_t)s_regs.id, &s_mb, f, n, s_resp);
        if (enc_regs_take_actions(&s_regs, ENC_ACT_SNAPSHOT)) {
            sample();                      /* lectura fresca justo al terminar la trama */
            enc_regs_snapshot(&s_regs);
        }
        swuart_frame_release();
        if (rl)
            swuart_send(s_resp, (uint16_t)rl);
    }

    if (!swuart_tx_busy())
        after_reply_actions();

    uint32_t now = HAL_GetTick();
    if (now != s_last_tick && (int32_t)(now - s_hold_until) >= 0) {
        s_last_tick = now;
        sample();
    }
}

uint16_t enc_node_angle_cal(void)     { return s_regs.cal; }
uint16_t enc_node_angle_raw(void)     { return s_regs.raw; }
uint16_t enc_node_status(void)        { return s_regs.status; }
uint16_t enc_node_sample_count(void)  { return s_regs.sample_count; }
uint8_t  enc_node_id(void)            { return (uint8_t)s_regs.id; }
