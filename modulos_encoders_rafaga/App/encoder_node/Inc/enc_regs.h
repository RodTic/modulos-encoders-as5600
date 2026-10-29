/* enc_regs.h - mapa de registros Modbus y calibracion. Sin hardware. */
#ifndef ENC_REGS_H
#define ENC_REGS_H

#include <stdint.h>
#include "enc_config.h"
#include "mb_slave.h"

/* Holding registers (FC 03 leer; FC 06 / FC 16 escribir) */
#define ENC_HREG_OFFSET     0u   /* 0..4095 cuentas */
#define ENC_HREG_DIRECTION  1u   /* 0 normal, 1 invertido */
#define ENC_HREG_SLAVE_ID   2u   /* 1..247, se aplica tras la respuesta */
#define ENC_HREG_COMMAND    3u   /* ENC_CMD_*, siempre se lee como 0 */

/* Input registers (FC 04, solo lectura) */
#define ENC_IREG_ANGLE_CAL     0u   /* angulo calibrado 0..4095 */
#define ENC_IREG_ANGLE_RAW     1u
#define ENC_IREG_STATUS        2u   /* ENC_STATUS_* */
#define ENC_IREG_SAMPLE_COUNT  3u   /* sube en cada lectura I2C buena */
#define ENC_IREG_SNAP_ANGLE    4u   /* angulo congelado por el snapshot */
#define ENC_IREG_SNAP_SEQ      5u   /* numero de snapshot */
#define ENC_IREG_UID0          6u   /* 6..11: UID de 96 bits del chip */
#define ENC_IREG_FW_VERSION    12u
/* Bloque de 3 registros (13..15) para leer de una vez, tras un snapshot, el angulo
 * CRUDO (sin offset ni sentido), el estado y el numero de snapshot. */
#define ENC_IREG_SNAP_RAW      13u
#define ENC_IREG_SNAP_STATUS   14u
#define ENC_IREG_SNAP_SEQ2     15u  /* copia de SNAP_SEQ (5) */
#define ENC_IREG_BURST_LATE    16u  /* veces que el nodo llego tarde a su turno de la rafaga (satura en 65535) */

#define ENC_CMD_SAVE      1u   /* guarda id/offset/sentido en flash */
#define ENC_CMD_RESTORE   2u   /* offset=0, sentido=0, id=247 (en RAM; luego SAVE) */
#define ENC_CMD_SET_ZERO  3u   /* offset := posicion actual (la lectura pasa a 0) */
#define ENC_CMD_RESET     4u
#define ENC_CMD_SNAPSHOT  5u   /* usar por broadcast (id 0) para sincronizar */
#define ENC_CMD_BURST     6u   /* por broadcast: snapshot + cada nodo contesta en su turno (ver enc_config.h) */

#define ENC_STATUS_MAGNET    0x01u  /* AS5600 detecta iman */
#define ENC_STATUS_TOO_WEAK  0x02u
#define ENC_STATUS_TOO_STRONG 0x04u
#define ENC_STATUS_I2C_ERROR 0x08u
#define ENC_STATUS_DEFAULTS  0x10u  /* sin config valida en flash: valores por defecto */
#define ENC_STATUS_SAVE_FAIL 0x20u  /* el ultimo SAVE fallo */

/* Acciones que enc_regs pide al nodo (el nodo las ejecuta) */
#define ENC_ACT_SNAPSHOT 0x01u   /* antes de responder */
#define ENC_ACT_SAVE     0x02u   /* despues de responder */
#define ENC_ACT_RESET    0x04u   /* despues de responder */
#define ENC_ACT_ID       0x08u   /* despues de responder */
#define ENC_ACT_BURST    0x10u   /* tras el snapshot, contestar en el turno (solo si llego por broadcast) */

typedef struct {
    uint16_t offset, direction, id, pending_id;
    uint16_t raw, cal, status, sample_count;
    uint16_t snap_cal, snap_seq, snap_raw, snap_status;
    uint16_t burst_late;
    uint16_t uid[6];
    uint8_t  actions;
} enc_regs_t;

/* cal = ((direction ? 4096 - raw : raw) - offset) mod 4096 */
uint16_t enc_calibrate(uint16_t raw, uint16_t direction, uint16_t offset);

void enc_regs_init(enc_regs_t *r, const uint16_t uid[6]);
void enc_regs_set_cfg(enc_regs_t *r, uint16_t id, uint16_t offset, uint16_t direction);
void enc_regs_update(enc_regs_t *r, uint16_t raw);          /* muestra nueva */
void enc_regs_set_magnet(enc_regs_t *r, uint8_t as5600_status);
void enc_regs_set_flag(enc_regs_t *r, uint16_t flag, int on);
void enc_regs_snapshot(enc_regs_t *r);
void enc_regs_apply_id(enc_regs_t *r);                       /* pending_id -> id */
uint8_t enc_regs_take_actions(enc_regs_t *r, uint8_t mask);  /* devuelve y limpia */
mb_regs_t enc_regs_mb(enc_regs_t *r);

/* Trama de la rafaga: id, 04, 06, snap_raw, snap_status, snap_seq, CRC (11 bytes). Es exactamente
 * la respuesta FC 04 a "leer los registros 13..15", asi que el maestro la lee igual. */
size_t enc_regs_burst_frame(enc_regs_t *r, uint8_t *resp);

/* Ganchos para el proyecto grande: registros >= ENC_USER_REG_BASE.
 * Definiciones debiles por defecto: devuelven MB_EX_ILLEGAL_ADDRESS.
 * write: commit=0 solo valida, commit=1 aplica. */
uint8_t enc_user_reg_read(uint16_t addr, uint8_t is_input, uint16_t *value);
uint8_t enc_user_reg_write(uint16_t addr, uint16_t value, uint8_t commit);

#endif
