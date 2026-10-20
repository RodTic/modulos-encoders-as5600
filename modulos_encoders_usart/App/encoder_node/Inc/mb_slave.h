/* mb_slave.h - Modbus RTU esclavo, sin dependencias de hardware. */
#ifndef MB_SLAVE_H
#define MB_SLAVE_H

#include <stddef.h>
#include <stdint.h>

#define MB_FRAME_MAX 256u

#define MB_FC_READ_HOLDING   0x03u
#define MB_FC_READ_INPUT     0x04u
#define MB_FC_WRITE_SINGLE   0x06u
#define MB_FC_WRITE_MULTIPLE 0x10u

#define MB_EX_NONE             0u
#define MB_EX_ILLEGAL_FUNCTION 1u
#define MB_EX_ILLEGAL_ADDRESS  2u
#define MB_EX_ILLEGAL_VALUE    3u
#define MB_EX_DEVICE_FAILURE   4u

/* Acceso a los registros. Ambos devuelven 0 o un codigo MB_EX_*.
 * write_reg con commit=0 solo valida; con commit=1 valida y aplica. */
typedef struct {
    uint8_t (*read_reg)(void *ctx, uint8_t is_input, uint16_t addr, uint16_t *value);
    uint8_t (*write_reg)(void *ctx, uint16_t addr, uint16_t value, uint8_t commit);
    void *ctx;
} mb_regs_t;

/* CRC-16 Modbus (poly 0xA001, init 0xFFFF). */
uint16_t mb_crc16(const uint8_t *buf, size_t len);

/* Procesa una trama RTU completa (id..CRC). Escribe la respuesta en resp
 * (minimo MB_FRAME_MAX bytes) y devuelve su largo; 0 = no responder (CRC mala,
 * otro ID, broadcast o trama corta). Soporta FC 03, 04, 06 y 16. */
size_t mb_process(uint8_t my_id, const mb_regs_t *regs,
                  const uint8_t *req, size_t len, uint8_t *resp);

#endif
