/* Test en PC del protocolo y la calibracion (sin placa).
 *   gcc -Wall -Wextra -IApp/encoder_node/Inc tools/test_mb_rtu.c \
 *       App/encoder_node/Src/mb_slave.c App/encoder_node/Src/enc_regs.c -o tools/test_mb_rtu.exe
 *   tools/test_mb_rtu.exe                                                      */
#include <stdio.h>
#include <string.h>
#include "enc_regs.h"

static int failures, checks;
#define CHECK(c) do { checks++; if (!(c)) { failures++; \
    printf("FALLA linea %d: %s\n", __LINE__, #c); } } while (0)

static enc_regs_t regs;

/* Arma id+fc+datos, le agrega la CRC y llama al esclavo. Devuelve el largo de la respuesta. */
static size_t xfer(uint8_t id, const uint8_t *pdu, size_t n, uint8_t *resp)
{
    uint8_t req[MB_FRAME_MAX];
    req[0] = id;
    memcpy(req + 1, pdu, n);
    uint16_t crc = mb_crc16(req, n + 1);
    req[n + 1] = (uint8_t)(crc & 0xFF);
    req[n + 2] = (uint8_t)(crc >> 8);
    mb_regs_t m = enc_regs_mb(&regs);
    return mb_process((uint8_t)regs.id, &m, req, n + 3, resp);
}

static int resp_ok(const uint8_t *r, size_t n)
{
    return n >= 4 && (uint16_t)(r[n - 2] | (r[n - 1] << 8)) == mb_crc16(r, n - 2);
}

static void fresh(void)
{
    static const uint16_t uid[6] = { 1, 2, 3, 4, 5, 6 };
    enc_regs_init(&regs, uid);
    regs.id = 1;
}

int main(void)
{
    uint8_t r[MB_FRAME_MAX];
    size_t n;

    /* --- CRC: vectores clasicos, calculados tambien con una implementacion independiente --- */
    { const uint8_t v[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x0A}; CHECK(mb_crc16(v, 6) == 0xCDC5); }
    { const uint8_t v[] = {0x01, 0x06, 0x00, 0x01, 0x00, 0x03}; CHECK(mb_crc16(v, 6) == 0x0B98); }
    { const uint8_t v[] = {0xF7, 0x03, 0x00, 0x00, 0x00, 0x04}; CHECK(mb_crc16(v, 6) == 0x9F50); }

    /* --- calibracion --- */
    CHECK(enc_calibrate(100, 0, 100) == 0);
    CHECK(enc_calibrate(100, 0, 101) == 4095);          /* wrap */
    CHECK(enc_calibrate(0, 0, 0) == 0);
    CHECK(enc_calibrate(100, 1, 0) == 3996);            /* invertido */
    CHECK(enc_calibrate(0, 1, 0) == 0);                 /* 4096-0 se enrolla a 0 */
    CHECK(enc_calibrate(100, 1, 3996) == 0);
    CHECK(enc_calibrate(4095, 0, 4095) == 0);

    /* --- FC06: escribir OFFSET (el requisito: id, funcion, valor) --- */
    fresh();
    { const uint8_t p[] = {0x06, 0x00, ENC_HREG_OFFSET, 0x01, 0x2C};            /* offset=300 */
      n = xfer(1, p, sizeof p, r);
      CHECK(n == 8 && resp_ok(r, n) && memcmp(r, "\x01\x06\x00\x00\x01\x2C", 6) == 0);
      CHECK(regs.offset == 300); }

    /* offset fuera de rango -> excepcion 03, y no cambia */
    { const uint8_t p[] = {0x06, 0x00, ENC_HREG_OFFSET, 0x10, 0x00};            /* 4096 */
      n = xfer(1, p, sizeof p, r);
      CHECK(n == 5 && resp_ok(r, n) && r[1] == 0x86 && r[2] == MB_EX_ILLEGAL_VALUE);
      CHECK(regs.offset == 300); }

    /* sentido */
    { const uint8_t p[] = {0x06, 0x00, ENC_HREG_DIRECTION, 0x00, 0x01};
      n = xfer(1, p, sizeof p, r);
      CHECK(n == 8 && regs.direction == 1); }
    { const uint8_t p[] = {0x06, 0x00, ENC_HREG_DIRECTION, 0x00, 0x02};
      n = xfer(1, p, sizeof p, r);
      CHECK(n == 5 && r[2] == MB_EX_ILLEGAL_VALUE && regs.direction == 1); }

    /* --- FC03 leer holding (offset, sentido, id, comando) --- */
    { const uint8_t p[] = {0x03, 0x00, 0x00, 0x00, 0x04};
      n = xfer(1, p, sizeof p, r);
      CHECK(n == 5 + 8 && resp_ok(r, n) && r[2] == 8);
      CHECK(r[3] == 0x01 && r[4] == 0x2C);              /* offset 300 */
      CHECK(r[5] == 0x00 && r[6] == 0x01);              /* sentido 1 */
      CHECK(r[7] == 0x00 && r[8] == 0x01);              /* id 1 */
      CHECK(r[9] == 0x00 && r[10] == 0x00); }           /* comando siempre 0 */

    /* --- FC04: angulo, uid, version --- */
    fresh();
    enc_regs_update(&regs, 1234);
    { const uint8_t p[] = {0x04, 0x00, 0x00, 0x00, 0x0D};                       /* input 0..12 */
      n = xfer(1, p, sizeof p, r);
      CHECK(n == 5 + 26 && resp_ok(r, n));
      CHECK(r[3] == (1234 >> 8) && r[4] == (1234 & 0xFF));     /* cal */
      CHECK(r[5] == (1234 >> 8) && r[6] == (1234 & 0xFF));     /* raw */
      CHECK(r[9] == 0 && r[10] == 1);                           /* sample_count */
      CHECK(r[15] == 0 && r[16] == 1 && r[25] == 0 && r[26] == 6);  /* uid 1..6 */
      CHECK(r[27] == 0x01 && r[28] == 0x00); }                  /* fw 0x0100 */

    /* --- excepciones --- */
    { const uint8_t p[] = {0x07, 0x00, 0x00};                                   /* FC desconocido */
      n = xfer(1, p, sizeof p, r);
      CHECK(n == 5 && r[1] == 0x87 && r[2] == MB_EX_ILLEGAL_FUNCTION); }
    { const uint8_t p[] = {0x03, 0x00, 0x04, 0x00, 0x01};                       /* holding 4 no existe */
      n = xfer(1, p, sizeof p, r);
      CHECK(n == 5 && r[1] == 0x83 && r[2] == MB_EX_ILLEGAL_ADDRESS); }
    { const uint8_t p[] = {0x04, 0x00, 0x10, 0x00, 0x01};                       /* input 16 no existe */
      n = xfer(1, p, sizeof p, r);
      CHECK(n == 5 && r[1] == 0x84 && r[2] == MB_EX_ILLEGAL_ADDRESS); }
    { const uint8_t p[] = {0x04, 0x00, 0x0F, 0x00, 0x02};                       /* 15..16: el 16 no existe */
      n = xfer(1, p, sizeof p, r);
      CHECK(n == 5 && r[2] == MB_EX_ILLEGAL_ADDRESS); }
    { const uint8_t p[] = {0x04, 0x00, 0x00, 0x00, 0x00};                       /* cantidad 0 */
      n = xfer(1, p, sizeof p, r);
      CHECK(n == 5 && r[2] == MB_EX_ILLEGAL_VALUE); }
    { const uint8_t p[] = {0x04, 0x00, 0x00, 0x00, 0x7E};                       /* cantidad 126 */
      n = xfer(1, p, sizeof p, r);
      CHECK(n == 5 && r[2] == MB_EX_ILLEGAL_VALUE); }
    { const uint8_t p[] = {0x06, 0x00, 0x04, 0x00, 0x01};                       /* holding 4 no existe */
      n = xfer(1, p, sizeof p, r);
      CHECK(n == 5 && r[1] == 0x86 && r[2] == MB_EX_ILLEGAL_ADDRESS && regs.cal == 1234); }
    { const uint8_t p[] = {0x03, 0x01, 0x00, 0x00, 0x01};                       /* rango de usuario, sin gancho */
      n = xfer(1, p, sizeof p, r);
      CHECK(n == 5 && r[2] == MB_EX_ILLEGAL_ADDRESS); }

    /* --- tramas que se ignoran --- */
    { const uint8_t p[] = {0x03, 0x00, 0x00, 0x00, 0x01};
      CHECK(xfer(2, p, sizeof p, r) == 0);               /* otro ID */
      uint8_t bad[8] = {1, 3, 0, 0, 0, 1, 0, 0};          /* CRC mala */
      mb_regs_t m = enc_regs_mb(&regs);
      CHECK(mb_process(1, &m, bad, 8, r) == 0);
      CHECK(mb_process(1, &m, bad, 3, r) == 0);           /* corta */
      CHECK(xfer(0, p, sizeof p, r) == 0); }              /* leer por broadcast */

    /* --- broadcast: escribe y no responde; snapshot --- */
    fresh();
    enc_regs_update(&regs, 500);
    { const uint8_t p[] = {0x06, 0x00, ENC_HREG_COMMAND, 0x00, ENC_CMD_SNAPSHOT};
      n = xfer(0, p, sizeof p, r);
      CHECK(n == 0);
      CHECK(enc_regs_take_actions(&regs, ENC_ACT_SNAPSHOT) == ENC_ACT_SNAPSHOT);
      enc_regs_snapshot(&regs);
      CHECK(regs.snap_cal == 500 && regs.snap_seq == 1); }
    { const uint8_t p[] = {0x06, 0x00, ENC_HREG_OFFSET, 0x00, 0x0A};            /* offset por broadcast */
      CHECK(xfer(0, p, sizeof p, r) == 0 && regs.offset == 10); }

    /* --- snapshot crudo (registros 13..15): no depende de offset ni de sentido --- */
    fresh();
    regs.offset = 1000; regs.direction = 1;
    enc_regs_set_magnet(&regs, (1u << 5));                  /* iman detectado */
    enc_regs_update(&regs, 700);
    CHECK(regs.cal == enc_calibrate(700, 1, 1000) && regs.cal != 700);
    enc_regs_snapshot(&regs);
    enc_regs_update(&regs, 900);                            /* el brazo sigue moviendose */
    { const uint8_t p[] = {0x04, 0x00, ENC_IREG_SNAP_RAW, 0x00, 0x03};
      n = xfer(1, p, sizeof p, r);
      CHECK(n == 5 + 6 && resp_ok(r, n) && r[2] == 6);
      CHECK(r[3] == (700 >> 8) && r[4] == (700 & 0xFF));              /* crudo congelado, sin offset */
      CHECK(r[5] == 0 && (r[6] & 0x0F) == ENC_STATUS_MAGNET);         /* estado en el snapshot */
      CHECK(r[7] == 0 && r[8] == 1); }                                /* SNAP_SEQ = 1 */
    { const uint8_t p[] = {0x04, 0x00, ENC_IREG_SNAP_SEQ, 0x00, 0x01};
      n = xfer(1, p, sizeof p, r);
      CHECK(n == 7 && r[3] == 0 && r[4] == 1); }                      /* y el registro 5 coincide */
    enc_regs_snapshot(&regs);
    CHECK(regs.snap_raw == 900 && regs.snap_seq == 2);

    /* --- SET_ZERO: la lectura actual pasa a 0, con y sin inversion --- */
    fresh();
    enc_regs_update(&regs, 700);
    { const uint8_t p[] = {0x06, 0x00, ENC_HREG_COMMAND, 0x00, ENC_CMD_SET_ZERO};
      xfer(1, p, sizeof p, r);
      CHECK(regs.offset == 700 && regs.cal == 0); }
    regs.direction = 1; enc_regs_update(&regs, 700);
    { const uint8_t p[] = {0x06, 0x00, ENC_HREG_COMMAND, 0x00, ENC_CMD_SET_ZERO};
      xfer(1, p, sizeof p, r);
      CHECK(regs.cal == 0);
      enc_regs_update(&regs, 701);                        /* girar un paso: invertido => 4095 */
      CHECK(regs.cal == 4095); }

    /* --- cambio de ID: responde con el ID viejo, se aplica despues --- */
    fresh();
    { const uint8_t p[] = {0x06, 0x00, ENC_HREG_SLAVE_ID, 0x00, 0x05};
      n = xfer(1, p, sizeof p, r);
      CHECK(n == 8 && r[0] == 1 && resp_ok(r, n));
      CHECK(regs.id == 1 && regs.pending_id == 5);
      CHECK(enc_regs_take_actions(&regs, ENC_ACT_ID) == ENC_ACT_ID);
      enc_regs_apply_id(&regs);
      CHECK(regs.id == 5); }
    { const uint8_t p[] = {0x06, 0x00, ENC_HREG_SLAVE_ID, 0x00, 0x00};          /* id 0 no vale */
      n = xfer(5, p, sizeof p, r);
      CHECK(n == 5 && r[2] == MB_EX_ILLEGAL_VALUE); }
    { const uint8_t p[] = {0x06, 0x00, ENC_HREG_SLAVE_ID, 0x00, 248};           /* ni 248 */
      n = xfer(5, p, sizeof p, r);
      CHECK(n == 5 && r[2] == MB_EX_ILLEGAL_VALUE); }

    /* --- comandos invalidos y SAVE --- */
    fresh();
    { const uint8_t p[] = {0x06, 0x00, ENC_HREG_COMMAND, 0x00, 0x09};
      n = xfer(1, p, sizeof p, r);
      CHECK(n == 5 && r[2] == MB_EX_ILLEGAL_VALUE); }
    { const uint8_t p[] = {0x06, 0x00, ENC_HREG_COMMAND, 0x00, ENC_CMD_SAVE};
      n = xfer(1, p, sizeof p, r);
      CHECK(n == 8 && enc_regs_take_actions(&regs, ENC_ACT_SAVE) == ENC_ACT_SAVE); }

    /* --- FC16: offset + sentido de una vez; atomico si algo es invalido --- */
    fresh();
    { const uint8_t p[] = {0x10, 0x00, 0x00, 0x00, 0x02, 0x04, 0x00, 0x64, 0x00, 0x01};
      n = xfer(1, p, sizeof p, r);
      CHECK(n == 8 && resp_ok(r, n) && memcmp(r, "\x01\x10\x00\x00\x00\x02", 6) == 0);
      CHECK(regs.offset == 100 && regs.direction == 1); }
    { const uint8_t p[] = {0x10, 0x00, 0x00, 0x00, 0x02, 0x04, 0x00, 0xC8, 0x00, 0x07}; /* sentido 7 invalido */
      n = xfer(1, p, sizeof p, r);
      CHECK(n == 5 && r[1] == 0x90 && r[2] == MB_EX_ILLEGAL_VALUE);
      CHECK(regs.offset == 100 && regs.direction == 1); }                       /* nada se aplico */
    { const uint8_t p[] = {0x10, 0x00, 0x00, 0x00, 0x02, 0x02, 0x00, 0x64};     /* byte count inconsistente */
      n = xfer(1, p, sizeof p, r);
      CHECK(n == 5 && r[2] == MB_EX_ILLEGAL_VALUE); }

    /* --- I2C / magnet: bits de estado --- */
    fresh();
    enc_regs_set_magnet(&regs, (1u << 5));                  /* MD */
    CHECK((regs.status & 0x07u) == ENC_STATUS_MAGNET);
    enc_regs_set_magnet(&regs, (1u << 4) | (1u << 3));      /* ML + MH */
    CHECK((regs.status & 0x07u) == (ENC_STATUS_TOO_WEAK | ENC_STATUS_TOO_STRONG));
    CHECK(regs.status & ENC_STATUS_DEFAULTS);               /* otros bits no se tocan */

    printf("%d chequeos, %d fallas\n", checks, failures);
    return failures ? 1 : 0;
}
