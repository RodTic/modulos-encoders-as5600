/* Simulador en PC de uno o varios nodos (usa el mismo mb_slave.c y enc_regs.c que el firmware).
 * Sirve para probar enc_tool.py sin placa:
 *   gcc -Wall -Wextra -std=c11 -IApp/encoder_node/Inc tools/sim_slave.c \
 *       App/encoder_node/Src/mb_slave.c App/encoder_node/Src/enc_regs.c -o tools/sim_slave.exe
 *   sim_slave.exe 1 2 3          (nodos con esos IDs)
 * Entrada por stdin:  "RAW <nodo> <valor>"  fija el angulo crudo del nodo (indice desde 0)
 *                     "RX <hex>"            trama Modbus que llega al bus
 * Salida: "OK" o "TX <hex>" (respuestas de todos los nodos concatenadas) / "TX -" */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "enc_regs.h"

#define MAXN 8

int main(int argc, char **argv)
{
    enc_regs_t nodes[MAXN];
    int n = 0;
    for (int i = 1; i < argc && n < MAXN; i++, n++) {
        uint16_t uid[6] = { (uint16_t)(n + 1), 0, 0, 0, 0, 0 };
        enc_regs_init(&nodes[n], uid);
        nodes[n].id = (uint16_t)atoi(argv[i]);
    }

    static char line[2048];
    while (fgets(line, sizeof line, stdin)) {
        if (strncmp(line, "RAW ", 4) == 0) {
            int idx, v;
            if (sscanf(line + 4, "%d %d", &idx, &v) == 2 && idx >= 0 && idx < n)
                enc_regs_update(&nodes[idx], (uint16_t)v);
            puts("OK");
        } else if (strncmp(line, "RX ", 3) == 0) {
            uint8_t req[MB_FRAME_MAX], out[MB_FRAME_MAX * MAXN];
            size_t len = 0, outlen = 0;
            for (const char *p = line + 3; p[0] && p[1] && p[0] != '\n' && len < sizeof req; p += 2) {
                char h[3] = { p[0], p[1], 0 };
                req[len++] = (uint8_t)strtoul(h, NULL, 16);
            }
            for (int i = 0; i < n; i++) {
                mb_regs_t m = enc_regs_mb(&nodes[i]);
                size_t rl = mb_process((uint8_t)nodes[i].id, &m, req, len, out + outlen);
                outlen += rl;
                if (enc_regs_take_actions(&nodes[i], ENC_ACT_SNAPSHOT))
                    enc_regs_snapshot(&nodes[i]);
                enc_regs_take_actions(&nodes[i], ENC_ACT_SAVE | ENC_ACT_RESET);   /* sin flash ni reset en PC */
                if (enc_regs_take_actions(&nodes[i], ENC_ACT_ID))
                    enc_regs_apply_id(&nodes[i]);
            }
            if (outlen == 0) {
                puts("TX -");
            } else {
                fputs("TX ", stdout);
                for (size_t i = 0; i < outlen; i++) printf("%02x", out[i]);
                putchar('\n');
            }
        }
        fflush(stdout);
    }
    return 0;
}
