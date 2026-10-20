/* enc_store.h - configuracion persistente en la ultima pagina de flash. */
#ifndef ENC_STORE_H
#define ENC_STORE_H

#include <stdint.h>

typedef struct { uint16_t id, offset, direction; } enc_cfg_t;

/* 0 = registro valido cargado; -1 = no hay registro valido (placa nueva). */
int enc_store_load(enc_cfg_t *c);

/* 0 = ok; -1 = error de flash; -2 = la pagina se solapa con el firmware
 * (bajar FLASH LENGTH a 63K en el .ld). Congela la CPU ~40 ms (borrado). */
int enc_store_save(const enc_cfg_t *c);

#endif
