# encoder_node

Módulo para el STM32F030C8T6 + AS5600 + SP3485. Lee el ángulo, lo calibra (offset y sentido) y lo entrega por Modbus RTU (RS-485). Se copia entero a otro proyecto.

## Qué hace

- Esclavo Modbus RTU, 115200 8N1, ID 1..247 (una placa nueva es 247).
- El offset, el sentido y el ID se guardan en la última página de flash.
- `broadcast` de snapshot: congela el ángulo de los 6 nodos casi en el mismo instante.

## Mapa de registros

Holding (FC 03 leer, FC 06 / FC 16 escribir):

| Dir | Nombre | Valores |
|---|---|---|
| 0 | OFFSET | 0..4095 cuentas |
| 1 | DIRECTION | 0 normal, 1 invertido |
| 2 | SLAVE_ID | 1..247. Se aplica después de responder |
| 3 | COMMAND | 1 guardar en flash, 2 valores de fábrica (en RAM; luego guardar), 3 poner cero aquí, 4 reset, 5 snapshot |

Input (FC 04, solo lectura):

| Dir | Nombre |
|---|---|
| 0 | ANGLE_CAL (0..4095 con offset y sentido aplicados) |
| 1 | ANGLE_RAW |
| 2 | STATUS: 0x01 imán ok, 0x02 débil, 0x04 fuerte, 0x08 error I2C, 0x10 sin config guardada, 0x20 falló el guardado |
| 3 | SAMPLE_COUNT |
| 4 | SNAP_ANGLE |
| 5 | SNAP_SEQ |
| 6..11 | UID de 96 bits del chip |
| 12 | FW_VERSION (0x0100 = 1.00) |

`cal = ((sentido ? 4096 - raw : raw) - offset) mod 4096`. Un grado son 4096/360 = 11,38 cuentas.

Los registros `>= 0x0100` son del proyecto grande: implementa `enc_user_reg_read` y `enc_user_reg_write` (declarados en `enc_regs.h`). Por defecto devuelven "dirección ilegal".

## Recursos que ocupa (no reutilizar en el proyecto grande)

| Recurso | Uso |
|---|---|
| PB9, PB10, PA8 | Tx, Rx y DE/RE del RS-485 |
| TIM16, TIM17, EXTI línea 10 | Bit-clock, fin de trama y flanco de arranque |
| IRQ TIM16, TIM17, EXTI4_15 | Los define `swuart.c`. Si el proyecto grande ya usa `EXTI4_15_IRQHandler` u otro, poner `ENC_DEFINE_IRQ_HANDLERS 0` y llamar a `swuart_*_isr()` desde los suyos |
| Última página flash 0x0800FC00 | Configuración |
| I2C1 (PB6/PB7) | Por el handle `hi2c1` de CubeMX. Se puede compartir si solo se accede desde el bucle principal |
| Prioridad NVIC 0 | TIM16 y EXTI. No poner otras ISR largas con prioridad 0 |

## Integrar en un proyecto (pasos)

1. Copiar la carpeta `encoder_node` dentro del proyecto (por ejemplo `App/encoder_node`).
2. CubeIDE: clic derecho en el proyecto, Properties, C/C++ General, Paths and Symbols.
   - Pestaña Includes, GNU C, Add: `${ProjName}/App/encoder_node/Inc`.
   - Pestaña Source Location, Add Folder: `App`.
3. CubeMX: HSE como cristal con 11,0592 MHz, PLL ×4 (HCLK = 44,2368 MHz, APB1 /1), SWD, e I2C1 en PB6/PB7.
4. En `main.c`:
   ```c
   /* USER CODE BEGIN Includes */
   #include "enc_node.h"
   /* USER CODE END Includes */
   ...
   /* USER CODE BEGIN 2 */
   enc_node_init(&hi2c1);        /* devuelve -1 si el reloj no es 44,2368 MHz */
   /* USER CODE END 2 */
   ...
   /* USER CODE BEGIN 3 */
   enc_node_poll();
   /* USER CODE END 3 */
   ```
5. En el `.ld`: `FLASH (rx) : ORIGIN = 0x8000000, LENGTH = 63K` (antes 64K). CubeMX regenera ese archivo, así que hay que repetirlo si vuelves a generar código.
6. Leer los datos desde la aplicación con `enc_node_angle_cal()`, `enc_node_angle_raw()`, `enc_node_status()` y `enc_node_sample_count()`.

## Notas de bus

- Terminación de 120 Ω solo en los dos extremos del cable. La placa trae R6 fija en cada módulo, así que hay que dejar sin montar R6 en los intermedios.
- Resistencias de bias (pull-up en A, pull-down en B) en el maestro, para que la línea no genere bytes basura en reposo.
- Asignar el ID con una placa nueva conectada a la vez, porque todas parten como 247.
- Guardar en flash congela el nodo unos 40 ms. El script espera 150 ms.
- Si hay errores de CRC a 115200: bajar `ENC_BAUD` a 57600 (en `enc_config.h` y `-b` en el script), o cablear a PA9/PA10 y reemplazar `swuart.c` por un USART1 de hardware.

## Pruebas en PC (sin placa)

Desde la carpeta del proyecto (`modulos_encoders/`):

```
gcc -Wall -Wextra -std=c11 -IApp/encoder_node/Inc tools/test_mb_rtu.c App/encoder_node/Src/mb_slave.c App/encoder_node/Src/enc_regs.c -o tools/test_mb_rtu.exe && tools/test_mb_rtu.exe
gcc -Wall -Wextra -std=c11 -IApp/encoder_node/Inc tools/sim_slave.c App/encoder_node/Src/mb_slave.c App/encoder_node/Src/enc_regs.c -o tools/sim_slave.exe && python tools/enc_tool.py selftest
```
