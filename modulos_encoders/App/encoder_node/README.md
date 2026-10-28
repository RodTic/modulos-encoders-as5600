# encoder_node

Módulo para el STM32F030C8T6 + AS5600 + SP3485. Lee el ángulo, lo calibra (offset y sentido) y lo entrega por Modbus RTU (RS-485). Se copia entero a otro proyecto.

## Qué hace

- Esclavo Modbus RTU, **230400 8N1**, ID 1..247 (una placa nueva es 247). El silencio de fin de trama es de 500 µs (el estándar pide 1,75 ms; se acorta para que la lectura de 6 placas sea más rápida).
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
| 13 | SNAP_RAW: ángulo **crudo** (sin offset ni sentido) congelado por el último snapshot |
| 14 | SNAP_STATUS: el STATUS en el momento del snapshot |
| 15 | SNAP_SEQ: número de snapshot (copia del registro 5) |

Los registros 13 a 15 se leen con una sola consulta FC 04 (3 registros, respuesta de 11 bytes). Es lo que usa `encoders_modbus.py`.

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
- `ENC_BAUD` es 230400, el máximo estimado del UART por software: la ISR de cada bit cuesta ~100 ciclos de CPU y el bit dura 192, y a 460800 es imposible (96). Está al límite y **no se ha probado en la placa**. Si hay errores de CRC, bajar `ENC_BAUD` a 115200 (el silencio pasa solo a 1750 µs) y cambiar el baudrate también en el script y en `LectorEncoders`; si sigue fallando, 57600. La otra salida es soldar los cables a PA9/PA10 y usar el proyecto `modulos_encoders_usart`.
- `ENC_RX_ENTRY_COMP_CYCLES` (80) centra el primer muestreo en el bit 0. Salió del desensamblado de la ISR; si hay errores de trama sin otra causa, es lo primero que se ajusta (probar 60 y 100).

## Lectura rápida desde el laptop

`tools/encoders_modbus.py` es un reemplazo de `LectorEncoders` (el de `encoders.py` del proyecto grande): misma interfaz, pero lee las 6 placas por Modbus en vez de las líneas del ESP32. Entrega los ángulos en grados, **crudos** (sin offset ni sentido) y nunca escribe en las placas.

```python
from encoders_modbus import LectorEncoders
lector = LectorEncoders("COM6", 230400)   # los baudios del firmware
print(lector.angulos, lector.frecuencia_hz)
```

Cada ronda hace un snapshot por broadcast y una consulta por placa. Si un canal falla (sin respuesta, imán ausente, CRC mala, snapshot perdido) conserva su último valor bueno y suma un error. Una placa caída se salta 1 s para no frenar a las demás. `sincronizar=False` omite el snapshot (más rápido, pero cada canal en un instante distinto). Con menos de 6 placas conectadas, pasa cuáles existen: `LectorEncoders("COM6", 921600, ids=(1, 2))`. Las demás no se consultan y sus canales quedan en `NaN` (tu cinemática debe tolerarlo, igual que antes con un canal que nunca respondió). Cada placa necesita su propio ID (se asigna una por una con `set-id`, porque todas vienen como 247). Si el AS5600 reporta imán ausente, débil o fuerte, el lector descarta esa lectura y conserva la anterior (cuenta un error); con `exigir_iman=False` la acepta igual, por ejemplo para probar sin imán.

**Frecuencia medida** con una placa real y este firmware (Debug, 230400 baudios): cada consulta cuesta **2,0 ms** (casi todo es latencia USB del adaptador) y una ronda con snapshot dura 5,0 ms, es decir **200 rondas por segundo con 1 placa y 0 errores en 1000 rondas**. Con 6 placas cada una suma 2,0 ms y la espera tras el snapshot es de ~2,85 ms, así que una ronda de 6 dura unos 15 ms: **~65 Hz** (calculado a partir de lo medido, no medido con 6), frente a ~35 Hz del ESP32. El baudrate casi no influye; mandan la latencia USB y el tiempo que tarda el módulo en soltar su buffer tras el snapshot (hasta ~2,1 ms, medido: con 2,5 ms de espera total, 0 errores; con 2,0 ms, 3 %; con 1,5 ms, 38 %).

Para medirla de verdad con las 6 placas conectadas y con ID 1 a 6:

```
python tools/enc_tool.py -p COM6 -b 230400 bench 1 2 3 4 5 6 --count 500
```

Poner el resultado en `FRECUENCIA_FIRMWARE_HZ` de `config.py` (se usa para decidir si los datos están frescos).

**Ver los ángulos en vivo.** `poll` lee sin parar hasta que lo detengas, desde que lo escribes:

```
python tools/enc_tool.py -p COM10 -b <baudrate> poll 247 --rate 20
```

Con **ESPACIO** lo pausas (en pausa no se manda nada al bus) y con **ESPACIO** otra vez continúa. **Q**, Esc o Ctrl+C lo detienen. Los avisos (`[PAUSA]`, `[CONTINUA]`) salen por stderr, así que `... poll 1 2 3 --csv > datos.csv` guarda un CSV limpio y el tiempo de cada fila no cuenta las pausas. Si pasas `--count N`, se detiene solo tras N rondas. Las teclas solo funcionan en Windows; en otros sistemas se detiene con Ctrl+C.

## Pruebas en PC (sin placa)

Desde la carpeta del proyecto (`modulos_encoders/`):

```
gcc -Wall -Wextra -std=c11 -IApp/encoder_node/Inc tools/test_mb_rtu.c App/encoder_node/Src/mb_slave.c App/encoder_node/Src/enc_regs.c -o tools/test_mb_rtu.exe && tools/test_mb_rtu.exe
gcc -Wall -Wextra -std=c11 -IApp/encoder_node/Inc tools/sim_slave.c App/encoder_node/Src/mb_slave.c App/encoder_node/Src/enc_regs.c -o tools/sim_slave.exe && python tools/enc_tool.py selftest && python tools/test_encoders_modbus.py
```
