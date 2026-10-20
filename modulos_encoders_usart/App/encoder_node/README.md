# encoder_node

Módulo para el STM32F030C8T6 + AS5600 + SP3485. Lee el ángulo, lo calibra (offset y sentido) y lo entrega por Modbus RTU (RS-485). Se copia entero a otro proyecto.

**Variante USART1 de hardware** (proyecto `modulos_encoders_usart`). Requiere modificar la placa: soldar un cable de PA9 (pin 30) a la red Tx y otro de PA10 (pin 31) a la red Rx. El módulo configura el USART1 por sí mismo, así que CubeMX no necesita activarlo. La variante sin modificar la placa (UART por software en PB9/PB10) está en el proyecto `modulos_encoders`; todo lo demás del módulo es idéntico.

## Qué hace

- Esclavo Modbus RTU, **921600 8N1**, ID 1..247 (una placa nueva es 247). El silencio de fin de trama es de 500 µs (el estándar pide 1,75 ms; se acorta para que la lectura de 6 placas sea más rápida).
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
| USART1 (PA9 Tx, PA10 Rx) | Puerto RS-485. Si el proyecto grande lo necesita para otra cosa, este módulo no puede usarse |
| PA8 | DE/RE del RS-485 |
| PB9, PB10 | Se dejan en alta impedancia porque comparten red con PA9/PA10 |
| TIM17 | Silencio de fin de trama |
| IRQ USART1, TIM17 | Los define `rs485_usart.c`. Si el proyecto grande ya usa `USART1_IRQHandler` u otro, poner `ENC_DEFINE_IRQ_HANDLERS 0` y llamar a `rs485_usart1_isr()` y `rs485_tim17_isr()` desde los suyos |
| Última página flash 0x0800FC00 | Configuración |
| I2C1 (PB6/PB7) | Por el handle `hi2c1` de CubeMX. Se puede compartir si solo se accede desde el bucle principal |
| Prioridad NVIC 0 | USART1. No poner otras ISR largas con prioridad 0 |

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
- `ENC_BAUD` es 921600: es exacto en el STM32 (BRR = 48) y en el adaptador. Se puede probar 1843200 (BRR = 24, también exacto), pero la lectura casi no se acelera porque la frenan el silencio de fin de trama y la latencia USB, no el baudrate. Si hay errores de CRC, bajar a 460800. Cambiar también el baudrate en el script y en `LectorEncoders`.

## Lectura rápida desde el laptop

`tools/encoders_modbus.py` es un reemplazo de `LectorEncoders` (el de `encoders.py` del proyecto grande): misma interfaz, pero lee las 6 placas por Modbus en vez de las líneas del ESP32. Entrega los ángulos en grados, **crudos** (sin offset ni sentido) y nunca escribe en las placas.

```python
from encoders_modbus import LectorEncoders
lector = LectorEncoders("COM6", 921600)   # los baudios del firmware
print(lector.angulos, lector.frecuencia_hz)
```

Cada ronda hace un snapshot por broadcast y una consulta por placa. Si un canal falla (sin respuesta, imán ausente, CRC mala, snapshot perdido) conserva su último valor bueno y suma un error. Una placa caída se salta 1 s para no frenar a las demás. `sincronizar=False` omite el snapshot (más rápido, pero cada canal en un instante distinto).

**Frecuencia estimada** (sin medir; la incógnita es la latencia USB de tu adaptador, entre 1 y 2,5 ms por consulta): **~45 a 80 rondas por segundo** con este firmware (921600 baudios), frente a ~35 Hz del ESP32. El baudrate casi no influye; mandan el silencio de fin de trama y la latencia USB.

Para medirla de verdad con las 6 placas conectadas y con ID 1 a 6:

```
python tools/enc_tool.py -p COM6 -b 921600 bench 1 2 3 4 5 6 --count 500
```

Poner el resultado en `FRECUENCIA_FIRMWARE_HZ` de `config.py` (se usa para decidir si los datos están frescos).

## Pruebas en PC (sin placa)

Desde la carpeta del proyecto (`modulos_encoders_usart/`):

```
gcc -Wall -Wextra -std=c11 -IApp/encoder_node/Inc tools/test_mb_rtu.c App/encoder_node/Src/mb_slave.c App/encoder_node/Src/enc_regs.c -o tools/test_mb_rtu.exe && tools/test_mb_rtu.exe
gcc -Wall -Wextra -std=c11 -IApp/encoder_node/Inc tools/sim_slave.c App/encoder_node/Src/mb_slave.c App/encoder_node/Src/enc_regs.c -o tools/sim_slave.exe && python tools/enc_tool.py selftest && python tools/test_encoders_modbus.py
```
