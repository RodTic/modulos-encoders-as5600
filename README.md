# Módulos encoder AS5600 + STM32F030 (Modbus RTU)

Firmware para un módulo de medición de ángulo: un sensor magnético AS5600 se lee por I2C con un STM32F030C8T6, y el módulo responde por RS-485 (Modbus RTU). Seis módulos en cadena entregan los ángulos de las articulaciones de un brazo pasivo a un laptop. Cada módulo se calibra por Modbus (ID, offset y sentido de giro) y guarda la calibración en flash.

## Tres proyectos

Los tres son proyectos completos de STM32CubeIDE, con el mismo módulo `encoder_node`. Los dos primeros cambian solo en cómo se comunican con el chip RS-485; el tercero es un piloto aparte:

| Carpeta | Puerto RS-485 | Baudrate | Lectura de 6 placas | Requiere |
|---|---|---|---|---|
| `modulos_encoders/` | UART por software en PB9 (Tx) y PB10 (Rx) | 230400 | ~48 Hz (calculado de lo medido con 2 placas: 108 Hz; con 1 placa 200 Hz) | La placa tal como está |
| `modulos_encoders_usart/` | USART1 de hardware en PA9 (Tx) y PA10 (Rx) | 921600 | ~48 Hz o algo más (sin medir) | Soldar un cable de PA9 (pin 30) a la red Tx y otro de PA10 (pin 31) a la red Rx |
| `modulos_encoders_rafaga/` (piloto) | UART por software en PB9 y PB10, como el primero | 230400 | ~110 Hz estimados con modo ráfaga (medido con 2 placas: 155 Hz, 0 errores; con 6 sin medir) | La placa tal como está |

El proyecto **piloto** `modulos_encoders_rafaga/` agrega un modo "ráfaga" que no es Modbus estándar: un solo broadcast y cada placa contesta sola en su turno, en vez de una consulta por placa. Sigue entendiendo todo el Modbus estándar. Los detalles y las reglas del bus están en su `App/encoder_node/README.md`.

Con una placa real se midieron 2,0 ms por consulta y 200 rondas/s; con 2 placas, 9,2 ms por ronda (108 Hz) con snapshot y 6,1 ms (165 Hz) sin él. El valor para 6 placas se calculó a partir de eso. Entre una respuesta y la siguiente petición el maestro deja ~0,9 ms de silencio (lo necesitan las demás placas para cerrar la trama que oyeron; sin él, la 2.ª placa fallaba el 72 % de las veces). Con las 6 placas conectadas se mide con `python tools/enc_tool.py -p COM6 -b <baudrate> bench 1 2 3 4 5 6`.

## Usarlo en el proyecto grande (Python)

`tools/encoders_modbus.py` reemplaza a la clase `LectorEncoders` de `encoders.py`: misma interfaz (`angulos`, `datos_frescos`, `estadisticas_errores`...), pero lee las 6 placas por Modbus en vez de las líneas del ESP32. Entrega grados crudos, sin offset ni sentido de giro, y nunca escribe en las placas. Hay una copia idéntica en la carpeta `tools/` de cada proyecto. Los baudios que se le pasan deben ser los del firmware que cargaste.

La diferencia en el código está en un solo archivo: `swuart.c` en el primero y `rs485_usart.c` en el segundo (más 3 líneas en `enc_node.c`). Si arreglas un error en el resto del módulo, hay que copiarlo a la otra carpeta.

## Contenido de cada proyecto

- `App/encoder_node/`: el módulo reutilizable (Modbus esclavo, driver del AS5600, puerto RS-485, guardado en flash). Su `README.md` tiene el mapa de registros y los pasos para integrarlo en otro proyecto.
- `Core/`, `Drivers/`, `Startup/`: proyecto STM32CubeIDE generado con CubeMX (HAL de ST).
- `tools/`: script de laptop (`enc_tool.py`, solo requiere `pyserial`), test del protocolo en PC y simulador.

## Compilar

Abrir la carpeta del proyecto en STM32CubeIDE (File → Open Projects from File System) y compilar con Ctrl+B. El reloj debe ser HSE 11,0592 MHz con PLL ×4 (ya está en el `.ioc`).

## Pruebas sin placa

Desde `modulos_encoders/`:

```
gcc -Wall -Wextra -std=c11 -IApp/encoder_node/Inc tools/test_mb_rtu.c App/encoder_node/Src/mb_slave.c App/encoder_node/Src/enc_regs.c -o tools/test_mb_rtu.exe && tools/test_mb_rtu.exe
gcc -Wall -Wextra -std=c11 -IApp/encoder_node/Inc tools/sim_slave.c App/encoder_node/Src/mb_slave.c App/encoder_node/Src/enc_regs.c -o tools/sim_slave.exe && python tools/enc_tool.py selftest
```

## Estado

El protocolo y la calibración están probados en PC. Los dos proyectos compilan con el ARM GCC de CubeIDE. Falta la prueba en hardware. El esquema usa PB9/PB10 para el RS-485, que no son pines de USART en el F030C8: por eso el proyecto `modulos_encoders` usa un UART por software, que a 115200 está al límite del chip (si hay errores de CRC, bajar `ENC_BAUD` a 57600). El proyecto `modulos_encoders_usart` evita ese límite, pero necesita los dos cables soldados.

Los archivos de `Drivers/` son de STMicroelectronics y mantienen sus propias licencias (ver los `LICENSE.txt` dentro).
