# Módulos encoder AS5600 + STM32F030 (Modbus RTU)

Firmware para un módulo de medición de ángulo: un sensor magnético AS5600 se lee por I2C con un STM32F030C8T6, y el módulo responde por RS-485 (Modbus RTU). Seis módulos en cadena entregan los ángulos de las articulaciones de un brazo pasivo a un laptop. Cada módulo se calibra por Modbus (ID, offset y sentido de giro) y guarda la calibración en flash.

## Contenido

- `modulos_encoders/App/encoder_node/`: el módulo reutilizable (Modbus esclavo, driver del AS5600, UART por software, guardado en flash). Su [README](modulos_encoders/App/encoder_node/README.md) tiene el mapa de registros y los pasos para integrarlo en otro proyecto.
- `modulos_encoders/Core/`, `Drivers/`, `Startup/`: proyecto STM32CubeIDE generado con CubeMX (HAL de ST).
- `modulos_encoders/tools/`: script de laptop (`enc_tool.py`, solo requiere `pyserial`), test del protocolo en PC y simulador.

## Compilar

Abrir `modulos_encoders` en STM32CubeIDE (File → Open Projects from File System) y compilar con Ctrl+B. El reloj debe ser HSE 11,0592 MHz con PLL ×4 (ya está en el `.ioc`).

## Pruebas sin placa

Desde `modulos_encoders/`:

```
gcc -Wall -Wextra -std=c11 -IApp/encoder_node/Inc tools/test_mb_rtu.c App/encoder_node/Src/mb_slave.c App/encoder_node/Src/enc_regs.c -o tools/test_mb_rtu.exe && tools/test_mb_rtu.exe
gcc -Wall -Wextra -std=c11 -IApp/encoder_node/Inc tools/sim_slave.c App/encoder_node/Src/mb_slave.c App/encoder_node/Src/enc_regs.c -o tools/sim_slave.exe && python tools/enc_tool.py selftest
```

## Estado

El protocolo y la calibración están probados en PC. El firmware compila con el ARM GCC de CubeIDE. Falta la prueba en hardware: el UART por software a 115200 está al límite del chip (el esquema usa PB9/PB10, que no son pines de USART en el F030C8). Si hay errores de CRC, bajar `ENC_BAUD` a 57600.

Los archivos de `Drivers/` son de STMicroelectronics y mantienen sus propias licencias (ver los `LICENSE.txt` dentro).
