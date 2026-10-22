"""Lectura en tiempo real de los 6 encoders AS5600 por Modbus RTU (RS-485).

Reemplaza a encoders.py: misma clase LectorEncoders, mismas propiedades. En vez de
leer las lineas CSV de un ESP32, este lector pregunta a las 6 placas STM32 (IDs 1..6)
a traves de un adaptador USB-RS485, en un HILO aparte.

Cada ronda hace:
  1. un snapshot por broadcast: las 6 placas congelan su angulo en el mismo instante
     (diferencia menor a ~0.5 ms);
  2. una consulta Modbus (FC 04) a cada placa, que devuelve el angulo congelado.

Los angulos salen en GRADOS (0..360), CRUDOS: no se aplica offset ni sentido de giro, y
este modulo NUNCA escribe en las placas. Se lee el registro de angulo crudo, asi que
da igual el offset que una placa tenga guardado.

Si un canal falla (sin respuesta, CRC mala, iman ausente/debil/fuerte, error I2C de la
placa, o la placa no tomo el snapshot de esa ronda) se cuenta el error y se CONSERVA el
ultimo valor bueno de esa articulacion, igual que con el "ERR" del ESP32. Una placa que
deja de responder se salta durante 1 s para no frenar a las demas con tiempos de espera.

Uso (igual que antes):
    lector = LectorEncoders("COM6", 921600)   # baudios = los del firmware de las placas
    angulos = lector.angulos                  # los 6 ultimos valores conocidos
    lector.cerrar()

Baudios del firmware: 921600 (proyecto modulos_encoders_usart) o 230400 (proyecto
modulos_encoders, UART por software).
"""
import math
import struct
import threading
import time
import traceback

import serial

try:
    from config import FRECUENCIA_FIRMWARE_HZ
except Exception:                       # prueba aislada, sin el config.py del proyecto grande
    FRECUENCIA_FIRMWARE_HZ = 50.0

NUM_ENCODERS = 6
CUENTAS_POR_VUELTA = 4096.0

# Datos "frescos" si llego algo en las ultimas 10 rondas. Poner en config.py la frecuencia
# medida (frecuencia_hz de esta clase, o el comando bench de enc_tool.py).
UMBRAL_DATOS_FRESCOS_S = 10.0 / FRECUENCIA_FIRMWARE_HZ

# --- Mapa de registros de las placas (ver App/encoder_node/README.md) ---
HREG_COMMAND = 3
CMD_SNAPSHOT = 5
IREG_RAW = 1              # en vivo: 1 = crudo, 2 = estado
IREG_SNAP_RAW = 13        # tras un snapshot: 13 = crudo, 14 = estado, 15 = numero de snapshot
STATUS_OK_MASK = 0x0F     # iman detectado (0x01), sin debil/fuerte/error I2C
STATUS_OK = 0x01

TIMEOUT_RESPUESTA_S = 0.02
FALLOS_PARA_SALTAR = 2
SALTAR_S = 1.0
MARGEN_PROCESO_NODO_S = 0.0009   # lectura I2C fresca del nodo tras el snapshot + margen


def listar_puertos_serie():
    """Puertos serie detectados (para ayudar cuando no se puede abrir el puerto)."""
    try:
        import serial.tools.list_ports
        return list(serial.tools.list_ports.comports())
    except Exception:
        return []


def _crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def _con_crc(frame):
    return frame + struct.pack("<H", _crc16(frame))


def silencio_nodo_s(baudios):
    """Silencio con el que la placa da por terminada una trama (igual que ENC_GAP_US del firmware)."""
    if baudios >= 230400:
        return 500e-6
    if baudios > 19200:
        return 1750e-6
    return 38.5 / baudios


def espera_tras_broadcast_s(baudios):
    """Tiempo minimo entre el broadcast y la siguiente consulta: que termine de salir la
    trama (8 bytes), que cada placa detecte el fin de trama y que tome su lectura fresca."""
    return 80.0 / baudios + silencio_nodo_s(baudios) + MARGEN_PROCESO_NODO_S


def _esperar(segundos):
    """Espera precisa para tiempos cortos (time.sleep solo es exacto a ~1 ms en Windows)."""
    fin = time.perf_counter() + segundos
    if segundos > 0.003:
        time.sleep(segundos - 0.002)
    while time.perf_counter() < fin:
        pass


class LectorEncoders:
    """Mantiene los 6 angulos actualizados consultando las placas en segundo plano.

    puerto: nombre del puerto ("COM6") o, para pruebas, un objeto con read/write/
    reset_input_buffer/close. baudios: los del firmware. sincronizar=False omite el
    snapshot y lee el angulo en vivo (algo mas rapido, pero cada canal en un instante).
    ids: IDs (1..6) de las placas que de verdad estan conectadas; por defecto las 6. Las
    demas no se consultan: su canal queda en NaN y no suma errores.
    """

    def __init__(self, puerto, baudios: int, sincronizar: bool = True, ids=None):
        if isinstance(puerto, str):
            try:
                self._serial = serial.Serial(puerto, baudios, bytesize=8, parity="N", stopbits=1,
                                             timeout=TIMEOUT_RESPUESTA_S, write_timeout=0.1)
            except Exception:
                puertos = listar_puertos_serie()
                if puertos:
                    print(f"[ERROR] No se pudo abrir {puerto}. Puertos detectados:")
                    for p in puertos:
                        print(f"    - {p.device}  ({p.description})")
                raise
        else:
            self._serial = puerto

        self._ids = tuple(range(1, NUM_ENCODERS + 1)) if ids is None else tuple(sorted(set(ids)))
        if not self._ids or self._ids[0] < 1 or self._ids[-1] > NUM_ENCODERS:
            raise ValueError("ids debe ser un subconjunto de 1..%d" % NUM_ENCODERS)
        self._baudios = baudios
        self._sincronizar = sincronizar
        self._espera_broadcast = espera_tras_broadcast_s(baudios)

        self._lock = threading.Lock()
        self._angulos = [math.nan] * NUM_ENCODERS
        self._ultima_lectura = 0.0
        self._errores = [0] * NUM_ENCODERS
        self._lineas = 0
        self._periodo_s = 0.0
        self._seq_previo = [None] * NUM_ENCODERS
        self._fallos_seguidos = [0] * NUM_ENCODERS
        self._saltar_hasta = [0.0] * NUM_ENCODERS
        self._activo = True
        self._hilo = threading.Thread(target=self._bucle_lectura, daemon=True)
        self._hilo.start()

    # ------------------------------------------------------------ Modbus
    def _consultar(self, nodo, direccion, cantidad):
        """FC 04. Devuelve la lista de registros o None si no hubo respuesta valida."""
        ser = self._serial
        ser.write(_con_crc(struct.pack(">BBHH", nodo, 4, direccion, cantidad)))
        esperado = 5 + 2 * cantidad
        resp = ser.read(esperado)
        if (len(resp) != esperado
                or struct.unpack("<H", resp[-2:])[0] != _crc16(resp[:-2])
                or resp[0] != nodo or resp[1] != 4 or resp[2] != 2 * cantidad):
            ser.reset_input_buffer()          # descartar restos antes de la siguiente consulta
            return None
        return list(struct.unpack(">%dH" % cantidad, resp[3:-2]))

    def _snapshot(self):
        """Broadcast (ID 0): escribe el comando SNAPSHOT en el registro 3. Nadie responde."""
        self._serial.write(_con_crc(struct.pack(">BBHH", 0, 6, HREG_COMMAND, CMD_SNAPSHOT)))
        _esperar(self._espera_broadcast)

    # ------------------------------------------------------------ hilo
    def _bucle_lectura(self):
        try:
            self._serial.reset_input_buffer()
        except Exception:
            pass

        while self._activo:
            inicio = time.perf_counter()
            try:
                alguno = self._ronda()
            except serial.SerialException:
                break
            except Exception:
                traceback.print_exc()
                break
            if not alguno:
                time.sleep(0.01)              # nada responde: no girar a toda velocidad
            dt = time.perf_counter() - inicio
            with self._lock:
                self._periodo_s = dt if self._periodo_s == 0.0 else 0.9 * self._periodo_s + 0.1 * dt

    def _ronda(self):
        """Una ronda sobre las 6 placas. Devuelve True si algun canal dio un valor bueno."""
        if self._sincronizar:
            self._snapshot()
            direccion, cantidad = IREG_SNAP_RAW, 3
        else:
            direccion, cantidad = IREG_RAW, 2

        nuevos = [None] * NUM_ENCODERS
        ahora = time.monotonic()
        for nodo in self._ids:
            i = nodo - 1
            if ahora < self._saltar_hasta[i]:
                continue                       # placa caida: se vuelve a probar mas tarde
            regs = self._consultar(nodo, direccion, cantidad)
            if regs is None:
                self._fallos_seguidos[i] += 1
                if self._fallos_seguidos[i] >= FALLOS_PARA_SALTAR:
                    self._saltar_hasta[i] = time.monotonic() + SALTAR_S
                    # al reintentar basta un fallo para volver a saltarla (1 espera por segundo, no 2)
                    self._fallos_seguidos[i] = FALLOS_PARA_SALTAR - 1
                continue
            self._fallos_seguidos[i] = 0

            crudo, estado = regs[0], regs[1]
            valido = (estado & STATUS_OK_MASK) == STATUS_OK and crudo < CUENTAS_POR_VUELTA
            if self._sincronizar:
                seq = regs[2]
                # seq 0 = nunca tomo un snapshot; igual al de la ronda anterior = se perdio el broadcast
                if seq == 0 or seq == self._seq_previo[i]:
                    valido = False
                self._seq_previo[i] = seq
            if valido:
                nuevos[i] = crudo * 360.0 / CUENTAS_POR_VUELTA

        alguno = any(v is not None for v in nuevos)
        with self._lock:
            if alguno:
                self._lineas += 1
                self._ultima_lectura = time.monotonic()
            for i, v in enumerate(nuevos):
                if v is not None:
                    self._angulos[i] = v
                elif (i + 1) in self._ids:
                    self._errores[i] += 1      # se conserva el ultimo valor bueno
        return alguno

    # ------------------------------------------------------------ interfaz (igual que encoders.py)
    @property
    def angulos(self) -> list[float]:
        with self._lock:
            return list(self._angulos)

    @property
    def datos_frescos(self) -> bool:
        with self._lock:
            return (time.monotonic() - self._ultima_lectura) < UMBRAL_DATOS_FRESCOS_S

    @property
    def alguna_vez_datos(self) -> bool:
        with self._lock:
            return self._ultima_lectura > 0.0

    @property
    def timestamp_ultima_lectura(self) -> float:
        """time.monotonic() de la ultima ronda con al menos un valor bueno."""
        with self._lock:
            return self._ultima_lectura

    @property
    def frecuencia_hz(self) -> float:
        """Rondas por segundo (promedio movil). 0 mientras no haya datos."""
        with self._lock:
            return 1.0 / self._periodo_s if self._periodo_s > 0.0 else 0.0

    def reiniciar_estadisticas(self):
        with self._lock:
            self._errores = [0] * NUM_ENCODERS
            self._lineas = 0

    @property
    def estadisticas_errores(self):
        """(rondas con datos, errores por canal) desde el ultimo reinicio."""
        with self._lock:
            return self._lineas, list(self._errores)

    def cerrar(self):
        self._activo = False
        self._hilo.join(timeout=2)
        self._serial.close()
