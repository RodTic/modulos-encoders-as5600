"""Prueba de encoders_modbus.LectorEncoders contra sim_slave.exe (el mismo C del firmware).

    gcc -Wall -Wextra -std=c11 -IApp/encoder_node/Inc tools/sim_slave.c App/encoder_node/Src/mb_slave.c App/encoder_node/Src/enc_regs.c -o tools/sim_slave.exe
    python tools/test_encoders_modbus.py

No mide velocidad (el simulador no tiene USB ni cable): eso se hace con "enc_tool.py bench" en la placa.
"""
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import enc_tool                       # noqa: E402  (SimPort, Bus)
import encoders_modbus as em          # noqa: E402

EXE = os.path.join(HERE, "sim_slave.exe")
fallas = []


def check(cond, msg):
    print(("ok     " if cond else "FALLA  ") + msg)
    if not cond:
        fallas.append(msg)


def esperar_hasta(cond, timeout=3.0):
    fin = time.monotonic() + timeout
    while time.monotonic() < fin:
        if cond():
            return True
        time.sleep(0.005)
    return False


class PuertoPrueba:
    """Envuelve SimPort: cuenta consultas por nodo y permite silenciar nodos o perder broadcasts."""

    def __init__(self, sim):
        self.sim = sim
        self.silenciados = set()
        self.perder_broadcast = False
        self.consultas = {}
        self.escrituras = []              # (id, funcion) de todo lo que se envio

    def reset_input_buffer(self):
        self.sim.reset_input_buffer()

    def flush(self):
        pass

    def write(self, data):
        nodo, fc = data[0], data[1]
        self.escrituras.append((nodo, fc))
        self.consultas[nodo] = self.consultas.get(nodo, 0) + 1
        if nodo == 0 and self.perder_broadcast:
            self.sim.buf = b""
            return
        self.sim.write(data)
        if nodo in self.silenciados:
            self.sim.buf = b""

    def read(self, n):
        return self.sim.read(n)

    def close(self):
        self.sim.close()


def angulo(cuentas):
    return cuentas * 360.0 / 4096.0


def cerca(a, b):
    return all(abs(x - y) < 1e-9 for x, y in zip(a, b))


def nuevo(ids=(1, 2, 3, 4, 5, 6), **kw):
    sim = enc_tool.SimPort(EXE, list(ids))
    return sim, PuertoPrueba(sim), kw


def main():
    if not os.path.exists(EXE):
        print("falta %s (ver el comando de compilacion arriba)" % EXE)
        return 1

    # --- 1) lectura normal de los 6, en grados y crudos
    sim, port, _ = nuevo()
    crudos = [100, 1024, 2048, 3000, 4000, 4095]
    for i, c in enumerate(crudos):
        sim.set_raw(i, c)
    lector = em.LectorEncoders(port, 921600)
    check(esperar_hasta(lambda: lector.alguna_vez_datos), "llegan datos")
    check(esperar_hasta(lambda: not any(math.isnan(a) for a in lector.angulos)), "los 6 canales tienen valor")
    check(cerca(lector.angulos, [angulo(c) for c in crudos]), "grados = crudo*360/4096 (0..360)")
    check(lector.datos_frescos, "datos_frescos")
    check(esperar_hasta(lambda: lector.frecuencia_hz > 0), "frecuencia_hz medida")
    lineas, errores = lector.estadisticas_errores
    check(lineas > 0 and errores == [0] * 6, "rondas contadas y cero errores")
    lector.reiniciar_estadisticas()
    check(lector.estadisticas_errores == (0, [0] * 6), "reiniciar_estadisticas")
    check(not any(fc in (6,) and nodo != 0 for nodo, fc in port.escrituras),
          "solo escribe el broadcast de snapshot (nunca toca las placas)")
    lector._activo = False
    lector._hilo.join(timeout=2)
    check(not lector._hilo.is_alive(), "el hilo termina")
    sim.close()

    # --- 2) un offset y un sentido guardados en la placa se ignoran: sale el crudo
    sim, port, _ = nuevo()
    bus = enc_tool.Bus(sim, retries=0)
    bus.write(1, enc_tool.HREG_OFFSET, 1000)
    bus.write(1, enc_tool.HREG_DIR, 1)
    sim.set_raw(0, 700)
    lector = em.LectorEncoders(port, 921600)
    esperar_hasta(lambda: not math.isnan(lector.angulos[0]))
    check(abs(lector.angulos[0] - angulo(700)) < 1e-9, "placa con offset 1000 y sentido invertido: igual sale el crudo")
    lector._activo = False
    lector._hilo.join(timeout=2)
    check(bus.read_holding(1, enc_tool.HREG_OFFSET, 2) == [1000, 1], "el lector no cambio offset ni sentido")
    sim.close()

    # --- 3) un nodo que nunca existio: error, no frena a los demas, se salta
    sim, port, _ = nuevo(ids=(1, 2, 3, 4, 5))
    for i in range(5):
        sim.set_raw(i, 500 + i)
    lector = em.LectorEncoders(port, 921600)
    esperar_hasta(lambda: lector.estadisticas_errores[0] >= 20)
    ang = lector.angulos
    check(math.isnan(ang[5]) and not any(math.isnan(a) for a in ang[:5]), "nodo 6 ausente: sin valor, los otros siguen")
    check(lector.estadisticas_errores[1][5] >= 20, "se cuenta un error por ronda del nodo 6")
    check(port.consultas.get(6, 0) <= 4, "nodo caido se salta (consultas al nodo 6: %d)" % port.consultas.get(6, 0))
    lector.cerrar()

    # --- 4) un nodo deja de responder: conserva el ultimo valor bueno
    sim, port, _ = nuevo()
    for i in range(6):
        sim.set_raw(i, 800 + i)
    lector = em.LectorEncoders(port, 921600)
    esperar_hasta(lambda: not any(math.isnan(a) for a in lector.angulos))
    antes = lector.angulos[2]
    port.silenciados.add(3)
    sim.set_raw(2, 1500)
    e0 = lector.estadisticas_errores[1][2]
    esperar_hasta(lambda: lector.estadisticas_errores[1][2] > e0 + 5)
    check(lector.angulos[2] == antes, "nodo 3 sin respuesta: conserva el ultimo valor bueno")
    lector._activo = False
    lector._hilo.join(timeout=2)
    sim.close()

    # --- 5) iman debil o ausente: error y conserva el valor
    sim, port, _ = nuevo()
    for i in range(6):
        sim.set_raw(i, 900 + i)
    lector = em.LectorEncoders(port, 921600)
    esperar_hasta(lambda: not any(math.isnan(a) for a in lector.angulos))
    antes = lector.angulos[1]
    sim.set_mag(1, 0x10)                  # ML: iman debil
    sim.set_raw(1, 2000)
    e0 = lector.estadisticas_errores[1][1]
    esperar_hasta(lambda: lector.estadisticas_errores[1][1] > e0 + 5)
    check(lector.angulos[1] == antes, "iman debil: error y conserva el valor")
    sim.set_mag(1, 0x20)                  # vuelve el iman
    esperar_hasta(lambda: abs(lector.angulos[1] - angulo(2000)) < 1e-9)
    check(abs(lector.angulos[1] - angulo(2000)) < 1e-9, "al volver el iman se recupera el valor nuevo")
    lector._activo = False
    lector._hilo.join(timeout=2)
    sim.close()

    # --- 6) broadcast perdido: las placas no toman snapshot, no se aceptan datos viejos
    sim, port, _ = nuevo()
    for i in range(6):
        sim.set_raw(i, 1100 + i)
    port.perder_broadcast = True
    lector = em.LectorEncoders(port, 921600)
    time.sleep(0.3)
    check(not lector.alguna_vez_datos and all(math.isnan(a) for a in lector.angulos),
          "sin snapshot (seq 0) no se aceptan lecturas")
    port.perder_broadcast = False
    check(esperar_hasta(lambda: not any(math.isnan(a) for a in lector.angulos)), "al volver el broadcast hay datos")
    lector._activo = False
    lector._hilo.join(timeout=2)
    sim.close()

    # --- 7) sincronizar=False: lectura en vivo, sin broadcast
    sim, port, _ = nuevo()
    for i in range(6):
        sim.set_raw(i, 1200 + 10 * i)
    lector = em.LectorEncoders(port, 230400, sincronizar=False)
    check(esperar_hasta(lambda: not any(math.isnan(a) for a in lector.angulos)), "sin snapshot: hay datos")
    check(cerca(lector.angulos, [angulo(1200 + 10 * i) for i in range(6)]), "sin snapshot: mismos grados")
    check(all(nodo != 0 for nodo, _ in port.escrituras), "sin snapshot: no hay broadcast")
    lector.cerrar()

    # --- 8) solo algunas placas conectadas (ids): no se consulta a las demas ni suman errores
    sim, port, _ = nuevo(ids=(1, 2))
    sim.set_raw(0, 512)
    sim.set_raw(1, 1536)
    lector = em.LectorEncoders(port, 921600, ids=(1, 2))
    check(esperar_hasta(lambda: not any(math.isnan(a) for a in lector.angulos[:2])), "ids=(1,2): hay datos de las 2 placas")
    ang = lector.angulos
    check(abs(ang[0] - angulo(512)) < 1e-9 and abs(ang[1] - angulo(1536)) < 1e-9, "ids=(1,2): grados correctos")
    check(all(math.isnan(a) for a in ang[2:]), "ids=(1,2): los otros 4 canales quedan en NaN")
    check(lector.estadisticas_errores[1] == [0] * 6, "ids=(1,2): ningun error por las placas que no existen")
    check(all(port.consultas.get(n, 0) == 0 for n in (3, 4, 5, 6)), "ids=(1,2): no se consulta a las placas 3 a 6")
    lector.cerrar()
    try:
        em.LectorEncoders(PuertoPrueba(enc_tool.SimPort(EXE, [1])), 921600, ids=(0, 7)).cerrar()
        check(False, "ids fuera de 1..6 debe dar ValueError")
    except ValueError:
        check(True, "ids fuera de 1..6 da ValueError")

    # --- 9) una placa ausente cuesta UNA consulta fallida por reintento, no dos
    sim, port, _ = nuevo(ids=(1, 2, 3, 4, 5))
    for i in range(5):
        sim.set_raw(i, 700 + i)
    saltar_original = em.SALTAR_S
    em.SALTAR_S = 0.2
    try:
        t0 = time.monotonic()
        lector = em.LectorEncoders(port, 921600)
        time.sleep(1.1)
        transcurrido = time.monotonic() - t0
        lector.cerrar()
    finally:
        em.SALTAR_S = saltar_original
    esperadas = 2 + int(transcurrido / 0.2) + 1        # 2 fallos iniciales + 1 reintento por periodo
    check(port.consultas.get(6, 0) <= esperadas,
          "placa ausente: %d consultas al nodo 6 en %.1f s (maximo %d; con 2 por reintento serian ~%d)"
          % (port.consultas.get(6, 0), transcurrido, esperadas, 2 + 2 * int(transcurrido / 0.2)))

    # --- 9b) placas que responden pero sin iman: no se frena el ciclo (solo se duerme si nadie responde)
    sim, port, _ = nuevo(ids=(1, 2))
    sim.set_mag(0, 0)
    sim.set_mag(1, 0)                                  # sin iman en las dos
    lector = em.LectorEncoders(port, 921600, ids=(1, 2))
    time.sleep(0.6)
    check(all(math.isnan(a) for a in lector.angulos), "sin iman y exigir_iman=True: no se acepta ninguna lectura")
    check(lector.frecuencia_hz > 120, "responden sin iman: la ronda no se frena (%.0f Hz, con pausa de 10 ms serian < 100)"
          % lector.frecuencia_hz)
    lector.cerrar()

    # --- 9c) exigir_iman=False acepta el angulo aunque el AS5600 no detecte iman
    sim, port, _ = nuevo(ids=(1, 2))
    sim.set_mag(0, 0x10)                               # iman debil
    sim.set_mag(1, 0)                                  # sin iman
    sim.set_raw(0, 1024)
    sim.set_raw(1, 2048)
    lector = em.LectorEncoders(port, 921600, ids=(1, 2), exigir_iman=False)
    check(esperar_hasta(lambda: not any(math.isnan(a) for a in lector.angulos[:2])), "exigir_iman=False: hay datos sin iman")
    check(abs(lector.angulos[0] - 90.0) < 1e-9 and abs(lector.angulos[1] - 180.0) < 1e-9,
          "exigir_iman=False: acepta iman debil y ausente")
    lector.cerrar()

    # --- 10) esperas calculadas igual que el firmware
    check(abs(em.silencio_nodo_s(921600) - 500e-6) < 1e-12 and abs(em.silencio_nodo_s(230400) - 500e-6) < 1e-12,
          "silencio 500 us a >= 230400")
    check(abs(em.silencio_nodo_s(115200) - 1750e-6) < 1e-12, "silencio 1750 us a 115200")
    check(0.0025 <= em.espera_tras_broadcast_s(921600) < 0.004 and 0.0025 <= em.espera_tras_broadcast_s(230400) < 0.004,
          "espera tras broadcast entre 2.5 y 4 ms (la que dio 0 errores en la placa)")
    check(all(em.pausa_entre_tramas_s(b) > em.silencio_nodo_s(b) + 0.0003 for b in (115200, 230400, 921600)),
          "la pausa entre tramas supera el silencio de fin de trama del nodo (si no, el nodo siguiente ignora la peticion)")

    print("%d falla(s)" % len(fallas))
    return 1 if fallas else 0


if __name__ == "__main__":
    sys.exit(main())
