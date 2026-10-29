#!/usr/bin/env python3
"""Herramienta de laptop para los modulos encoder (Modbus RTU sobre RS-485).

Solo necesita pyserial (pip install pyserial). Ejemplos (COM5 = tu adaptador USB-RS485):

  python enc_tool.py -p COM5 scan                      # busca nodos (una placa nueva es ID 247)
  python enc_tool.py -p COM5 set-id 247 1 --save       # darle ID 1 (conecta UNA placa nueva a la vez)
  python enc_tool.py -p COM5 read 1                    # ver todos sus registros
  python enc_tool.py -p COM5 set-offset 1 1500 --save
  python enc_tool.py -p COM5 set-zero 1 --save         # la posicion actual pasa a ser 0
  python enc_tool.py -p COM5 set-dir 1 1 --save        # invertir el sentido
  python enc_tool.py -p COM5 poll 1 2 3 4 5 6 --rate 20   # lee sin parar; ESPACIO pausa/continua, Q sale
  python enc_tool.py -p COM5 poll 1 2 3 4 5 6 --rate 50 --csv > datos.csv
  python enc_tool.py selftest                          # prueba sin hardware
"""
import argparse
import contextlib
import io
import os
import struct
import subprocess
import sys
import threading
import time

HREG_OFFSET, HREG_DIR, HREG_ID, HREG_CMD = 0, 1, 2, 3
CMD_SAVE, CMD_RESTORE, CMD_SET_ZERO, CMD_RESET, CMD_SNAPSHOT = 1, 2, 3, 4, 5
IREG_ANGLE_CAL, IREG_ANGLE_RAW, IREG_STATUS, IREG_COUNT, IREG_SNAP, IREG_SNAP_SEQ = 0, 1, 2, 3, 4, 5
IREG_UID0, IREG_FW = 6, 12
IREG_SNAP_RAW = 13          # 13 = crudo congelado, 14 = estado, 15 = numero de snapshot
DEFAULT_ID = 247
SAVE_WAIT_S = 0.15          # el nodo no responde ~40 ms mientras borra/escribe flash
STATUS_BITS = [(0x01, "iman detectado"), (0x02, "iman debil"), (0x04, "iman fuerte"),
               (0x08, "ERROR I2C"), (0x10, "valores por defecto (sin guardar)"),
               (0x20, "FALLO AL GUARDAR")]


class ModbusError(Exception):
    pass


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def with_crc(frame):
    return frame + struct.pack("<H", crc16(frame))


def silencio_nodo_s(baud):
    """Silencio con el que la placa da por terminada una trama (igual que ENC_GAP_US del firmware)."""
    if baud >= 230400:
        return 500e-6
    if baud > 19200:
        return 1750e-6
    return 38.5 / baud


def espera_tras_broadcast_s(baud):
    """Minimo entre un broadcast y la siguiente consulta: sale la trama (8 bytes), la placa
    detecta el fin de trama, termina su lectura I2C en curso y toma otra fresca. Los 2 ms de
    margen se midieron en una placa real (firmware Debug): con 2.5 ms de espera total a 230400
    hubo 0 errores en 400 rondas; con 2.0 ms, 3 %."""
    return 80.0 / baud + silencio_nodo_s(baud) + 0.0020


def pausa_entre_tramas_s(baud):
    """Silencio que el maestro deja tras una respuesta antes de la siguiente peticion. Los demas nodos
    oyen todo el bus: si entre la respuesta del nodo 1 y la peticion al nodo 2 hay menos silencio que
    el fin de trama (silencio_nodo_s), el nodo 2 ve una sola trama larga y la ignora. Medido con 2
    placas a 230400 (pausa contada desde que el PC recibe la respuesta): 0 us -> 87 % de fallos,
    400 us -> 4 %, 600 us -> 1 %, 800 us -> 0 %. El margen de 0.4 ms sobre el silencio da 0.9 ms."""
    return silencio_nodo_s(baud) + 0.0004


def esperar(segundos):
    """Espera precisa para tiempos cortos (time.sleep solo es exacto a ~1 ms en Windows)."""
    fin = time.perf_counter() + segundos
    if segundos > 0.003:
        time.sleep(segundos - 0.002)
    while time.perf_counter() < fin:
        pass


class Bus:
    """port: cualquier objeto con write(bytes), read(n), reset_input_buffer(), flush()."""

    def __init__(self, port, retries=2, baud=115200):
        self.port = port
        self.retries = retries
        self.baud = baud
        self._pausa = pausa_entre_tramas_s(baud)
        self._libre = 0.0                     # perf_counter desde el que se puede volver a hablar

    def _esperar_silencio(self):
        falta = self._libre - time.perf_counter()
        if falta > 0:
            esperar(falta)

    def _txrx(self, req, expect_len):
        err = "sin respuesta (timeout)"
        for _ in range(self.retries + 1):
            self._esperar_silencio()
            self.port.reset_input_buffer()
            self.port.write(req)
            self.port.flush()
            head = self.port.read(5)
            if len(head) < 5:
                self._libre = time.perf_counter() + self._pausa
                err = "sin respuesta (timeout)"
                continue
            total = 5 if head[1] & 0x80 else expect_len
            resp = head + self.port.read(total - 5)
            self._libre = time.perf_counter() + self._pausa
            if len(resp) < total or crc16(resp[:-2]) != struct.unpack("<H", resp[-2:])[0]:
                err = "respuesta incompleta o con CRC mala"
                continue
            if resp[0] != req[0] or (resp[1] & 0x7F) != req[1]:
                err = "respuesta de otro nodo o funcion"
                continue
            if resp[1] & 0x80:
                raise ModbusError("excepcion Modbus %d del nodo %d" % (resp[2], req[0]))
            return resp
        raise ModbusError("nodo %d: %s" % (req[0], err))

    def read(self, node, fc, addr, qty):
        resp = self._txrx(with_crc(struct.pack(">BBHH", node, fc, addr, qty)), 5 + 2 * qty)
        return list(struct.unpack(">%dH" % qty, resp[3:-2]))

    def read_holding(self, node, addr, qty):
        return self.read(node, 3, addr, qty)

    def read_input(self, node, addr, qty):
        return self.read(node, 4, addr, qty)

    def write(self, node, addr, value):
        self._txrx(with_crc(struct.pack(">BBHH", node, 6, addr, value)), 8)

    def command(self, node, cmd):
        self.write(node, HREG_CMD, cmd)
        if cmd in (CMD_SAVE, CMD_RESET):
            time.sleep(SAVE_WAIT_S)

    def broadcast(self, addr, value):
        """Escritura a todos (id 0): nadie responde."""
        self._esperar_silencio()
        self.port.reset_input_buffer()
        self.port.write(with_crc(struct.pack(">BBHH", 0, 6, addr, value)))
        self.port.flush()
        esperar(espera_tras_broadcast_s(self.baud))     # que los nodos terminen su lectura fresca

    def snapshot(self):
        self.broadcast(HREG_CMD, CMD_SNAPSHOT)


def deg(counts):
    return counts * 360.0 / 4096.0


def describe_status(st):
    return ", ".join(name for bit, name in STATUS_BITS if st & bit) or "-"


def cmd_read(bus, a):
    h = bus.read_holding(a.id, 0, 3)
    i = bus.read_input(a.id, 0, 13)
    uid = "".join("%04X" % w for w in i[IREG_UID0:IREG_UID0 + 6])
    print("nodo %d  firmware %x.%02x  UID %s" % (a.id, i[IREG_FW] >> 8, i[IREG_FW] & 0xFF, uid))
    print("  offset=%d  sentido=%d  id=%d" % tuple(h))
    print("  angulo calibrado=%d (%.2f deg)  crudo=%d" % (i[0], deg(i[0]), i[1]))
    print("  estado=0x%02X [%s]  muestras=%d" % (i[2], describe_status(i[2]), i[3]))
    print("  snapshot=%d (seq %d)" % (i[IREG_SNAP], i[IREG_SNAP_SEQ]))


def cmd_scan(bus, a):
    bus.retries = 0
    found = 0
    for node in [DEFAULT_ID] + list(range(1, 247)):
        try:
            fw = bus.read_input(node, IREG_FW, 1)[0]
        except ModbusError:
            continue
        found += 1
        uid = bus.read_input(node, IREG_UID0, 6)
        print("nodo %3d  firmware %x.%02x  UID %s" % (node, fw >> 8, fw & 0xFF, "".join("%04X" % w for w in uid)))
    print("%d nodo(s) encontrado(s)" % found)


def maybe_save(bus, node, a):
    if a.save:
        bus.command(node, CMD_SAVE)
        print("guardado en flash")


def cmd_set_id(bus, a):
    bus.write(a.old, HREG_ID, a.new)
    time.sleep(0.05)
    print("nodo %d ahora responde como %d" % (a.old, a.new))
    maybe_save(bus, a.new, a)


def cmd_set_offset(bus, a):
    bus.write(a.id, HREG_OFFSET, a.value)
    maybe_save(bus, a.id, a)


def cmd_set_dir(bus, a):
    bus.write(a.id, HREG_DIR, a.value)
    maybe_save(bus, a.id, a)


def cmd_set_zero(bus, a):
    bus.command(a.id, CMD_SET_ZERO)
    print("offset nuevo = %d" % bus.read_holding(a.id, HREG_OFFSET, 1)[0])
    maybe_save(bus, a.id, a)


def cmd_simple(cmd):
    return lambda bus, a: bus.command(a.id, cmd)


TECLA_PAUSA = " "                       # barra espaciadora: pausa / continua
TECLAS_SALIR = ("q", "Q", "\x1b", "\x03")   # Q, Esc o Ctrl+C


def leer_tecla():
    """Tecla pulsada o None, sin bloquear. Solo funciona en Windows (msvcrt); en otros sistemas
    devuelve siempre None y poll se detiene con Ctrl+C."""
    try:
        import msvcrt
    except ImportError:
        return None
    if not msvcrt.kbhit():
        return None
    c = msvcrt.getwch()
    if c in ("\x00", "\xe0"):           # tecla especial (flechas, F1...): trae un segundo codigo
        msvcrt.getwch()
        return None
    return c


def cmd_poll(bus, a, leer_tecla=leer_tecla):
    """Snapshot por broadcast y lectura del angulo congelado de cada nodo, sin parar hasta que lo
    detengas (o hasta --count rondas, si lo pasas). ESPACIO pausa y vuelve a continuar (en pausa no
    se manda nada al bus); Q, Esc o Ctrl+C salen. Los avisos van por stderr para no ensuciar un CSV."""
    aviso = sys.stderr
    limite = "se detiene tras %d rondas" % a.count if a.count > 0 else "sin limite de rondas"
    print("poll: ESPACIO = pausar/continuar, Q = salir (%s)" % limite, file=aviso, flush=True)
    if a.csv:
        print("t_s," + ",".join("id%d_deg" % n for n in a.ids) + ",seq", flush=True)
    period = 1.0 / a.rate if a.rate > 0 else 0
    t0 = time.time()
    pausado_desde, pausa_total, k = None, 0.0, 0
    try:
        while a.count <= 0 or k < a.count:
            salir = False
            while True:                                  # todas las teclas que haya pendientes
                c = leer_tecla()
                if c is None:
                    break
                if c == TECLA_PAUSA:
                    if pausado_desde is None:
                        pausado_desde = time.time()
                        print("[PAUSA] ESPACIO para continuar", file=aviso, flush=True)
                    else:
                        pausa_total += time.time() - pausado_desde
                        pausado_desde = None
                        print("[CONTINUA]", file=aviso, flush=True)
                elif c in TECLAS_SALIR:
                    salir = True
            if salir:
                break
            if pausado_desde is not None:
                time.sleep(0.03)
                continue

            t_start = time.time()
            bus.snapshot()
            row, seqs = [], set()
            for node in a.ids:
                try:
                    ang, seq = bus.read_input(node, IREG_SNAP, 2)
                    row.append(deg(ang))
                    seqs.add(seq)
                except ModbusError:
                    row.append(float("nan"))
            note = "" if len(seqs) <= 1 else "  (!) snapshots distintos %s" % sorted(seqs)
            t = time.time() - t0 - pausa_total           # el tiempo no cuenta las pausas
            if a.csv:
                print("%.4f," % t + ",".join("%.3f" % v for v in row) + ",%s" % (sorted(seqs)[0] if seqs else ""),
                      flush=True)
            else:
                print("t=%7.3f  " % t + "  ".join("%d:%7.2f" % (n, v) for n, v in zip(a.ids, row)) + note,
                      flush=True)
            k += 1
            sleep = period - (time.time() - t_start)
            if sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        pass
    print("poll detenido tras %d rondas" % k, file=aviso, flush=True)


def cmd_bench(bus, a):
    """Mide cuantas rondas por segundo logra tu adaptador con estas placas (misma ronda que
    encoders_modbus.py: snapshot por broadcast + una consulta FC 04 por placa)."""
    bus.retries = 0
    direccion, cantidad = (IREG_ANGLE_RAW, 2) if a.no_sync else (IREG_SNAP_RAW, 3)
    errores = {n: 0 for n in a.ids}
    t0 = time.perf_counter()
    for _ in range(a.count):
        if not a.no_sync:
            bus.snapshot()
        for n in a.ids:
            try:
                bus.read_input(n, direccion, cantidad)
            except ModbusError:
                errores[n] += 1
    dt = time.perf_counter() - t0
    print("%d rondas en %.2f s: %.1f rondas/s (%.1f ms por ronda) a %d baudios, %s" % (
        a.count, dt, a.count / dt, 1000.0 * dt / a.count, bus.baud,
        "sin snapshot" if a.no_sync else "con snapshot"))
    print("errores por nodo: " + "  ".join("%d:%d" % (n, e) for n, e in errores.items()))


# ---------------------------------------------------------------- autoprueba
class SimPort:
    """Habla con tools/sim_slave.exe (mismo C del firmware) por un pipe."""

    def __init__(self, exe, ids):
        self.p = subprocess.Popen([exe] + [str(i) for i in ids], stdin=subprocess.PIPE,
                                  stdout=subprocess.PIPE, text=True, bufsize=1)
        self.buf = b""
        self._lock = threading.Lock()     # una pregunta y su respuesta a la vez (el lector corre en otro hilo)

    def _ask(self, line):
        with self._lock:
            self.p.stdin.write(line + "\n")
            self.p.stdin.flush()
            return self.p.stdout.readline().strip()

    def set_raw(self, idx, value):
        assert self._ask("RAW %d %d" % (idx, value)) == "OK"

    def set_mag(self, idx, value):
        assert self._ask("MAG %d %d" % (idx, value)) == "OK"

    def reset_input_buffer(self):
        self.buf = b""

    def write(self, data):
        r = self._ask("RX " + data.hex())
        self.buf = b"" if r == "TX -" else bytes.fromhex(r[3:])

    def read(self, n):
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def flush(self):
        pass

    def close(self):
        self.p.stdin.close()
        self.p.wait()


def selftest(a):
    fails = []

    def check(cond, msg):
        print(("ok     " if cond else "FALLA  ") + msg)
        if not cond:
            fails.append(msg)

    check(crc16(bytes.fromhex("01030000000A")) == 0xCDC5, "CRC 01 03 00 00 00 0A -> C5 CD")
    check(crc16(bytes.fromhex("010600010003")) == 0x0B98, "CRC 01 06 00 01 00 03 -> 98 0B")
    check(with_crc(bytes.fromhex("F70300000004")).hex() == "f70300000004509f", "with_crc agrega la CRC byte bajo primero")

    exe = a.sim or os.path.join(os.path.dirname(os.path.abspath(__file__)), "sim_slave.exe")
    if not os.path.exists(exe):
        print("sin %s: se omite la prueba contra el C del firmware (compilalo, ver sim_slave.c)" % exe)
        return 1 if fails else 0

    # Nodo de fabrica (ID 247)
    sim = SimPort(exe, [DEFAULT_ID])
    bus = Bus(sim, retries=0)
    check(bus.read_input(247, IREG_FW, 1) == [0x0100], "firmware 1.00 por FC04")
    sim.set_raw(0, 100)
    bus.write(247, HREG_OFFSET, 100)
    check(bus.read_input(247, IREG_ANGLE_CAL, 2) == [0, 100], "offset=100 con raw=100 -> calibrado 0")
    bus.write(247, HREG_DIR, 1)
    check(bus.read_input(247, IREG_ANGLE_CAL, 1) == [3896], "sentido invertido: (4096-100)-100 = 3896")
    bus.command(247, CMD_SET_ZERO)
    check(bus.read_input(247, IREG_ANGLE_CAL, 1) == [0] and bus.read_holding(247, HREG_OFFSET, 1) == [3996],
          "set-zero deja el calibrado en 0 (offset 3996)")
    try:
        bus.write(247, HREG_OFFSET, 5000)
        check(False, "offset 5000 debe dar excepcion")
    except ModbusError as e:
        check("excepcion Modbus 3" in str(e), "offset 5000 -> excepcion 3")
    bus.write(247, HREG_ID, 5)
    try:
        bus.read_input(5 + 1, IREG_FW, 1)
        check(False, "id inexistente no debe responder")
    except ModbusError:
        check(True, "un ID ajeno no responde")
    check(bus.read_input(5, IREG_FW, 1) == [0x0100], "tras set-id el nodo responde como 5")
    sim.close()

    # Seis nodos en el mismo bus: snapshot sincronizado
    sim = SimPort(exe, [1, 2, 3, 4, 5, 6])
    bus = Bus(sim, retries=0)
    for k in range(6):
        sim.set_raw(k, 100 * (k + 1))
    bus.snapshot()
    for k in range(6):
        sim.set_raw(k, 3000)                  # el brazo se mueve despues del snapshot
    snaps = [bus.read_input(n, IREG_SNAP, 2) for n in range(1, 7)]
    check([s[0] for s in snaps] == [100 * (k + 1) for k in range(6)], "snapshot congela los 6 angulos")
    check(len({s[1] for s in snaps}) == 1, "los 6 nodos tienen el mismo SNAP_SEQ")
    check(bus.read_input(3, IREG_ANGLE_CAL, 1) == [3000], "la lectura en vivo si sigue al angulo")
    sim.close()

    # poll: sin limite, ESPACIO pausa (sin trafico en el bus) y reanuda, Q sale; con --count se detiene solo
    sim = SimPort(exe, [1, 2])
    for k in range(2):
        sim.set_raw(k, 512 * (k + 1))

    class PuertoContado:
        def __init__(self, s):
            self.s, self.escrituras = s, 0

        def reset_input_buffer(self):
            self.s.reset_input_buffer()

        def write(self, d):
            self.escrituras += 1
            self.s.write(d)

        def read(self, n):
            return self.s.read(n)

        def flush(self):
            pass

    pc = PuertoContado(sim)
    pbus = Bus(pc, retries=0)
    est = {"fase": 0, "t": 0.0, "al_pausar": 0, "al_seguir": 0, "inicio": time.time()}

    def teclas():
        if time.time() - est["inicio"] > 8:                      # seguridad: no colgar la prueba
            return "q"
        e = pc.escrituras
        if est["fase"] == 0 and e >= 20:
            est.update(fase=1, t=time.time(), al_pausar=e)
            return " "                                           # pausa
        if est["fase"] == 1 and time.time() - est["t"] > 0.3:
            est.update(fase=2, al_seguir=pc.escrituras)
            return " "                                           # continua
        if est["fase"] == 2 and pc.escrituras >= est["al_seguir"] + 15:
            est["fase"] = 3
            return "q"
        return None

    args = argparse.Namespace(ids=[1, 2], count=0, rate=0, csv=False)
    sal, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(sal), contextlib.redirect_stderr(err):
        cmd_poll(pbus, args, leer_tecla=teclas)
    check(est["fase"] == 3, "poll sin limite: pausa con ESPACIO, continua con ESPACIO y sale con Q")
    check(est["al_seguir"] == est["al_pausar"], "en pausa no se manda nada al bus")
    check("[PAUSA]" in err.getvalue() and "[CONTINUA]" in err.getvalue(), "avisa de la pausa y la reanudacion")
    filas = [l for l in sal.getvalue().splitlines() if l.startswith("t=")]
    # cada ronda son 3 escrituras (snapshot + 2 lecturas): 20 antes de la pausa y 15 despues = ~12 filas
    check(len(filas) >= 10 and "1:" in filas[0] and "2:" in filas[0], "poll imprime las lecturas (%d filas)" % len(filas))

    args = argparse.Namespace(ids=[1, 2], count=3, rate=0, csv=True)
    sal, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(sal), contextlib.redirect_stderr(err):
        cmd_poll(pbus, args, leer_tecla=lambda: None)
    lineas = sal.getvalue().splitlines()
    check(len(lineas) == 4 and lineas[0].startswith("t_s,") and "ESPACIO" not in sal.getvalue(),
          "con --count 3 se detiene solo y el CSV sale limpio (avisos por stderr)")
    sim.close()

    print("%d falla(s)" % len(fails))
    return 1 if fails else 0


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description="Calibracion y lectura de los modulos encoder (Modbus RTU).")
    ap.add_argument("-p", "--port", help="puerto serie, ej. COM5")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=0.05, help="timeout de lectura en s")
    sub = ap.add_subparsers(dest="cmd", required=True)

    def add(name, fn, *args, save=False):
        sp = sub.add_parser(name)
        for aname, kw in args:
            sp.add_argument(aname, **kw)
        if save:
            sp.add_argument("--save", action="store_true", help="guardar en flash despues")
        sp.set_defaults(fn=fn)
        return sp

    num = {"type": int}
    add("read", cmd_read, ("id", num))
    add("scan", cmd_scan)
    add("set-id", cmd_set_id, ("old", num), ("new", num), save=True)
    add("set-offset", cmd_set_offset, ("id", num), ("value", num), save=True)
    add("set-dir", cmd_set_dir, ("id", num), ("value", {"type": int, "choices": [0, 1]}), save=True)
    add("set-zero", cmd_set_zero, ("id", num), save=True)
    add("save", cmd_simple(CMD_SAVE), ("id", num))
    add("restore", cmd_simple(CMD_RESTORE), ("id", num))
    add("reset", cmd_simple(CMD_RESET), ("id", num))
    add("snapshot", lambda bus, a: bus.snapshot())
    sp = add("poll", cmd_poll, ("ids", {"type": int, "nargs": "+"}))
    sp.add_argument("--count", type=int, default=0, help="rondas a leer; 0 = sin limite (por defecto)")
    sp.add_argument("--rate", type=float, default=20, help="rondas por segundo (0 = lo mas rapido)")
    sp.add_argument("--csv", action="store_true")
    sp = add("bench", cmd_bench, ("ids", {"type": int, "nargs": "+"}))
    sp.add_argument("--count", type=int, default=500)
    sp.add_argument("--no-sync", action="store_true", help="sin snapshot (lectura en vivo)")
    sp = sub.add_parser("selftest")
    sp.add_argument("--sim", help="ruta a sim_slave.exe")
    sp.set_defaults(fn=None)

    a = ap.parse_args()
    if a.cmd == "selftest":
        return selftest(a)
    if not a.port:
        ap.error("falta -p/--port")
    import serial
    with serial.Serial(a.port, a.baud, bytesize=8, parity="N", stopbits=1, timeout=a.timeout) as ser:
        try:
            a.fn(Bus(ser, baud=a.baud), a)
        except ModbusError as e:
            print("ERROR:", e)
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
