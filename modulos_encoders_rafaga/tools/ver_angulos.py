import time
from encoders_modbus import LectorEncoders

lector = LectorEncoders("COM11", 230400, ids=(1, 2))
try:
    while True:
        print(["%.1f" % a for a in lector.angulos[:2]], "%.0f Hz" % lector.frecuencia_hz)
        time.sleep(0.2)
except KeyboardInterrupt:
    pass
lector.cerrar()