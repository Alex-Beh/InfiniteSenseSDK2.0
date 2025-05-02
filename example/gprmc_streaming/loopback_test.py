# Loopback Test

import serial
import time

ser = serial.Serial('/dev/ttyUSB0',      # or /dev/ttyUSB0
                    # baudrate=9600,       # <-- set to the same value you used
                    baudrate=115200,       # <-- set to the same value you used
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=1)
print(f"Listening on {ser.port} @ {ser.baudrate} baud – Ctrl‑C to quit")

# --- Read forever -----------------------------------------------------
try:
    while True:
        ser.write(b'>')        # send the character '>'
        time.sleep(1.00)
        print("echo:", ser.read(1))
except KeyboardInterrupt:
    pass
finally:
    ser.close()
    print("\nSerial port closed – bye.")