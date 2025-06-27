import serial
from time import sleep

ser = serial.Serial('/dev/ttyUSB0',      # or /dev/ttyUSB0
                    # baudrate=9600,       # <-- set to the same value you used
                    baudrate=115200,       # <-- set to the same value you used
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=1)

def read_vendor_nmea(ser):
    raw = ser.readline()                      # blocks until LF
    # if raw:
    #     print("RAW :", raw.hex(" "))

    if not raw or raw[:1] != b'$' or len(raw) < 6:
        return None, False

    raw = raw.rstrip(b'\r\n')                 # drop LF/CR

    star = raw.rfind(b'*')                    # <─ find the asterisk
    if star == -1 or len(raw) - star < 3:     # need at least '*XX'
        return None, False

    payload = raw[1:star]                     # bytes between $ and *
    cs_hex  = raw[star+1:star+3]              # the two ASCII hex digits

    try:
        cs_rx = int(cs_hex, 16)
    except ValueError:
        return None, False

    cs_calc = 0
    for b in payload:
        cs_calc ^= b

    nmea = b'$' + payload + b'*' + cs_hex + b'\r\n'
    return nmea.decode('ascii', errors='replace'), cs_calc == cs_rx


while True:
    nmea, ok = read_vendor_nmea(ser)
    if nmea:
        print("✓" if ok else "✗", nmea)
    else:
        print("waiting …", end='\r')
        sleep(0.1)
