import serial
import time
import struct

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1.0)
time.sleep(0.5)
ser.reset_input_buffer()
ser.reset_output_buffer()

print("Sending window config...")
cmd = b'W' + struct.pack('<II', 100, 1000)
ser.write(cmd)
ser.flush()

start = time.time()
while time.time() - start < 3.0:
    res = ser.read(100)
    if res:
        print(f"Received {len(res)} bytes: {res.hex()}")
    time.sleep(0.05)

ser.close()
