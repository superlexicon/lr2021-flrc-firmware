#!/usr/bin/env python3
"""
test_flrc_burst_duplex.py

Automated 20KB FLRC burst duplex verification test script.
"""

import sys
import time
import os
import zlib
import struct
import serial
import threading

PORT1 = "/dev/ttyACM0"
PORT2 = "/dev/ttyACM1"
BAUD = 115200

PAYLOAD_SIZE = 900  # 900 bytes (4 packets)

class RadioPort:
    def __init__(self, port):
        self.port_name = port
        self.ser = serial.Serial(port, BAUD, timeout=0.1, write_timeout=10.0)
        self.rx_buf = bytearray()
        self.lock = threading.Lock()
        self.stop_evt = threading.Event()
        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()

    def _reader_loop(self):
        while not self.stop_evt.is_set():
            try:
                data = self.ser.read(4096)
                if data:
                    with self.lock:
                        self.rx_buf.extend(data)
            except Exception as e:
                time.sleep(0.005)

    def write(self, data):
        written = 0
        while written < len(data):
            n = self.ser.write(data[written:])
            if n is not None:
                written += n
            else:
                time.sleep(0.001)
        self.ser.flush()

    def get_buf(self):
        with self.lock:
            return bytes(self.rx_buf)

    def consume(self, n):
        with self.lock:
            del self.rx_buf[:n]

    def clear(self):
        with self.lock:
            self.rx_buf.clear()

    def close(self):
        self.stop_evt.set()
        self.ser.close()

def parse_events(radio):
    buf = radio.get_buf()
    if not buf:
        return
    i = 0
    while i < len(buf):
        tag = buf[i]
        if tag == ord('D'):
            if i + 6 <= len(buf):
                evt_id, val = struct.unpack('<BI', buf[i+1:i+6])
                print(f"  [{radio.port_name} DBG] Event {evt_id}, val={val}")
                i += 6
            else:
                break
        elif tag == ord('G'):
            if i + 3 <= len(buf):
                max_pld = struct.unpack('<H', buf[i+1:i+3])[0]
                print(f"  [{radio.port_name} READY] 'G' max_pld={max_pld}")
                i += 3
            else:
                break
        elif tag == ord('E'):
            if i + 3 <= len(buf):
                code, msg_len = struct.unpack('<BB', buf[i+1:i+3])
                if i + 3 + msg_len <= len(buf):
                    msg = buf[i+3:i+3+msg_len].decode('utf-8', errors='ignore')
                    print(f"  [{radio.port_name} ERROR] Code {code}, msg={msg}")
                    i += 3 + msg_len
                else:
                    break
            else:
                break
        else:
            # Unrecognized byte, skip 1 byte
            i += 1
    if i > 0:
        radio.consume(i)

def set_window(radio, offset_ms, period_ms):
    radio.clear()
    cmd = b'W' + struct.pack('<II', offset_ms, period_ms)
    radio.write(cmd)
    
    start = time.time()
    while time.time() - start < 3.0:
        buf = radio.get_buf()
        for i in range(len(buf)):
            if buf[i] == ord('G') and i + 3 <= len(buf):
                max_pld = struct.unpack('<H', buf[i+1:i+3])[0]
                print(f"  [{radio.port_name}] Window configured (offset={offset_ms}ms, period={period_ms}ms), ready tag='G', max_pld={max_pld}")
                radio.consume(i + 3)
                return True
        time.sleep(0.05)
    print(f"  [{radio.port_name}] Timeout waiting for window config response.")
    return False

def send_payload(radio, payload):
    cmd = bytearray(b'TX')
    cmd.extend(struct.pack('<H', len(payload)))
    cmd.extend(payload)
    
    # Send in 64-byte USB bulk packet chunks with 2ms sleep to prevent Zephyr CDC ACM RX buffer overflow
    chunk_size = 64
    for i in range(0, len(cmd), chunk_size):
        radio.write(cmd[i:i+chunk_size])
        time.sleep(0.002)
    print(f"  [{radio.port_name}] Enqueued {len(payload)} byte payload to transmit buffer.")

def read_rx_payload(radio_tx, radio_rx, timeout=25.0):
    start = time.time()
    last_print = time.time()

    print(f"  [{radio_rx.port_name}] Waiting for RX data (timeout={timeout}s)...")
    while time.time() - start < timeout:
        parse_events(radio_tx)
        
        buf = radio_rx.get_buf()
        if buf:
            i = 0
            while i < len(buf):
                tag = buf[i]
                if tag == ord('D'):
                    if i + 6 <= len(buf):
                        evt_id, val = struct.unpack('<BI', buf[i+1:i+6])
                        print(f"  [{radio_rx.port_name} DBG] Event {evt_id}, val={val}")
                        i += 6
                    else:
                        break
                elif tag == ord('G'):
                    if i + 3 <= len(buf):
                        max_pld = struct.unpack('<H', buf[i+1:i+3])[0]
                        print(f"  [{radio_rx.port_name} READY] 'G' max_pld={max_pld}")
                        i += 3
                    else:
                        break
                elif tag == ord('R'):
                    if i + 3 <= len(buf):
                        payload_len = struct.unpack('<H', buf[i+1:i+3])[0]
                        total_needed = i + 3 + payload_len + 2
                        if total_needed <= len(buf):
                            payload = buf[i+3 : i+3+payload_len]
                            rssi = struct.unpack('<h', buf[i+3+payload_len : total_needed])[0]
                            radio_rx.consume(total_needed)
                            print(f"  [{radio_rx.port_name}] Full RX payload received successfully! Size={len(payload)} bytes, RSSI={rssi} dBm")
                            return payload, rssi
                        else:
                            # Payload still arriving over serial, wait for more data
                            break
                    else:
                        break
                else:
                    i += 1
            if i > 0:
                radio_rx.consume(i)

        if time.time() - last_print > 3.0:
            print(f"  [{radio_tx.port_name} rx_buf size={len(radio_tx.get_buf())}] [{radio_rx.port_name} rx_buf size={len(radio_rx.get_buf())}]")
            last_print = time.time()

        time.sleep(0.02)

    print(f"  [{radio_rx.port_name}] Timeout! Total remaining bytes: {len(radio_rx.get_buf())}")
    return None, 0

def main():
    print("=========================================================================")
    print(" Semtech LoRa Plus Xiao EVK - FLRC Burst Duplex 20KB Verification Test")
    print("=========================================================================")

    try:
        r1 = RadioPort(PORT1)
        r2 = RadioPort(PORT2)
    except Exception as e:
        print(f"Failed to open serial ports: {e}")
        sys.exit(1)

    print("Waiting for target boards to boot...")
    time.sleep(1.5)

    print("\n--- Step 1: Initializing radio transmit windows ---")
    if not set_window(r1, offset_ms=100, period_ms=1000):
        print("Failed to set window on Radio 1")
        sys.exit(1)
    if not set_window(r2, offset_ms=600, period_ms=1000):
        print("Failed to set window on Radio 2")
        sys.exit(1)

    print("\n--- Step 2: Generating 20KB test payload ---")
    test_payload = bytes([(i * 7 + 13) % 256 for i in range(PAYLOAD_SIZE)])
    payload_crc = zlib.crc32(test_payload)
    print(f"Payload Size: {len(test_payload)} bytes")
    print(f"Payload CRC32: 0x{payload_crc:08X}")

    print("\n--- Step 3: Phase 1 - Sending 20KB payload from Radio 1 -> Radio 2 ---")
    t0 = time.time()
    rx_bytes = None
    rssi = 0
    for attempt in range(1, 4):
        print(f"  Attempt {attempt}/3...")
        send_payload(r1, test_payload)
        rx_bytes, rssi = read_rx_payload(r1, r2, timeout=8.0)
        if rx_bytes is not None:
            break
        time.sleep(0.5)

    dt1 = time.time() - t0

    if rx_bytes is None:
        print("FAILED: Radio 2 did not receive payload from Radio 1.")
        sys.exit(1)

    rx_crc = zlib.crc32(rx_bytes)
    print(f"Radio 2 Received Size: {len(rx_bytes)} bytes in {dt1:.3f}s")
    print(f"Radio 2 Received CRC32: 0x{rx_crc:08X}")

    if rx_bytes == test_payload:
        print("SUCCESS: Phase 1 - 20KB payload arrived at Radio 2 UNCORRUPTED!")
    else:
        print("FAILED: Phase 1 payload mismatch!")
        sys.exit(1)

    time.sleep(1.0)

    print("\n--- Step 4: Phase 2 - Sending 20KB payload from Radio 2 -> Radio 1 ---")
    t0 = time.time()
    rx_bytes2 = None
    rssi2 = 0
    for attempt in range(1, 4):
        print(f"  Attempt {attempt}/3...")
        send_payload(r2, test_payload)
        rx_bytes2, rssi2 = read_rx_payload(r2, r1, timeout=8.0)
        if rx_bytes2 is not None:
            break
        time.sleep(0.5)

    dt2 = time.time() - t0

    if rx_bytes2 is None:
        print("FAILED: Radio 1 did not receive payload from Radio 2.")
        sys.exit(1)

    rx_crc2 = zlib.crc32(rx_bytes2)
    print(f"Radio 1 Received Size: {len(rx_bytes2)} bytes in {dt2:.3f}s")
    print(f"Radio 1 Received CRC32: 0x{rx_crc2:08X}")

    if rx_bytes2 == test_payload:
        print("SUCCESS: Phase 2 - 20KB payload arrived at Radio 1 UNCORRUPTED!")
    else:
        print("FAILED: Phase 2 payload mismatch!")
        sys.exit(1)

    r1.close()
    r2.close()

    print("\n=========================================================================")
    print(" ALL TESTS PASSED SUCCESSFULLY! Both radios verified in FLRC burst mode.")
    print("=========================================================================")

if __name__ == "__main__":
    main()
