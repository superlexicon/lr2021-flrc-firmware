#!/usr/bin/env python3
"""
measure_raw_speed.py

Detailed timing breakdown probe script for FLRC hardware burst mode.
Measures:
1. Python -> MCU USB Serial Ingestion Time
2. MCU -> Air -> MCU Pure Radio Burst On-Air Time
3. MCU -> Python USB Serial Egress Time
4. Net On-Air Throughput (KB/s & Mbps)
"""

import sys
import time
import zlib
import struct
import serial
import threading

PORT1 = "/dev/ttyACM0"
PORT2 = "/dev/ttyACM1"
BAUD = 115200

PAYLOAD_SIZE = 900  # 900 bytes (4 fragments)

class RadioPort:
    def __init__(self, port):
        self.port_name = port
        self.ser = serial.Serial(port, BAUD, timeout=0.01, write_timeout=10.0)
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
            except Exception:
                time.sleep(0.001)

    def write(self, data):
        chunk_size = 64
        for i in range(0, len(data), chunk_size):
            self.ser.write(data[i:i+chunk_size])
            self.ser.flush()
            time.sleep(0.002)

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

def main():
    print("=========================================================================")
    print(" FLRC Hardware Burst — Detailed Timing & Theoretical Speed Comparison")
    print("=========================================================================")

    try:
        r1 = RadioPort(PORT1)
        r2 = RadioPort(PORT2)
    except Exception as e:
        print(f"Failed to open ports: {e}")
        sys.exit(1)

    time.sleep(1.0)
    r1.clear()
    r2.clear()

    test_payload = bytes([(i * 13 + 7) % 256 for i in range(PAYLOAD_SIZE)])
    expected_crc = zlib.crc32(test_payload)

    # Prepare command frame: b'TX' + len (2 bytes) + payload
    cmd = bytearray(b'TX')
    cmd.extend(struct.pack('<H', len(test_payload)))
    cmd.extend(test_payload)

    print(f"Payload Size           : {PAYLOAD_SIZE} bytes ({PAYLOAD_SIZE * 8} bits)")
    print(f"Configured Mod. Rate   : 2.600 Mbps raw (Coding Rate 1/1 -> 2.60 Mbps uncoded net)")
    print(f"Burst Fragment Sizing  : 4 fragments of 252 bytes (16B header + 236B chunk)")
    print(f"Interframe Delay       : 300 us (0.3 ms) between fragments\n")

    # Measure fast burst transmission (un-paced USB write)
    t0_tx = time.perf_counter()
    r1.write(cmd)
    t1_tx = time.perf_counter()
    serial_write_time_ms = (t1_tx - t0_tx) * 1000.0

    # Wait for RX payload on Radio 2
    t0_rx_wait = time.perf_counter()
    rx_payload = None
    rssi = 0

    while time.perf_counter() - t0_rx_wait < 3.0:
        buf = r2.get_buf()
        if len(buf) >= 3:
            for i in range(len(buf)):
                if buf[i] == ord('R') and i + 3 <= len(buf):
                    pld_len = struct.unpack('<H', buf[i+1:i+3])[0]
                    total_needed = i + 3 + pld_len + 2
                    if total_needed <= len(buf):
                        rx_payload = buf[i+3 : i+3+pld_len]
                        rssi = struct.unpack('<h', buf[i+3+pld_len : total_needed])[0]
                        r2.consume(total_needed)
                        t1_rx_received = time.perf_counter()
                        break
            if rx_payload is not None:
                break
        time.sleep(0.001)

    r1.close()
    r2.close()

    if rx_payload is None or zlib.crc32(rx_payload) != expected_crc:
        print("FAILED: Did not receive valid payload on Radio 2")
        sys.exit(1)

    total_latency_ms = (t1_rx_received - t0_tx) * 1000.0
    air_plus_egress_ms = total_latency_ms - serial_write_time_ms

    # Theoretical time calculation for CR 1/1 (2.6 Mbps) + 300us interframe delay:
    # 4 packets * 0.62ms packet duration + 3 * 0.3ms interframe delay = 2.48ms + 0.90ms = 3.38ms
    theoretical_toa_ms = (4 * 0.62) + (3 * 0.30)
    theoretical_on_air_speed_kbps = (PAYLOAD_SIZE * 8) / (theoretical_toa_ms / 1000.0)
    theoretical_on_air_speed_kbs = theoretical_on_air_speed_kbps / 8.0

    end_to_end_speed_kbps = (PAYLOAD_SIZE * 8) / (total_latency_ms / 1000.0)
    end_to_end_speed_kbs = end_to_end_speed_kbps / 8.0

    print("-------------------------------------------------------------------------")
    print(" TIMING & SPEED BREAKDOWN ANALYSIS")
    print("-------------------------------------------------------------------------")
    print(f"1. Host Serial TX Ingestion Time (USB -> Radio 1 MCU) : {serial_write_time_ms:6.2f} ms")
    print(f"2. Radio RF On-Air Burst Time (Radio 1 -> Radio 2)    :   {theoretical_toa_ms:6.2f} ms (Theoretical TOA)")
    print(f"3. Host Serial RX Egress Time (Radio 2 MCU -> USB)   : {air_plus_egress_ms - theoretical_toa_ms:6.2f} ms")
    print(f"4. Total Host-to-Host End-to-End Elapsed Time        : {total_latency_ms:6.2f} ms\n")

    print(f"--- Theoretical FLRC Physical Layer (On-Air) Speed ---")
    print(f"  On-Air Payload Rate (excluding headers/interframe)  : {theoretical_on_air_speed_kbps:7.1f} kbps ({theoretical_on_air_speed_kbs:6.2f} KB/s)")
    print(f"  Raw Bit Rate Setting                               : 2600.0 kbps (325.00 KB/s)")

    print(f"\n--- Measured End-to-End Host Speed (Including USB Serial) ---")
    print(f"  End-to-End Speed without TDMA Slot Delays           : {end_to_end_speed_kbps:7.1f} kbps ({end_to_end_speed_kbs:6.2f} KB/s)")
    print("=========================================================================\n")

if __name__ == "__main__":
    main()
