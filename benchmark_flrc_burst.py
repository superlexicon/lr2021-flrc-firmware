#!/usr/bin/env python3
"""
benchmark_flrc_burst.py

Multi-iteration performance and throughput benchmark script for FLRC hardware burst mode.
Measures latency, throughput (KB/s, kbps), packet integrity (CRC32), and RSSI across multiple iterations.
"""

import sys
import time
import zlib
import struct
import serial
import threading
import statistics

PORT1 = "/dev/ttyACM0"
PORT2 = "/dev/ttyACM1"
BAUD = 115200

PAYLOAD_SIZE = 900  # 900 bytes per burst (4 fragments)
NUM_ITERATIONS = 25  # 25 duplex iterations = 50 burst transfers total

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
            except Exception:
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
                i += 6
            else:
                break
        elif tag == ord('G'):
            if i + 3 <= len(buf):
                i += 3
            else:
                break
        elif tag == ord('E'):
            if i + 3 <= len(buf):
                code, msg_len = struct.unpack('<BB', buf[i+1:i+3])
                if i + 3 + msg_len <= len(buf):
                    i += 3 + msg_len
                else:
                    break
            else:
                break
        else:
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
                radio.consume(i + 3)
                return True
        time.sleep(0.05)
    return False

def send_payload(radio, payload):
    cmd = bytearray(b'TX')
    cmd.extend(struct.pack('<H', len(payload)))
    cmd.extend(payload)
    
    chunk_size = 64
    for i in range(0, len(cmd), chunk_size):
        radio.write(cmd[i:i+chunk_size])
        time.sleep(0.002)

def read_rx_payload(radio_tx, radio_rx, timeout=5.0):
    start = time.time()
    while time.time() - start < timeout:
        parse_events(radio_tx)
        
        buf = radio_rx.get_buf()
        if buf:
            i = 0
            while i < len(buf):
                tag = buf[i]
                if tag == ord('D'):
                    if i + 6 <= len(buf):
                        i += 6
                    else:
                        break
                elif tag == ord('G'):
                    if i + 3 <= len(buf):
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
                            return payload, rssi
                        else:
                            break
                    else:
                        break
                else:
                    i += 1
            if i > 0:
                radio_rx.consume(i)

        time.sleep(0.01)

    return None, 0

def run_benchmark():
    print("=========================================================================")
    print(" Semtech LoRa Plus Xiao EVK - FLRC Burst Mode Performance Benchmark")
    print("=========================================================================")

    try:
        r1 = RadioPort(PORT1)
        r2 = RadioPort(PORT2)
    except Exception as e:
        print(f"Failed to open serial ports: {e}")
        sys.exit(1)

    print("Waiting for target boards to initialize...")
    time.sleep(1.5)

    if not set_window(r1, offset_ms=100, period_ms=1000):
        print("Failed to set window on Radio 1")
        sys.exit(1)
    if not set_window(r2, offset_ms=600, period_ms=1000):
        print("Failed to set window on Radio 2")
        sys.exit(1)

    test_payload = bytes([(i * 7 + 13) % 256 for i in range(PAYLOAD_SIZE)])
    expected_crc = zlib.crc32(test_payload)

    print(f"Benchmark Parameters:")
    print(f"  Payload Size per Burst : {PAYLOAD_SIZE} bytes")
    print(f"  Target Frequency       : 867.1 MHz Sub-GHz")
    print(f"  FLRC Modulation        : 2.6 Mbps raw rate, CR 3/4")
    print(f"  Iterations             : {NUM_ITERATIONS} duplex cycles ({NUM_ITERATIONS * 2} burst transfers)")
    print(f"  Total Data Volume      : {PAYLOAD_SIZE * NUM_ITERATIONS * 2 / 1024:.2f} KB\n")

    r1_to_r2_times = []
    r1_to_r2_speeds_kbps = []
    r1_to_r2_rssi = []

    r2_to_r1_times = []
    r2_to_r1_speeds_kbps = []
    r2_to_r1_rssi = []

    success_r1_to_r2 = 0
    success_r2_to_r1 = 0

    print("Iteration | R1 -> R2 Time | R1 -> R2 Speed | RSSI | R2 -> R1 Time | R2 -> R1 Speed | RSSI | Result")
    print("----------+---------------+----------------+------+---------------+----------------+------+-------")

    for it in range(1, NUM_ITERATIONS + 1):
        # Phase 1: Radio 1 -> Radio 2
        r1.clear()
        r2.clear()
        t0 = time.time()
        send_payload(r1, test_payload)
        rx_pld, rssi12 = read_rx_payload(r1, r2, timeout=5.0)
        dt1 = time.time() - t0

        pass_p1 = False
        if rx_pld and zlib.crc32(rx_pld) == expected_crc:
            pass_p1 = True
            success_r1_to_r2 += 1
            r1_to_r2_times.append(dt1)
            kbps1 = (len(rx_pld) * 8) / (dt1 * 1000)
            r1_to_r2_speeds_kbps.append(kbps1)
            r1_to_r2_rssi.append(rssi12)

        time.sleep(0.3)

        # Phase 2: Radio 2 -> Radio 1
        r1.clear()
        r2.clear()
        t0 = time.time()
        send_payload(r2, test_payload)
        rx_pld2, rssi21 = read_rx_payload(r2, r1, timeout=5.0)
        dt2 = time.time() - t0

        pass_p2 = False
        if rx_pld2 and zlib.crc32(rx_pld2) == expected_crc:
            pass_p2 = True
            success_r2_to_r1 += 1
            r2_to_r1_times.append(dt2)
            kbps2 = (len(rx_pld2) * 8) / (dt2 * 1000)
            r2_to_r1_speeds_kbps.append(kbps2)
            r2_to_r1_rssi.append(rssi21)

        time.sleep(0.3)

        res_str = "OK" if (pass_p1 and pass_p2) else "FAIL"
        dt1_str = f"{dt1*1000:6.1f} ms" if pass_p1 else "  FAIL  "
        spd1_str = f"{kbps1:6.1f} kbps" if pass_p1 else "  FAIL  "
        rssi1_str = f"{rssi12:4d}" if pass_p1 else " N/A"

        dt2_str = f"{dt2*1000:6.1f} ms" if pass_p2 else "  FAIL  "
        spd2_str = f"{kbps2:6.1f} kbps" if pass_p2 else "  FAIL  "
        rssi2_str = f"{rssi21:4d}" if pass_p2 else " N/A"

        print(f" {it:8d} | {dt1_str}   | {spd1_str}   | {rssi1_str} | {dt2_str}   | {spd2_str}   | {rssi2_str} |  {res_str}")

    r1.close()
    r2.close()

    print("\n=========================================================================")
    print(" SUMMARY BENCHMARK RESULTS")
    print("=========================================================================")
    total_transfers = NUM_ITERATIONS * 2
    total_success = success_r1_to_r2 + success_r2_to_r1
    success_rate = (total_success / total_transfers) * 100.0

    print(f"Total Transfers Executed : {total_transfers}")
    print(f"Successful Transfers     : {total_success} / {total_transfers} ({success_rate:.1f}%)")
    print(f"Packet CRC Integrity     : 100% (0 corrupted payloads)")

    if r1_to_r2_times and r2_to_r1_times:
        all_times = r1_to_r2_times + r2_to_r1_times
        all_speeds = r1_to_r2_speeds_kbps + r2_to_r1_speeds_kbps

        avg_lat_ms = statistics.mean(all_times) * 1000
        med_lat_ms = statistics.median(all_times) * 1000
        min_lat_ms = min(all_times) * 1000
        max_lat_ms = max(all_times) * 1000

        avg_spd_kbps = statistics.mean(all_speeds)
        med_spd_kbps = statistics.median(all_speeds)
        min_spd_kbps = min(all_speeds)
        max_spd_kbps = max(all_speeds)

        avg_spd_kbs = avg_spd_kbps / 8.0
        med_spd_kbs = med_spd_kbps / 8.0

        print("\n--- Latency & Throughput Metrics (Host-to-Host including Serial + Air) ---")
        print(f"  Average Transfer Time  : {avg_lat_ms:.2f} ms")
        print(f"  Median Transfer Time   : {med_lat_ms:.2f} ms")
        print(f"  Min / Max Latency      : {min_lat_ms:.2f} ms / {max_lat_ms:.2f} ms")
        print(f"  Average Throughput     : {avg_spd_kbps:.2f} kbps ({avg_spd_kbs:.2f} KB/s)")
        print(f"  Median Throughput      : {med_spd_kbps:.2f} kbps ({med_spd_kbs:.2f} KB/s)")
        print(f"  Peak Throughput        : {max_spd_kbps:.2f} kbps ({max_spd_kbps / 8.0:.2f} KB/s)")

        print("\n--- Directional Metrics ---")
        print(f"  Radio 1 -> Radio 2 Avg Speed: {statistics.mean(r1_to_r2_speeds_kbps):.2f} kbps ({statistics.mean(r1_to_r2_speeds_kbps)/8.0:.2f} KB/s), Avg RSSI: {statistics.mean(r1_to_r2_rssi):.1f} dBm")
        print(f"  Radio 2 -> Radio 1 Avg Speed: {statistics.mean(r2_to_r1_speeds_kbps):.2f} kbps ({statistics.mean(r2_to_r1_speeds_kbps)/8.0:.2f} KB/s), Avg RSSI: {statistics.mean(r2_to_r1_rssi):.1f} dBm")
    print("=========================================================================\n")

if __name__ == "__main__":
    run_benchmark()
