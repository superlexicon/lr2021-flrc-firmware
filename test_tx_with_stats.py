#!/usr/bin/env python3
"""
test_tx_with_stats.py

Simplified TX test that polls firmware stats every 5 seconds.
Run on the TX node. Uses a single serial port, no reader thread.

Usage:
  python3 test_tx_with_stats.py --port /dev/ttyACM1 --duration 180
"""

import sys
import time
import struct
import zlib
import argparse

BAUD = 115200
FREQ_HZ = 915000000
BW_HZ = 1333000
CODING_RATE = 2
TX_PWR_DBM = 20
SYNC_WORD = 0x534C2021

PAYLOAD_SIZES = [155, 430, 877]
TX_INTERVAL_MS = 1000

CHUNK_SIZE = 64
CHUNK_DELAY_S = 0.002


def write_chunked(ser, data):
    for i in range(0, len(data), CHUNK_SIZE):
        ser.write(data[i:i + CHUNK_SIZE])
        time.sleep(CHUNK_DELAY_S)
    ser.flush()


def send_config(ser):
    body = struct.pack('<I', FREQ_HZ)
    body += struct.pack('<I', BW_HZ)
    body += struct.pack('B', CODING_RATE)
    body += struct.pack('b', TX_PWR_DBM)
    body += struct.pack('<I', SYNC_WORD)
    body += struct.pack('B', 0)
    write_chunked(ser, b'C' + struct.pack('<H', len(body)) + body)


def wait_ready(ser, timeout=15.0):
    deadline = time.time() + timeout
    last_config = 0
    while time.time() < deadline:
        raw = ser.read(64)
        if raw:
            for i in range(len(raw)):
                if raw[i:i+1] == b'G' and i + 2 < len(raw):
                    max_val = struct.unpack('<H', raw[i+1:i+3])[0]
                    return max_val
        if time.time() - last_config > 1.0:
            send_config(ser)
            last_config = time.time()
        time.sleep(0.1)
    return None


def send_tx(ser, payload):
    write_chunked(ser, b'TX' + struct.pack('<H', len(payload)) + payload)


def poll_stats(ser):
    """Send 'S' and parse the response. Returns stats dict or None."""
    ser.write(b'S')
    ser.flush()
    time.sleep(0.15)
    data = ser.read(4096)
    i = 0
    while i < len(data):
        if data[i] == ord('S') and i + 27 <= len(data):
            slen = struct.unpack('<H', data[i+1:i+3])[0]
            if slen >= 24 and i + 3 + slen <= len(data):
                vals = struct.unpack('<6I', data[i+3:i+27])
                return {
                    'bursts_tx': vals[0], 'bursts_rx': vals[1],
                    'packets_tx': vals[2], 'packets_rx': vals[3],
                    'crc_errors': vals[4], 'host_tx_dropped': vals[5],
                }
            i += 1
        else:
            i += 1
    return None


def main():
    parser = argparse.ArgumentParser(description='TX test with firmware stats polling')
    parser.add_argument('--port', default='/dev/ttyACM1')
    parser.add_argument('--duration', type=int, default=180)
    parser.add_argument('--interval', type=int, default=TX_INTERVAL_MS)
    parser.add_argument('--node-id', type=int, default=1)
    args = parser.parse_args()

    import serial

    ser = serial.Serial(args.port, BAUD, timeout=0.05, write_timeout=10.0)
    ser.dtr = True
    ser.rts = True
    time.sleep(2.0)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.5)

    print("Sending CONFIG...")
    send_config(ser)
    max_pl = wait_ready(ser)
    if max_pl is None:
        print("FAILED: no READY")
        ser.close()
        sys.exit(1)
    print(f"TX radio ready (max payload: {max_pl})")

    duration_s = args.duration
    interval_s = args.interval / 1000.0
    deadline = time.time() + duration_s
    tx_count = 0
    size_idx = 0
    stats_history = []
    last_stats_poll = 0
    start_time = time.time()

    print(f"\nTX test: {duration_s}s, interval {args.interval}ms, sizes {PAYLOAD_SIZES}")
    print(f"{'='*70}\n")

    while time.time() < deadline:
        # TX
        size = PAYLOAD_SIZES[size_idx % len(PAYLOAD_SIZES)]
        size_idx += 1
        body = bytes([(args.node_id * 31 + tx_count * 7 + i * 13) % 256
                      for i in range(size - 4)])
        crc = zlib.crc32(body) & 0xFFFFFFFF
        payload = body + struct.pack('<I', crc)
        send_tx(ser, payload)
        tx_count += 1

        # Poll stats every ~10 TX cycles
        if tx_count % 10 == 0:
            # Drain any pending data first
            ser.read(4096)
            s = poll_stats(ser)
            if s:
                s['timestamp'] = time.time()
                s['tx_count'] = tx_count
                stats_history.append(s)
                elapsed = time.time() - start_time
                print(f"  [{elapsed:5.0f}s] TX#{tx_count:3d} STATS: "
                      f"tx_bursts={s['bursts_tx']:5d} rx_bursts={s['bursts_rx']:5d} "
                      f"tx_pkts={s['packets_tx']:5d} rx_pkts={s['packets_rx']:5d} "
                      f"crc_err={s['crc_errors']:4d} dropped={s['host_tx_dropped']:4d}")

        if tx_count % 30 == 0:
            elapsed = int(time.time() - start_time)
            packets = (size + 236 - 1) // 236
            print(f"  [{elapsed}s] TX #{tx_count}: {size}B ({packets}pkt)")

        time.sleep(interval_s)

    # Final stats
    ser.read(4096)
    s = poll_stats(ser)
    if s:
        elapsed = time.time() - start_time
        print(f"  [{elapsed:5.0f}s] FINAL STATS: "
              f"tx_bursts={s['bursts_tx']:5d} rx_bursts={s['bursts_rx']:5d} "
              f"dropped={s['host_tx_dropped']:4d}")

    # Print evolution
    if stats_history:
        print(f"\n{'='*70}")
        print(" TX FIRMWARE STATS EVOLUTION")
        print(f"{'='*70}")
        print(f"  {'Time':>6s}  {'TX#':>4s}  {'TX_burst':>8s}  {'RX_burst':>8s}  "
              f"{'TX_pkt':>8s}  {'RX_pkt':>8s}  {'CRC_err':>7s}  {'Dropped':>7s}  {'Delta':>6s}")
        prev_bursts = 0
        for s in stats_history:
            elapsed = s['timestamp'] - start_time
            delta = s['bursts_tx'] - prev_bursts
            prev_bursts = s['bursts_tx']
            print(f"  {elapsed:5.0f}s  {s['tx_count']:4d}  {s['bursts_tx']:8d}  {s['bursts_rx']:8d}  "
                  f"{s['packets_tx']:8d}  {s['packets_rx']:8d}  "
                  f"{s['crc_errors']:7d}  {s['host_tx_dropped']:7d}  +{delta:5d}")

    ser.close()
    print(f"\nDone. Sent {tx_count} TX payloads in {time.time() - start_time:.1f}s.")


if __name__ == '__main__':
    main()
