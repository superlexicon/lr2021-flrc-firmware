#!/usr/bin/env python3
"""
test_burst_broadcast.py

Multi-node FLRC burst broadcast/receive test — no reader thread, direct reads.

Usage:
  python3 test_burst_broadcast.py --role tx --port /dev/ttyACM0
  python3 test_burst_broadcast.py --role rx --port /dev/ttyACM0
"""

import sys
import time
import struct
import argparse
from collections import defaultdict

BAUD = 115200
FREQ_HZ = 915000000
BW_HZ = 1200000
CODING_RATE = 2
TX_PWR_DBM = 20
SYNC_WORD = 0x534C2021

TX_INTERVAL_MS = 1000
TEST_DURATION_S = 60

PAYLOAD_SIZES = [155, 430, 868, 1300]


def send_config(ser):
    body = struct.pack('<I', FREQ_HZ)
    body += struct.pack('<I', BW_HZ)
    body += struct.pack('B', CODING_RATE)
    body += struct.pack('b', TX_PWR_DBM)
    body += struct.pack('<I', SYNC_WORD)
    body += struct.pack('B', 0)
    frame = b'C' + struct.pack('<H', len(body)) + body
    ser.write(frame)
    ser.flush()


def wait_ready(ser, timeout=15.0):
    deadline = time.time() + timeout
    last_config = 0
    while time.time() < deadline:
        try:
            raw = ser.read(64)
        except Exception:
            time.sleep(0.1)
            continue
        if raw:
            for i in range(len(raw)):
                if raw[i:i+1] == b'G' and i + 2 < len(raw):
                    max_val = struct.unpack('<H', raw[i+1:i+3])[0]
                    print(f"  READY (max payload: {max_val})")
                    return max_val
            continue
        if (time.time() - last_config) > 1.0:
            send_config(ser)
            last_config = time.time()
        time.sleep(0.1)
    print("  READY timeout!")
    return None


def send_tx(ser, payload):
    frame = b'TX' + struct.pack('<H', len(payload)) + payload
    ser.write(frame)
    ser.flush()


def read_rx_frames(ser, stats):
    """Parse RX frames. Returns number of frames parsed."""
    count = 0
    try:
        raw = ser.read(4096)
    except Exception:
        return 0
    if not raw:
        return 0

    i = 0
    while i < len(raw):
        tag = raw[i:i+1]
        if tag == b'R' and i + 3 <= len(raw):
            plen = struct.unpack('<H', raw[i+1:i+3])[0]
            total_needed = 3 + plen + 2  # tag + len + payload + rssi
            if i + total_needed <= len(raw):
                payload = raw[i+3:i+3+plen]
                rssi = struct.unpack('<h', raw[i+3+plen:i+5+plen])[0]
                stats['rx_count'] += 1
                stats['rx_by_size'][plen] += 1
                stats['last_rssi'] = rssi
                if plen in PAYLOAD_SIZES:
                    stats['rx_test_payloads'] += 1
                    print(f"  RX: {plen}B RSSI={rssi}dBm (total test: {stats['rx_test_payloads']})")
                i += total_needed
                count += 1
            else:
                i += 1  # incomplete frame, skip
        elif tag == b'G' and i + 3 <= len(raw):
            i += 3  # READY, skip
        elif tag == b'E' and i + 2 <= len(raw):
            if i + 2 < len(raw):
                msg_len = raw[i+2]
                i += 3 + msg_len
            else:
                i += 1
        else:
            i += 1  # unknown byte, skip
    return count


def run_tx(ser, duration_s, tx_interval_ms=TX_INTERVAL_MS):
    print(f"=== TX: every {tx_interval_ms}ms for {duration_s}s ===")
    stats = {'tx_count': 0, 'tx_by_size': defaultdict(int)}
    deadline = time.time() + duration_s
    size_idx = 0

    while time.time() < deadline:
        size = PAYLOAD_SIZES[size_idx % len(PAYLOAD_SIZES)]
        size_idx += 1
        payload = bytes([(i * 13 + 7) % 256 for i in range(size)])
        send_tx(ser, payload)
        stats['tx_count'] += 1
        stats['tx_by_size'][size] += 1

        elapsed = int(time.time() - (deadline - duration_s))
        if stats['tx_count'] % 10 == 0:
            print(f"  [{elapsed}s] TX #{stats['tx_count']}: {size}B")

        time.sleep(tx_interval_ms / 1000.0)

    print(f"\n=== TX SUMMARY: {stats['tx_count']} TXs ===")
    for size, count in sorted(stats['tx_by_size'].items()):
        packets = (size + 236 - 1) // 236
        print(f"  {size}B ({packets}pkt): {count}")
    return stats


def run_rx(ser, duration_s):
    print(f"=== RX: listening {duration_s}s ===")
    stats = {
        'rx_count': 0, 'rx_by_size': defaultdict(int),
        'rx_test_payloads': 0, 'errors': 0, 'last_rssi': 0,
    }
    deadline = time.time() + duration_s
    last_report = time.time()

    while time.time() < deadline:
        read_rx_frames(ser, stats)
        if time.time() - last_report >= 10.0:
            elapsed = int(time.time() - (deadline - duration_s))
            print(f"  [{elapsed}s] RX: {stats['rx_count']} total, "
                  f"{stats['rx_test_payloads']} test, RSSI={stats['last_rssi']}dBm")
            if stats['rx_by_size']:
                print(f"    {dict(stats['rx_by_size'])}")
            last_report = time.time()
        time.sleep(0.001)

    read_rx_frames(ser, stats)
    print(f"\n=== RX SUMMARY: {stats['rx_count']} frames ===")
    for size, count in sorted(stats['rx_by_size'].items()):
        packets = (size + 236 - 1) // 236
        print(f"  {size}B ({packets}pkt): {count}")
    return stats


def main():
    parser = argparse.ArgumentParser(description='FLRC burst broadcast/receive test')
    parser.add_argument('--role', choices=['tx', 'rx'], required=True)
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--duration', type=int, default=TEST_DURATION_S)
    parser.add_argument('--interval', type=int, default=TX_INTERVAL_MS)
    args = parser.parse_args()

    import serial

    tx_interval = args.interval

    print(f"FLRC Burst Test: {args.role.upper()} on {args.port}, {args.duration}s")

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

    time.sleep(0.5)
    ser.timeout = 0.01  # short timeout for RX loop

    if args.role == 'tx':
        run_tx(ser, args.duration, tx_interval)
    else:
        run_rx(ser, args.duration)

    ser.close()
    print("Done.")


if __name__ == '__main__':
    main()
