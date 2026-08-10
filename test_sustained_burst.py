#!/usr/bin/env python3
"""
test_sustained_burst.py

Reproduces the multi-packet burst delivery degradation observed in the
superlexicon consensus run:

  - First ~30s after a fresh board reset: multi-packet bursts (877B / 4pkts)
    deliver reliably.
  - After ~30s: delivery drops to ~1 per minute while single-packet frames
    (155B announcements) continue at their normal 30s interval.

This script runs on ONE node and uses BOTH radios (dual-radio setup):
  - TX radio  (/dev/ttyACMx): sends a sequence of payloads on a 1s cadence,
    cycling through sizes that match real superlexicon traffic:
      155B (1pkt  - announcement equivalent)
      430B (2pkt  - single proposal equivalent)
      877B (4pkt  - multi-proposal equivalent)
  - RX radio  (/dev/ttyACMy): listens continuously and logs every received
    frame with timestamp, size, and RSSI.

At the end, it prints a timeline of delivery rate in 10-second buckets so
the degradation (if present) is immediately visible as a cliff in the
per-bucket counts.

Usage (run on one node, with a peer node running the same script or
superlexicon):

  # Node A: TX on primary radio, RX on secondary radio
  python3 test_sustained_burst.py --tx-port /dev/ttyACM1 --rx-port /dev/ttyACM2

  # Node B: just listen (its TX radio can be running superlexicon or idle)
  python3 test_sustained_burst.py --rx-only --rx-port /dev/ttyACM0

  # Single-radio mode: one port does both TX and RX
  python3 test_sustained_burst.py --tx-port /dev/ttyACM0 --rx-port /dev/ttyACM0
"""

import sys
import time
import struct
import zlib
import argparse
import threading
from collections import defaultdict

BAUD = 115200
FREQ_HZ = 915000000
BW_HZ = 1333000
CODING_RATE = 2
TX_PWR_DBM = 20
SYNC_WORD = 0x534C2021

# Payload sizes matching real superlexicon traffic
PAYLOAD_SIZES = [155, 430, 877]
TX_INTERVAL_MS = 1000
DEFAULT_DURATION_S = 180  # 3 minutes — long enough to see the degradation

CHUNK_SIZE = 64        # USB bulk write chunk
CHUNK_DELAY_S = 0.002  # 2ms between chunks (prevents MCU UART RX overflow)


# ---------------------------------------------------------------------------
# Serial helpers
# ---------------------------------------------------------------------------

class RadioPort:
    """Thin wrapper around pyserial with a background reader thread."""

    def __init__(self, port, name=""):
        import serial as pyserial
        self.port_name = port
        self.name = name or port
        self.ser = pyserial.Serial(port, BAUD, timeout=0.05, write_timeout=10.0)
        self.ser.dtr = True
        self.ser.rts = True
        time.sleep(2.0)  # let the MCU settle after port open
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        time.sleep(0.5)

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
        """Write in small chunks to avoid MCU UART buffer overflow."""
        for i in range(0, len(data), CHUNK_SIZE):
            chunk = data[i:i + CHUNK_SIZE]
            written = 0
            while written < len(chunk):
                n = self.ser.write(chunk[written:])
                if n is not None:
                    written += n
                else:
                    time.sleep(0.001)
            time.sleep(CHUNK_DELAY_S)
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
        try:
            self.ser.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Protocol primitives
# ---------------------------------------------------------------------------

def send_config(radio):
    body = struct.pack('<I', FREQ_HZ)
    body += struct.pack('<I', BW_HZ)
    body += struct.pack('B', CODING_RATE)
    body += struct.pack('b', TX_PWR_DBM)
    body += struct.pack('<I', SYNC_WORD)
    body += struct.pack('B', 0)  # role=Both
    frame = b'C' + struct.pack('<H', len(body)) + body
    radio.write(frame)


def wait_ready(radio, timeout=15.0):
    radio.clear()
    deadline = time.time() + timeout
    last_config = 0
    while time.time() < deadline:
        buf = radio.get_buf()
        for i in range(len(buf)):
            if buf[i] == ord('G') and i + 3 <= len(buf):
                max_pld = struct.unpack('<H', buf[i+1:i+3])[0]
                radio.consume(i + 3)
                return max_pld
        if time.time() - last_config > 1.0:
            send_config(radio)
            last_config = time.time()
        time.sleep(0.1)
    return None


def send_tx(radio, payload):
    """Send a TX command with the given payload."""
    frame = b'TX' + struct.pack('<H', len(payload)) + payload
    radio.write(frame)


# ---------------------------------------------------------------------------
# RX frame parser
# ---------------------------------------------------------------------------

def parse_rx_frames(radio):
    """Parse all complete RX frames from the buffer.

    Returns a list of (timestamp, payload_len, rssi, payload) tuples.
    Consumes parsed bytes from the buffer.
    """
    results = []
    buf = radio.get_buf()
    i = 0
    while i < len(buf):
        tag = buf[i]
        if tag == ord('R') and i + 3 <= len(buf):
            plen = struct.unpack('<H', buf[i+1:i+3])[0]
            total_needed = 3 + plen + 2  # tag + len + payload + rssi(int16)
            if i + total_needed <= len(buf):
                payload = buf[i+3:i+3+plen]
                rssi = struct.unpack('<h', buf[i+3+plen:i+5+plen])[0]
                results.append((time.time(), plen, rssi, payload))
                i += total_needed
            else:
                break  # incomplete — wait for more data
        elif tag == ord('G') and i + 3 <= len(buf):
            i += 3  # READY, skip
        elif tag == ord('E') and i + 3 <= len(buf):
            msg_len = buf[i+2]
            if i + 3 + msg_len <= len(buf):
                code = buf[i+1]
                msg = buf[i+3:i+3+msg_len].decode('utf-8', errors='ignore')
                print(f"  [{radio.name}] ERROR code={code}: {msg}")
                i += 3 + msg_len
            else:
                break
        else:
            i += 1  # skip unknown byte
    if i > 0:
        radio.consume(i)
    return results


# ---------------------------------------------------------------------------
# TX loop
# ---------------------------------------------------------------------------

def run_tx_loop(radio, duration_s, interval_ms, node_id, shared_radio=None):
    """Continuously TX payloads cycling through PAYLOAD_SIZES.

    If shared_radio is set (same RadioPort as RX), TX is interleaved with
    RX parsing in the same thread to avoid two threads competing for one
    serial port.
    """
    deadline = time.time() + duration_s
    tx_count = 0
    size_idx = 0
    interval_s = interval_ms / 1000.0

    while time.time() < deadline:
        size = PAYLOAD_SIZES[size_idx % len(PAYLOAD_SIZES)]
        size_idx += 1

        # Build a deterministic payload with a CRC trailer for integrity check
        body = bytes([(node_id * 31 + tx_count * 7 + i * 13) % 256
                      for i in range(size - 4)])
        crc = zlib.crc32(body) & 0xFFFFFFFF
        payload = body + struct.pack('<I', crc)

        send_tx(radio, payload)
        tx_count += 1

        if tx_count % 30 == 0:
            elapsed = int(time.time() - (deadline - duration_s))
            packets = (size + 236 - 1) // 236
            print(f"  [{elapsed}s] TX #{tx_count}: {size}B ({packets}pkt)")

        # In shared mode, parse RX during the sleep window
        if shared_radio:
            t_end = time.time() + interval_s
            while time.time() < t_end:
                parse_rx_frames(shared_radio)
                time.sleep(0.01)
        else:
            time.sleep(interval_s)

    return tx_count


# ---------------------------------------------------------------------------
# RX logger
# ---------------------------------------------------------------------------

def run_rx_logger(radio, duration_s, results_list):
    """Continuously parse and log RX frames with timestamps.

    Appends (timestamp, plen, rssi, crc_ok) to results_list.
    """
    deadline = time.time() + duration_s
    last_report = time.time()

    while time.time() < deadline:
        new_frames = parse_rx_frames(radio)
        for ts, plen, rssi, payload in new_frames:
            # Verify CRC if the payload is large enough to have a trailer
            if plen >= 4:
                body = payload[:-4]
                expected_crc = struct.unpack('<I', payload[-4:])[0]
                actual_crc = zlib.crc32(body) & 0xFFFFFFFF
                crc_ok = expected_crc == actual_crc
            else:
                crc_ok = True

            results_list.append((ts, plen, rssi, crc_ok))

            mark = "✓" if crc_ok else "✗"
            packets = (plen + 236 - 1) // 236
            elapsed = ts - (deadline - duration_s)
            print(f"  [{elapsed:6.1f}s] RX: {plen:4d}B ({packets}pkt) "
                  f"RSSI={rssi:+4d}dBm CRC={mark}")

        if time.time() - last_report >= 10.0:
            elapsed = int(time.time() - (deadline - duration_s))
            recent = [f for f in results_list if f[0] > time.time() - 10.0]
            large = [f for f in recent if f[1] > 236]
            print(f"  [{elapsed}s] --- 10s window: {len(recent)} total RX, "
                  f"{len(large)} multi-pkt, buf={len(radio.get_buf())}B ---")
            last_report = time.time()

        time.sleep(0.01)


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------

def print_report(frames, duration_s, tx_count):
    """Print a summary report with 10-second bucket timeline."""
    print("\n" + "=" * 70)
    print(" SUSTAINED BURST DELIVERY REPORT")
    print("=" * 70)

    total_rx = len(frames)
    crc_fail = sum(1 for f in frames if not f[3])
    print(f"Duration:        {duration_s}s")
    print(f"Total TX:        {tx_count}")
    print(f"Total RX:        {total_rx}")
    if crc_fail:
        print(f"CRC failures:    {crc_fail}")
    if tx_count > 0:
        print(f"Delivery rate:   {total_rx / tx_count * 100:.1f}%")

    # Breakdown by packet count
    by_pkt = defaultdict(int)
    for ts, plen, rssi, crc_ok in frames:
        pkt = (plen + 236 - 1) // 236
        by_pkt[pkt] += 1
    print(f"\nRX by packet count:")
    for pkt in sorted(by_pkt):
        print(f"  {pkt}pkt: {by_pkt[pkt]:4d} frames")

    # Timeline: 10-second buckets
    if frames:
        print(f"\nDelivery timeline (10s buckets):")
        print(f"  {'Bucket':>8s}  {'Total':>5s}  {'1pkt':>5s}  {'2pkt':>5s}  {'4pkt':>5s}  {'Bar':s}")
        print(f"  {'-'*8}  {'-'*5}  {'-'*5}  {'-'*5}  {'-'*5}  {'-'*30}")
        start_time = frames[0][0]
        bucket_size = 10.0
        max_bucket = int((frames[-1][0] - start_time) / bucket_size) + 1
        for b in range(max_bucket):
            bucket_start = start_time + b * bucket_size
            bucket_end = bucket_start + bucket_size
            bucket_frames = [f for f in frames if bucket_start <= f[0] < bucket_end]
            total = len(bucket_frames)
            s1 = sum(1 for f in bucket_frames if f[1] <= 236)
            s2 = sum(1 for f in bucket_frames if 236 < f[1] <= 472)
            s4 = sum(1 for f in bucket_frames if f[1] > 472)
            bar = "#" * min(total, 50)
            label = f"{b*bucket_size:.0f}-{(b+1)*bucket_size:.0f}s"
            print(f"  {label:>8s}  {total:5d}  {s1:5d}  {s2:5d}  {s4:5d}  {bar}")

    # RSSI stats
    if frames:
        rssis = [f[2] for f in frames]
        print(f"\nRSSI: min={min(rssis)}dBm max={max(rssis)}dBm "
              f"avg={sum(rssis)/len(rssis):.1f}dBm")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Sustained multi-packet burst delivery test (dual-radio)')
    parser.add_argument('--tx-port', default=None,
                        help='Serial port for TX radio (e.g. /dev/ttyACM1)')
    parser.add_argument('--rx-port', default=None,
                        help='Serial port for RX radio (e.g. /dev/ttyACM2)')
    parser.add_argument('--tx-only', action='store_true',
                        help='Only transmit (no RX listener)')
    parser.add_argument('--rx-only', action='store_true',
                        help='Only receive (no TX)')
    parser.add_argument('--duration', type=int, default=DEFAULT_DURATION_S,
                        help=f'Test duration in seconds (default: {DEFAULT_DURATION_S})')
    parser.add_argument('--interval', type=int, default=TX_INTERVAL_MS,
                        help=f'TX interval in ms (default: {TX_INTERVAL_MS})')
    parser.add_argument('--node-id', type=int, default=1,
                        help='Node ID for payload generation (default: 1)')
    args = parser.parse_args()

    tx_radio = None
    rx_radio = None
    rx_frames = []  # list of (timestamp, plen, rssi, crc_ok)

    try:
        # ── Determine mode ─────────────────────────────────────────────
        shared_port = (args.tx_port and args.rx_port
                       and args.tx_port == args.rx_port)

        # ── Open radios ────────────────────────────────────────────────
        if not args.rx_only and args.tx_port:
            print(f"Opening TX radio on {args.tx_port}...")
            tx_radio = RadioPort(args.tx_port, "TX")
            print("  Sending CONFIG...")
            send_config(tx_radio)
            max_pl = wait_ready(tx_radio)
            if max_pl is None:
                print("FAILED: TX radio not ready")
                sys.exit(1)
            print(f"  TX radio ready (max payload: {max_pl})")

        if shared_port:
            rx_radio = tx_radio  # same port for both
            print(f"  (shared port mode: RX on same port as TX)")
        elif not args.tx_only and args.rx_port:
            print(f"Opening RX radio on {args.rx_port}...")
            rx_radio = RadioPort(args.rx_port, "RX")
            print("  Sending CONFIG...")
            send_config(rx_radio)
            max_pl = wait_ready(rx_radio)
            if max_pl is None:
                print("FAILED: RX radio not ready")
                sys.exit(1)
            print(f"  RX radio ready (max payload: {max_pl})")

        if not tx_radio and not rx_radio:
            print("ERROR: must specify --tx-port and/or --rx-port")
            sys.exit(1)

        print(f"\n{'='*70}")
        mode = "dual-radio" if tx_radio and rx_radio and tx_radio is not rx_radio else \
               "shared" if shared_port else \
               "TX-only" if not rx_radio else "RX-only"
        print(f" Mode: {mode}")
        print(f" Running sustained burst test for {args.duration}s")
        print(f" TX interval: {args.interval}ms, sizes: {PAYLOAD_SIZES}")
        print(f"{'='*70}\n")

        tx_count = 0
        start_time = time.time()

        # ── Run TX and RX ──────────────────────────────────────────────
        threads = []

        if tx_radio:
            if shared_port:
                # Single thread: TX interleaved with RX parsing
                def tx_with_rx():
                    nonlocal tx_count
                    tx_count = run_tx_loop(
                        tx_radio, args.duration, args.interval,
                        args.node_id, shared_radio=rx_radio)
                    # Drain remaining RX after TX loop ends
                    if rx_radio:
                        for ts, plen, rssi, payload in parse_rx_frames(rx_radio):
                            if plen >= 4:
                                body = payload[:-4]
                                exp = struct.unpack('<I', payload[-4:])[0]
                                act = zlib.crc32(body) & 0xFFFFFFFF
                                ok = exp == act
                            else:
                                ok = True
                            rx_frames.append((ts, plen, rssi, ok))
                tx_with_rx()
            else:
                def tx_thread():
                    nonlocal tx_count
                    tx_count = run_tx_loop(tx_radio, args.duration,
                                           args.interval, args.node_id)
                t = threading.Thread(target=tx_thread, daemon=True)
                threads.append(t)
                t.start()

        if rx_radio and not shared_port:
            def rx_thread():
                run_rx_logger(rx_radio, args.duration, rx_frames)
            t = threading.Thread(target=rx_thread, daemon=True)
            threads.append(t)
            t.start()

        # Wait for completion
        for t in threads:
            t.join()

        elapsed = time.time() - start_time
        print(f"\nTest completed in {elapsed:.1f}s")

        # ── Report ─────────────────────────────────────────────────────
        if rx_radio and rx_frames:
            print_report(rx_frames, args.duration, tx_count)
        elif tx_radio:
            print(f"TX-only mode: sent {tx_count} payloads in {elapsed:.1f}s")
        else:
            print("No frames received.")

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if tx_radio:
            tx_radio.close()
        if rx_radio and rx_radio is not tx_radio:
            rx_radio.close()


if __name__ == '__main__':
    main()
