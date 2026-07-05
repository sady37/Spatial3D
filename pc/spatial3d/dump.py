"""Raw / structured diagnostic for the radar DATA port.

Run this FIRST when bringing up the link — it tells you whether a port is
streaming, whether the TI magic word is present, and dumps frame headers +
TLV type/length so you can confirm the format against the L-SDK demo source.

    python -m spatial3d.dump --data-port /dev/cu.usbmodem0000RA444
    python -m spatial3d.dump --data-port /dev/cu.usbmodem0000RA444 --raw
"""

from __future__ import annotations

import argparse
import sys
import time

from .tlv import MAGIC, parse_frame, read_frame
from .uart_reader import DATA_BAUD, open_serial


def raw_probe(port: str, baud: int, seconds: float) -> None:
    ser = open_serial(port, baud, timeout=0.3)
    ser.reset_input_buffer()
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < seconds:
        d = ser.read(4096)
        if d:
            buf.extend(d)
    ser.close()

    print(f"[raw] {port}@{baud}: {len(buf)} bytes in {seconds:.0f}s")
    if not buf:
        print("      (无字节 — 该口没在吐数据,或波特率不对,或不是 DATA 口)")
        return
    hit = buf.find(MAGIC)
    print(f"      TI magic: {'found @ offset ' + str(hit) if hit >= 0 else 'NOT found'}")
    print(f"      first 48 bytes: {buf[:48].hex(' ')}")


def structured(port: str, baud: int, n_frames: int) -> None:
    ser = open_serial(port, baud, timeout=2.0)
    try:
        for i in range(n_frames):
            frame = read_frame(ser)
            h = frame.header
            print(f"\n--- frame #{h.frame_number} "
                  f"(len={h.total_packet_len}, tlvs={h.num_tlvs}, "
                  f"objs={h.num_detected_obj}) ---")
            for t in frame.tlvs:
                print(f"    TLV type={t.type} len={len(t.payload)}")
            pts = frame.detected_points()
            if len(pts):
                print(f"    detected points: {len(pts)}, first 3:")
                for p in pts[:3]:
                    print(f"      x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f} v={p[3]:.3f}")
    finally:
        ser.close()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="TI radar DATA-port diagnostic")
    ap.add_argument("--data-port", required=True)
    ap.add_argument("--baud", type=int, default=DATA_BAUD)
    ap.add_argument("--raw", action="store_true", help="hexdump only, no TLV parse")
    ap.add_argument("--seconds", type=float, default=2.0, help="raw probe window")
    ap.add_argument("--frames", type=int, default=5, help="frames to parse")
    args = ap.parse_args(argv)

    if args.raw:
        raw_probe(args.data_port, args.baud, args.seconds)
    else:
        raw_probe(args.data_port, args.baud, args.seconds)
        print("\n[structured] parsing TLV frames...")
        try:
            structured(args.data_port, args.baud, args.frames)
        except Exception as e:  # noqa: BLE001 - diagnostic, show anything
            print(f"parse failed: {e}", file=sys.stderr)
            print("→ 用 --raw 看原始字节,再对照 L-SDK demo 的 output.h 调 tlv.py")
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
