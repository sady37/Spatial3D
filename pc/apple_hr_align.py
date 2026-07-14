"""Parse Apple Health export + align its HR samples to a radar cube's wall-clock, to
use Apple Watch HR as ground truth for radar HR validation.

RECORD PROTOCOL: on the watch start a Workout ("Other") DURING the radar capture ->
HR sampled ~every 5s (background is far sparser). Then iPhone Health app -> profile ->
"Export All Health Data" -> export.zip -> export.xml.  (Or a CSV app -> use --csv.)

Watch optical HR lags 10-30s in RECOVERY -> trust it for RESTING; use a chest-strap
ECG for dynamic (tachy) segments.

    python3 apple_hr_align.py export.xml --npz chairL_20260713_183013.npz
    python3 apple_hr_align.py hr.csv --csv --t0 "2026-07-13 18:30:13"   # CSV route
"""
import argparse, csv, sys
import xml.etree.ElementTree as ET
from datetime import datetime
import numpy as np


def _to_epoch(s):
    """Apple dates look like '2026-07-13 18:30:10 +0800'."""
    for fmt in ("%Y-%m-%d %H:%M:%S %z", "%Y-%m-%d %H:%M:%S"):
        try:
            return datetime.strptime(s.strip(), fmt).timestamp()
        except ValueError:
            continue
    return None


def parse_health_xml(path):
    """Stream export.xml -> sorted np arrays (epoch_s, bpm) for HeartRate records."""
    out = []
    for _, el in ET.iterparse(path, events=("end",)):
        if el.tag == "Record" and el.get("type") == "HKQuantityTypeIdentifierHeartRate":
            e = _to_epoch(el.get("startDate", ""))
            try:
                v = float(el.get("value"))
            except (TypeError, ValueError):
                v = None
            if e and v:
                out.append((e, v))
            el.clear()
    out.sort()
    a = np.array(out)
    return a[:, 0], a[:, 1]


def parse_csv(path):
    """Generic CSV with a time column and a bpm column (auto-detect)."""
    ep, hr = [], []
    with open(path) as f:
        rows = list(csv.reader(f))
    hdr = [h.lower() for h in rows[0]]
    ti = next((i for i, h in enumerate(hdr) if "date" in h or "time" in h), 0)
    hi = next((i for i, h in enumerate(hdr) if "heart" in h or "bpm" in h or "rate" in h), 1)
    for r in rows[1:]:
        e = _to_epoch(r[ti]) if not r[ti].replace(".", "").isdigit() else float(r[ti])
        try:
            v = float(r[hi])
        except (ValueError, IndexError):
            continue
        if e:
            ep.append(e); hr.append(v)
    o = np.array(sorted(zip(ep, hr)))
    return o[:, 0], o[:, 1]


def radar_walltime(npz):
    """Per-frame epoch seconds from a radar cube (block_start_epoch + frame_ts)."""
    d = np.load(npz, allow_pickle=True)
    n = int(d["counts"].astype(int).min())
    ft = np.asarray(d["frame_ts"], float)[:n]
    if ft.max() > 1e12:                       # ms epoch
        ft = ft / 1000.0
    if ft[0] < 1e8 and "block_start_epoch" in d.files:   # relative -> add block start
        ft = ft + float(d["block_start_epoch"])
    fps = (len(ft) - 1) / (ft[-1] - ft[0])
    return ft, fps


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("hr_file"); ap.add_argument("--csv", action="store_true")
    ap.add_argument("--npz", help="radar cube to align to")
    ap.add_argument("--win", type=float, default=30.0, help="HR window (s)")
    ap.add_argument("--step", type=float, default=10.0)
    a = ap.parse_args()

    ep, hr = (parse_csv if a.csv else parse_health_xml)(a.hr_file)
    print(f"parsed {len(hr)} HR samples: {datetime.fromtimestamp(ep[0])} .. "
          f"{datetime.fromtimestamp(ep[-1])}, median {np.median(hr):.0f} bpm, "
          f"median gap {np.median(np.diff(ep)):.0f}s")
    if np.median(np.diff(ep)) > 20:
        print("  ! sparse sampling (>20s) — was a Workout running? background HR is too "
              "coarse for per-window validation.")

    if not a.npz:
        print("\n(no --npz) HR CSV ready. Re-run with --npz <cube> to align to a capture.")
        return
    ft, fps = radar_walltime(a.npz)
    print(f"\nradar {a.npz}: {len(ft)} frames @ {fps:.2f}fps, "
          f"{datetime.fromtimestamp(ft[0])} .. {datetime.fromtimestamp(ft[-1])}")
    ov0, ov1 = max(ep[0], ft[0]), min(ep[-1], ft[-1])
    if ov1 <= ov0:
        print("  !! NO time overlap between watch HR and radar — check clocks/timezone.")
        return
    print(f"overlap {ov1-ov0:.0f}s -> truth(t) trajectory (watch HR interpolated):")
    print(f"  {'t_win(s)':>9} {'wall-clock':>20} {'watch HR':>9}")
    t = ov0 + a.win / 2
    while t + a.win / 2 <= ov1:
        m = (ep >= t - a.win / 2) & (ep <= t + a.win / 2)
        val = np.mean(hr[m]) if m.any() else np.interp(t, ep, hr)
        print(f"  {t-ft[0]:9.0f} {datetime.fromtimestamp(t).strftime('%H:%M:%S'):>20} "
              f"{val:9.0f}")
        t += a.step
    print("\nalign a radar HR estimate on the same window centers to score MAE / r.")


if __name__ == "__main__":
    main()
