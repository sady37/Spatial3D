"""Per-2s HR/RR from radar cubes (product-cadence) — GATED by the living-person gate.

Trailing window HR=16s / RR=30s, step 2s. BEFORE reporting vitals, each window must
pass living_gate (RR-band spatial concentration + tight cluster = a real breathing
person at a range). If not present -> report "no person" (blank HR/RR), never fabricate.
This closes the empty-chair fabrication (which otherwise reports a band-center HR ~80).

    .venv/bin/python3 hrrr_2s.py chairL
"""
import sys, glob, csv, datetime as dt
import numpy as np
from bcg_vitals import demod_channels, estimate_rr, estimate_hr
from living_gate import living_window

HRWIN, RRWIN, STEP = 16.0, 30.0, 2.0
prefix = sys.argv[1] if len(sys.argv) > 1 else "chairL"
files = sorted(glob.glob(f"{prefix}_*.npz"))

rows = []
for f in files:
    d = np.load(f, allow_pickle=True)
    cube = np.asarray(d["snapshots"], np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int); dr = float(d["dr_m"])
    try:
        ts = d["frame_ts"]; fps = len(ts) / (ts[-1] - ts[0])
    except KeyError:
        n0 = int(counts.min()); fps = 18.8; ts = np.arange(n0) / fps  # old cubes: synthetic ts
    chans = demod_channels(cube[:, :int(counts.min()), :], bins)
    n = chans.shape[1]
    hw, rw, st = int(HRWIN * fps), int(RRWIN * fps), int(STEP * fps)
    # pass 1: raw living-gate flag + range per window (occupancy is a slow signal)
    raw, rng, tcs = [], [], []
    for end in range(rw, n + 1, st):
        live = living_window(chans[:, end - rw:end], bins, dr, fps)
        raw.append(1 if live["present"] else 0); rng.append(live["range_m"]); tcs.append(ts[end - 1])
    raw = np.array(raw)
    # smooth: occupancy = trailing ~30s majority vote (don't flip every 2s)
    K = max(1, int(30 / STEP))
    occ = np.array([1 if raw[max(0, i - K + 1):i + 1].mean() >= 0.5 else 0 for i in range(len(raw))])
    # pass 2: report vitals only where occupied; else no-person
    for j, end in enumerate(range(rw, n + 1, st)):
        if not occ[j]:
            rows.append((tcs[j], None, None, None, 0, rng[j])); continue
        segR = chans[:, end - rw:end]; segH = chans[:, end - hw:end]
        rr, f0, _, _ = estimate_rr(segR, fps)
        hr = estimate_hr(segH, fps, f0)
        rows.append((tcs[j], hr["hr"], rr, hr["strength"], 1, rng[j]))

if not rows:
    print("no windows (need >=30s per file)"); sys.exit(0)
t0 = rows[0][0]
out = f"{prefix}_hrrr_2s.csv"
with open(out, "w", newline="") as fp:
    w = csv.writer(fp)
    w.writerow(["local_time", "t_rel_s", "hr", "rr", "strength", "occ", "range_m"])
    for tc, hr, rr, s, occ, rng in rows:
        w.writerow([dt.datetime.fromtimestamp(tc).strftime("%H:%M:%S"), round(tc - t0, 1),
                    round(hr, 1) if hr else "", round(rr) if rr else "",
                    round(s, 2) if s else "", occ, rng])
n_person = sum(r[4] for r in rows)
hrv = np.array([r[1] for r in rows if r[1]])
print(f"saved {out}: {len(rows)} rows @2s  |  person={n_person} ({n_person/len(rows):.0%})  "
      f"no-person={len(rows)-n_person}")
if len(hrv):
    print(f"HR (person windows only): med={np.median(hrv):.1f} [{hrv.min():.0f},{hrv.max():.0f}]")
else:
    print("no-person the whole time -> NO HR reported (gate held; nothing fabricated)")
