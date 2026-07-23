"""Rolling whole-room baseline/variance store fed by the periodic wide cube sweep.

WHY (user 2026-07-23): the fall presence test is 差值/基值 -- (P_live - P_empty)/P_empty vs the
FIXED install baseline -- compared against ONE global threshold (1.0). That threshold is a guess:
we have never measured how much a bin drifts when NOTHING changes. Session 0723 showed the cost --
z40 came back 320.44 / -0.44 / 72.08 / 41.57 / 28.79 / 15.91 / 11.24 across the night, four orders
of magnitude, all compared against the same 0.4. And bin 35 (3.71m) is the top peak in three
UNRELATED captures including an empty room, i.e. a chronically unstable bin nobody had noticed.

WHAT THIS IS NOT: it is NOT a re-record of the install background. The hard rule stands -- the
empty-room MEAN is captured once at install and never re-recorded, because you cannot ask an old
person to arrange an empty room before they fall. What accumulates here is
  (a) samples taken only at moments a bin is PROVEN unoccupied (track/cloud masked out), and
  (b) the DISPERSION of those samples,
which is a different statistic from the install mean and cannot absorb a body that is present.

DESIGN (user spec):
  * every SWEEP_PERIOD_S, 2 shots cover the room (cube_sweep's SHOTS: bins 1-39 + 32-64)
  * mask every bin within MASK_R_M (1 m) of ANY current track / below-floor cloud mass -- a person
    contaminates NEIGHBOURING bins through multipath, not just their own, so masking the single
    occupied bin is not enough
  * keep RAW per-sweep per-bin power for RETAIN_DAYS (7), and derive every statistic on demand
    (whole-day, hour-of-day, last-2h) rather than pre-aggregating into a fixed shape

Raw retention is deliberate: 64 bins x 12 sweeps/day x 7 days = ~5.4k floats (~43 KB). At that
size there is no reason to collapse into Welford accumulators and lose the ability to re-slice.

TWO LAYERS (user 2026-07-23 "留这些数据, 便于以后房间绘制训练"):
  1. ROLLING (baseline_sweeps.npz) -- trace power + mask only, last RETAIN_DAYS. Small and fast;
     this is what the live presence threshold reads.
  2. ARCHIVE (baseline_archive/sweep_YYYYMMDD.npz) -- NEVER pruned, one file per day, holding the
     full per-bin 16x16 COVARIANCE plus the track positions at sweep time. Trace power is a scalar
     and cannot do DOA: the room-drawing chain (build_static_scene / scene_layers -> MUSIC ->
     azimuth) needs the covariance, so an archive of powers would be useless for exactly the future
     use it is being kept for. Storing the concurrent track xy alongside gives every sweep a free
     occupancy label for supervised work later.
     Cost: 64 bins x 16x16 complex64 = 131 KB/sweep, ~1.6 MB/day at a 2 h cadence, ~0.6 GB/year.
"""
from __future__ import annotations
import os
import time
import numpy as np

MAX_BIN = 64                 # store a fixed-width row so slices stay trivial
RETAIN_DAYS = 7.0            # ROLLING window only -- the archive below is never pruned
_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "record",
                     "baseline_sweeps.npz")
_ARCHIVE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "record",
                            "baseline_archive")


def archive_sweep(cov_by_bin, masked_bins, tracks_xy=(), ts=None):
    """Append the FULL covariance sweep to the permanent per-day archive (never pruned).

    cov_by_bin : {bin: (16,16) complex ndarray}
    tracks_xy  : [(x, y), ...] track positions at sweep time -- the occupancy label
    Kept separate from the rolling store so the live threshold path stays small and fast."""
    ts = time.time() if ts is None else float(ts)
    try:
        os.makedirs(_ARCHIVE_DIR, exist_ok=True)
        day = time.strftime("%Y%m%d", time.localtime(ts))
        path = os.path.join(_ARCHIVE_DIR, f"sweep_{day}.npz")
        bins = sorted(int(b) for b in cov_by_bin)
        cov = np.stack([np.asarray(cov_by_bin[b], np.complex64) for b in bins])[None, ...]
        rec = {"ts": np.array([ts]),
               "bins": np.array(bins, np.int16)[None, :],
               "cov": cov,
               "mask": np.array([[b not in masked_bins for b in bins]], bool),
               "tracks": np.array([_pad_tracks(tracks_xy)], np.float32)}
        if os.path.exists(path):
            old = np.load(path, allow_pickle=False)
            if old["bins"].shape[1] == len(bins):        # same sweep geometry -> stack
                rec = {k: np.concatenate([old[k], rec[k]]) for k in rec}
            else:                                        # geometry changed -> start a new file
                path = os.path.join(_ARCHIVE_DIR, f"sweep_{day}_{len(bins)}b.npz")
                if os.path.exists(path):
                    old = np.load(path, allow_pickle=False)
                    rec = {k: np.concatenate([old[k], rec[k]]) for k in rec}
        _atomic_savez(path, **rec)
        return path
    except Exception:
        return None


def _pad_tracks(tracks_xy, n=4):
    a = np.full((n, 2), np.nan, np.float32)
    for i, (x, y) in enumerate(list(tracks_xy)[:n]):
        a[i] = (x, y)
    return a


def _load():
    """-> (ts[n], power[n,MAX_BIN], mask[n,MAX_BIN] bool). mask=True means USABLE (unoccupied)."""
    try:
        d = np.load(_PATH, allow_pickle=False)
        return d["ts"], d["power"], d["mask"].astype(bool)
    except Exception:
        return (np.zeros(0), np.zeros((0, MAX_BIN), np.float64),
                np.zeros((0, MAX_BIN), bool))


def record_sweep(power_by_bin, masked_bins, ts=None, path=None):
    """Append one sweep. power_by_bin: {bin: trace-power}. masked_bins: set of bins to EXCLUDE
    (occupied / multipath-contaminated). Rows older than RETAIN_DAYS are dropped."""
    ts = time.time() if ts is None else float(ts)
    p = np.full(MAX_BIN, np.nan)
    m = np.zeros(MAX_BIN, bool)
    for b, v in power_by_bin.items():
        b = int(b)
        if 0 <= b < MAX_BIN and np.isfinite(v):
            p[b] = float(v)
            m[b] = b not in masked_bins
    global _PATH
    _p = path or _PATH
    try:
        d = np.load(_p, allow_pickle=False)
        T, P, M = d["ts"], d["power"], d["mask"].astype(bool)
    except Exception:
        T = np.zeros(0); P = np.zeros((0, MAX_BIN)); M = np.zeros((0, MAX_BIN), bool)
    T = np.concatenate([T, [ts]])
    P = np.vstack([P, p[None, :]])
    M = np.vstack([M, m[None, :]])
    keep = T >= ts - RETAIN_DAYS * 86400.0
    T, P, M = T[keep], P[keep], M[keep]
    _atomic_savez(_p, ts=T, power=P, mask=M)
    return len(T)


def _atomic_savez(path, **arrays):
    """savez_compressed to a tmp file then rename. NOTE: np.savez appends '.npz' when handed a
    NAME that lacks it -- which silently breaks tmp+rename -- so hand it a file OBJECT."""
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "wb") as fh:
        np.savez_compressed(fh, **arrays)
    os.replace(tmp, path)


def stats(hour=None, since_s=None, min_n=3, path=None):
    """Per-bin robust baseline + dispersion over the USABLE (unmasked) samples.

    hour     : restrict to this hour-of-day (diurnal slice; the drift being tracked is thermal)
    since_s  : restrict to the last N seconds (e.g. 7200 for the user's "recent 2h" view)
    Returns {bin: {"med", "sigma", "n"}} -- MEDIAN not mean, so a handful of samples where the
    mask failed to catch an occupant cannot drag the baseline. sigma uses MAD*1.4826 for the same
    reason: a robust spread is what a threshold should be built on."""
    T, P, M = _load() if path is None else (lambda d: (d["ts"], d["power"], d["mask"].astype(bool)))(
        np.load(path, allow_pickle=False))
    if len(T) == 0:
        return {}
    sel = np.ones(len(T), bool)
    if since_s is not None:
        sel &= T >= time.time() - float(since_s)
    if hour is not None:
        hrs = np.array([time.localtime(t).tm_hour for t in T])
        sel &= hrs == int(hour)
    if not sel.any():
        return {}
    P, M = P[sel], M[sel]
    out = {}
    for b in range(MAX_BIN):
        v = P[:, b][M[:, b] & np.isfinite(P[:, b])]
        if len(v) < min_n:
            continue
        med = float(np.median(v))
        mad = float(np.median(np.abs(v - med)))
        out[b] = {"med": med, "sigma": 1.4826 * mad, "n": int(len(v))}
    return out


def summary(path=None):
    """Compact health view: coverage + which bins are chronically unstable (sigma/med)."""
    T, P, M = _load() if path is None else (lambda d: (d["ts"], d["power"], d["mask"].astype(bool)))(
        np.load(path, allow_pickle=False))
    if len(T) == 0:
        return {"sweeps": 0}
    st = stats(path=path)
    worst = sorted(((b, s["sigma"] / s["med"]) for b, s in st.items() if s["med"] > 0),
                   key=lambda t: -t[1])[:8]
    return {"sweeps": int(len(T)),
            "span_h": round(float(T.max() - T.min()) / 3600.0, 1),
            "bins_with_stats": len(st),
            "usable_frac": round(float(M.mean()), 3),
            "unstable_top": [(b, round(r, 2)) for b, r in worst]}
