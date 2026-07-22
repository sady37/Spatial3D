"""Occupancy detection by 差值/基值 = (live-empty)/empty vs the FIXED install background.

Per range bin: ratio = (P_live - P_empty) / P_empty, P = trace(covariance).
- The RATIO (not absolute diff) solves 近大远小: near reflectors have huge base+diff,
  a far body small base+diff; the fraction makes them comparable.
- A body lying on the floor forms a body-floor DIHEDRAL (retroreflector) that ADDS a
  2.5-4x (=+4..+6dB) return over a contiguous block. Thermal multipath drift is a
  <1dB LOCAL vector-sum wobble -- it CANNOT multiply a large contiguous region by 2-4x,
  so `ratio >= 1 (2x)` over adjacent bins = a real body, unfakeable by multipath.
- Works vs a FIXED install background across power-cycles (no re-record, no same-session
  empty). See memory: ratio-vs-background-detects-lying, no-rerecord-background.
- Detects PRESENCE/占据 of a floor body, NOT posture (posture = on-chip z). Azimuth is
  coarse/specular-limited so x is a rough bearing; RANGE is the exact axis.
"""
from __future__ import annotations
import os
import math
import numpy as np

from .build_static_scene import (real_array, AZ, to_ground, NEAR_GATE,
                                 AZ_CAL_K, AZ_CAL_OFF)

# --- 思路B z<=40 half-angle floor-band geometry (mount 2.0m, down-tilt 25deg) ---
Z40_H = 2.0
Z40_TILT = 25.0
Z40_ZMAX = 0.4

_HERE = os.path.dirname(os.path.abspath(__file__))
_EMPTY = os.path.join(_HERE, "..", "case", "empty_20260721.npz")   # FIXED install background


def _power(npz):
    d = np.load(npz, allow_pickle=True)
    b = d["bins"].astype(int)
    cov = d["covariances"]
    dr = float(d["dr_m"])
    p = np.array([np.real(np.trace(cov[i])) for i in range(len(b))])
    return b, p, cov, dr


def detect_occupancy(live_npz, empty_npz=_EMPTY, near_gate=None, thr=1.0, min_block=1,
                     wall_gate=5.8, shadow_thr=-0.2):
    """差值/基值 occupancy detector.

    thr        : ratio threshold (1.0 = signal doubled = +3dB). Multipath can't fake this
                 over a contiguous block.
    min_block  : min contiguous bins in a block (1 keeps compact/far bodies; >=2 is stricter).
    wall_gate  : a floor body is < this range (m); blocks beyond = the far wall / bay-window
                 multipath ghost -> not a body.
    shadow_thr : a real floor body OCCLUDES -> a negative-ratio neighbour (shadow); a wall
                 ghost has none. A block is `confirmed` iff it clears BOTH gates.
    Returns {range_m, ratio, blocks:[...{confirmed}]}. blocks sorted, confirmed first.
    """
    near_gate = NEAR_GATE if near_gate is None else near_gate
    bL, PL, covL, dr = _power(live_npz)
    bE, PE, covE, _ = _power(empty_npz)
    iE = {int(x): i for i, x in enumerate(bE)}
    iL = {int(x): i for i, x in enumerate(bL)}
    common = np.intersect1d(bL, bE)                     # bin-NUMBER alignment (32-bin vs 63-bin)
    rng = common * dr
    ratio = np.array([(PL[iL[int(x)]] - PE[iE[int(x)]]) / (PE[iE[int(x)]] + 1e-12) for x in common])

    keep = rng >= near_gate
    hot = keep & (ratio >= thr)
    idx = np.where(hot)[0]
    blocks = []
    if len(idx):
        arr = real_array()
        runs = np.split(idx, np.where(np.diff(idx) > 1)[0] + 1)
        for run in runs:
            if len(run) < min_block:
                continue
            pk = int(run[np.argmax(ratio[run])])
            R = covL[iL[int(common[pk])]]
            P = np.array([np.real(a.conj() @ R @ a) for a in
                          (arr.steering_vector(az, 0.0) for az in AZ)])
            az = AZ[int(np.argmax(P))]
            x, y = to_ground(rng[pk], az)
            # occlusion shadow = negative ratio just BEFORE the block (body darkens what's behind
            # it in range from the radar's view -> actually the bins in FRONT that it steals from);
            # report the min ratio in the 3 bins leading into the block as a confirming signature.
            lead = range(max(0, run[0] - 3), run[0])
            shadow = float(np.min(ratio[list(lead)])) if len(lead) else 0.0
            confirmed = bool(rng[pk] < wall_gate and shadow < shadow_thr)
            blocks.append({"r0": round(float(rng[run[0]]), 2), "r1": round(float(rng[run[-1]]), 2),
                           "span_m": round(float(rng[run[-1]] - rng[run[0]]), 2),
                           "peak_ratio": round(float(ratio[pk]), 2),
                           "peak_range": round(float(rng[pk]), 2), "nbins": int(len(run)),
                           "x": round(float(x), 2), "y": round(float(y), 2),
                           "shadow": round(shadow, 2), "confirmed": confirmed})
    blocks.sort(key=lambda b: (not b["confirmed"], -b["peak_ratio"]))
    return {"range_m": rng, "ratio": ratio, "blocks": blocks}


def _xy_power(cov, bins, dr, xs, ys, cell, near_gate):
    """Accumulate RAW az-power per 10cm X/Y cell (no per-bin normalisation, so the
    cross-capture ratio is meaningful). Each range bin smears over its ~29deg az arc."""
    arr = real_array()
    G = np.zeros((len(ys), len(xs)))
    for i in range(len(bins)):
        r = bins[i] * dr
        if r < near_gate:
            continue
        R = cov[i]
        P = np.array([np.real(a.conj() @ R @ a) for a in
                      (arr.steering_vector(az, 0.0) for az in AZ)])
        gx, gy = to_ground(r, AZ)
        ix = np.round((gx - xs[0]) / cell).astype(int)
        iy = np.round((gy - ys[0]) / cell).astype(int)
        ok = (ix >= 0) & (ix < len(xs)) & (iy >= 0) & (iy < len(ys))
        for j in np.where(ok)[0]:
            G[iy[j], ix[j]] += P[j]
    return G


def detect_occupancy_xy(live_npz, empty_npz=_EMPTY, cell=0.10, thr=1.0, min_cells=30,
                        near_gate=None, wall_gate=5.8, xlim=3.0, ylim=6.0):
    """2D 差值/基值 on a 10x10cm X/Y cell grid + a MINIMUM connected-cell-count gate, so a
    single strong point reflector (few cells) is NOT called a body. Validated 2026-07-22:
    lying/sitting largest-block 59-98 cells, standing 9, empty 0 -> min_cells~30 separates.
    CAVEAT: absolute counts are az-smear-inflated (coarse 29deg beam), so this is a RELATIVE
    size discriminator, not a true footprint; pair with the shadow gate for strong point glints.
    Returns {occupied, cells, largest_block, centroid(x,y)}."""
    from scipy import ndimage
    near_gate = NEAR_GATE if near_gate is None else near_gate
    bL, _, covL, dr = _power(live_npz)
    bE, _, covE, _ = _power(empty_npz)
    xs = np.arange(-xlim, xlim + cell, cell)
    ys = np.arange(0.0, ylim + cell, cell)
    GL = _xy_power(covL, bL, dr, xs, ys, cell, near_gate)
    GE = _xy_power(covE, bE, dr, xs, ys, cell, near_gate)
    ratio = (GL - GE) / (GE + 1e-9)
    hot = ratio >= thr
    hot[ys[:, None].repeat(len(xs), 1) >= wall_gate] = False      # drop far-wall ghost band
    lab, n = ndimage.label(hot)
    if n == 0:
        return {"occupied": False, "cells": 0, "largest_block": 0, "centroid": None}
    sizes = ndimage.sum(np.ones_like(lab), lab, range(1, n + 1))
    big = int(np.argmax(sizes)) + 1
    bs = int(sizes[big - 1])
    yi, xi = np.where(lab == big)
    cen = (round(float(xs[xi].mean()), 2), round(float(ys[yi].mean()), 2))
    return {"occupied": bs >= min_cells, "cells": int(hot.sum()),
            "largest_block": bs, "centroid": cen}


# ======================= 思路B: z<=40 half-angle accumulation =======================
# For each range R, the FLOOR band z in [0, ZMAX] maps (half-angle geometry) to a KNOWN
# elevation window el in [asin((H-ZMAX)/R)-TILT, asin(H/R)-TILT] rel boresight. We ACCUMULATE
# the Bartlett power over that window (blurry -- the 29deg beam is wider than the window --
# but that's fine: the SAME blurry window is applied to the background AND live, so the error
# CANCELS in the ratio ("错都是一样的错"). Validated 2026-07-22: lying(z<=40)->POSITIVE (and
# AMPLIFIED vs omni: less high-layer dilution), sitting/standing(z>40)->NEGATIVE, empty->~0.
def _z40_els(R, n=4):
    if Z40_H / R >= 1.0:                       # floor unreachable at this slant range
        return None
    t_lo = math.degrees(math.asin(min(1.0, (Z40_H - Z40_ZMAX) / R)))   # z=ZMAX depression
    t_hi = math.degrees(math.asin(min(1.0, Z40_H / R)))                # z=0 depression
    return np.linspace(t_lo - Z40_TILT, t_hi - Z40_TILT, n)            # el window rel boresight


def _z40_power(cov, bins, dr, azg, arr):
    """Per range bin: accumulate Bartlett power over the z<=40 el window x azimuth."""
    prof = np.full(len(bins), np.nan)
    for i in range(len(bins)):
        els = _z40_els(bins[i] * dr)
        if els is None:
            continue
        Rc = cov[i]
        prof[i] = sum(np.real(np.vdot(a, Rc @ a))
                      for e in els for a in (arr.steering_vector(az, math.radians(e)) for az in azg))
    return prof


def detect_fall_z40(live_npz, empty_npz=_EMPTY, thr=1.0, omni_thr=1.0, az_step=3.0,
                    near=2.1, wall=5.8):
    """思路B FALL detector: z<=40 half-angle accumulation, 差值/基值 vs the FIXED background.
    Returns {fallen, peak_ratio, peak_range, profile}. Lying floor body -> peak_ratio >= thr;
    sitting/standing -> negative (rejected, z>40); empty -> ~0. This is fall-SPECIFIC (omni
    would also fire on sitting). `near`/`wall` bound the floor-reachable range and drop the
    far-wall (bay-window) multipath ghost band that otherwise spikes the peak."""
    arr = real_array()
    azg = np.deg2rad(np.arange(-50, 50.001, az_step))
    bL, oL, covL, dr = _power(live_npz)       # oL = omni power (trace) per bin
    bE, oE, covE, _ = _power(empty_npz)
    zL = _z40_power(covL, bL, dr, azg, arr)
    zE = _z40_power(covE, bE, dr, azg, arr)
    iL = {int(x): k for k, x in enumerate(bL)}
    iE = {int(x): k for k, x in enumerate(bE)}
    # Anchor on the body's DOMINANT reflector = the max-OMNI bin in the floor window (torso for a
    # sitter, whole body for a lyer -- NOT the sitter's feet, which are a weaker forward bin). Then
    # z<=40 THERE labels fallen: a lyer's dominant bin is on the floor (positive); a sitter's/stander's
    # dominant bin (torso) is elevated (negative) -> rejected. omni_thr rejects empty/noise.
    best_b, best_omni = None, omni_thr
    for b in sorted(set(iL) & set(iE)):
        R = b * dr
        if not (near <= R <= wall) or oE[iE[b]] <= 0:
            continue
        omni_r = (oL[iL[b]] - oE[iE[b]]) / oE[iE[b]]
        if omni_r >= best_omni:
            best_omni, best_b = omni_r, b
    if best_b is None:                                   # no body present
        return {"fallen": False, "peak_ratio": 0.0, "peak_range": None, "peak_omni": 0.0}
    ez = zE[iE[best_b]]
    z40_r = (zL[iL[best_b]] - ez) / ez if (not np.isnan(ez) and ez > 0) else -9.0
    return {"fallen": bool(z40_r >= thr), "peak_ratio": round(float(z40_r), 2),
            "peak_range": round(best_b * dr, 2), "peak_omni": round(float(best_omni), 2)}


def z40_cell_map(npz, cell=0.10, xlim=3.0, ylim=6.0, az_step=2.0):
    """Per-cell z<=40 floor-band intensity map (for the ratio figure). Projects each
    (range, az)'s z<=40-window power to its floor cell (z=0 -> ground g=sqrt(R^2-H^2))."""
    arr = real_array()
    azg = np.deg2rad(np.arange(-50, 50.001, az_step))
    b, _, cov, dr = _power(npz)
    xs = np.arange(-xlim, xlim + cell, cell); ys = np.arange(0.0, ylim + cell, cell)
    G = np.zeros((len(ys), len(xs)))
    for i in range(len(b)):
        els = _z40_els(b[i] * dr)
        if els is None:
            continue
        g = math.sqrt(max((b[i] * dr) ** 2 - Z40_H ** 2, 0.01)); Rc = cov[i]
        for az in azg:
            p = float(np.mean([np.real(np.vdot(a, Rc @ a))
                               for a in (arr.steering_vector(az, math.radians(e)) for e in els)]))
            aw = math.radians(AZ_CAL_K * math.degrees(az) + AZ_CAL_OFF)
            ix = int(round((g * math.sin(aw) - xs[0]) / cell))
            iy = int(round((g * math.cos(aw) - ys[0]) / cell))
            if 0 <= ix < len(xs) and 0 <= iy < len(ys):
                G[iy, ix] += p
    return xs, ys, G


def cube_free_gate(points_xyz, floor_at=None, margin=0.45, min_pts=8):
    """THE cube-free fall gate (see memory fall-cube-free-gate): a suspected lying/lost trigger
    when a body's 2nd-highest point sits <= `margin` above the TRUE-LOCAL-FLOOR (NOT naive z=0).
    Mirrors falldet/window.py's validated on-chip window logic. ONLY when this triggers do we
    fire detect_fall_z40 to CONFIRM a real floor body — so a normal sit/stand (points well above
    the local floor -> no trigger -> no cube) is filtered HERE, upstream of the cube.

    points_xyz : iterable of world (x, y, z) points.
    floor_at   : fn(x, y) -> local floor world-z (falldet/window.py FloorMap.at); default 0.0.
                 Pass the true-local-floor (H_g or the server sloped -0.5*rad(elev_acc)*R line).
    Returns {trigger, top_h, ground_range} — ground_range = where to aim the cubeQuery."""
    P = np.asarray(list(points_xyz), float)
    if len(P) < min_pts:
        return {"trigger": False, "top_h": None, "ground_range": None}
    fa = floor_at or (lambda x, y: 0.0)
    rel = np.sort(np.array([P[i, 2] - fa(P[i, 0], P[i, 1]) for i in range(len(P))]))[::-1]
    top_h = float(rel[1])                                  # 2nd-highest above local floor (robust)
    trig = top_h <= margin
    gr = float(np.median(np.hypot(P[:, 0], P[:, 1]))) if trig else None
    return {"trigger": bool(trig), "top_h": round(top_h, 2),
            "ground_range": round(gr, 2) if gr is not None else None}


def confirm_fall(live_npz, empty_npz=_EMPTY, **kw):
    """Two-stage fall = cube_free_gate (upstream, on live point cloud) THEN detect_fall_z40
    (this, the cube CONFIRM). This wrapper is the CONFIRM half; call it only after
    cube_free_gate(...)['trigger'] is True on the live cloud. Kept split so the cube never
    runs unless the point-cloud-Z gate fired (the whole point of fall-cube-free-gate)."""
    return detect_fall_z40(live_npz, empty_npz, **kw)


if __name__ == "__main__":
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    CASES = [("test1_20260721", "躺切2.6", 2.6), ("test2_20260721", "躺切5.2", 5.2),
             ("test3_20260721", "躺径", 3.8), ("test5_20260721", "坐", 4.15),
             ("standR_20260721", "站", 4.45), ("empty_20260721", "空(对照)", None)]
    THR = 1.0
    print(f"差值/基值 occupancy detector — thr={THR} (2x signal), near-gate {NEAR_GATE}m, "
          f"background=empty_20260721 (FIXED install)\n")
    print(f"{'case':14}{'blocks':>7}   detail (peak_ratio @range, span, nbins, shadow, coarse-x)")
    print("-" * 100)
    fig, axes = plt.subplots(2, 3, figsize=(16, 8))
    for ax, (c, lbl, true_r) in zip(axes.flat, CASES):
        npz = os.path.join(_HERE, "..", "case", f"{c}.npz")
        r = detect_occupancy(npz, thr=THR)
        rng, ratio, blocks = r["range_m"], r["ratio"], r["blocks"]
        conf = [b for b in blocks if b["confirmed"]]
        det = "  ".join(f"{'✓BODY' if b['confirmed'] else 'x ghost'}[{b['peak_ratio']:+.1f}"
                        f"@{b['peak_range']:.2f}m sh{b['shadow']:+.1f}]" for b in blocks)
        print(f"{c:14}{len(conf):>7}   {det or '(none)'}")
        ax.axhline(THR, color="#e74c3c", ls="--", lw=1, label=f"thr {THR} (2x)")
        ax.axhline(0, color="#888", lw=0.6)
        ax.plot(rng, ratio, "-o", ms=2, color="#2c3e50")
        ax.axvspan(0, NEAR_GATE, color="gray", alpha=0.15)
        for b in blocks:
            ax.axvspan(b["r0"] - 0.05, b["r1"] + 0.05, color="#f39c12", alpha=0.35)
        if true_r:
            ax.axvline(true_r, color="#27ae60", ls=":", lw=1.2, label=f"true {true_r}m")
        ax.set_title(f"{c}  ({lbl}) — {len(blocks)} block(s)", fontsize=9)
        ax.set_xlabel("range (m)"); ax.set_ylabel("差值/基值")
        ax.set_ylim(-1.5, max(3.5, ratio.max() * 1.1)); ax.legend(fontsize=7); ax.grid(alpha=0.25)
    fig.suptitle("Occupancy by 差值/基值 = (live-empty)/empty vs FIXED background "
                 "(orange=detected block, green=true body range)", fontsize=11)
    fig.tight_layout()
    out = os.path.join(_HERE, "occupancy_ratio_20260722.png")
    fig.savefig(out, dpi=115); print(f"\nsaved {out}")
