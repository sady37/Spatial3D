"""Per-track ghost-vs-body discriminators on the top-5 highest points -- observation only.

GOAL (user 2026-07-23): reject a METAL / metal-reflection GHOST that reached the fall path. A strong
metal reflector, lit up when a person walks past, spawns a track: a DENSE, concentrated knot of
points at one facet, moving COHERENTLY (rigid -- the whole return shifts together, or sits still). A
real body is the opposite: its top-5 highest points are SPREAD over the body's extent and move
INCOHERENTLY -- breathing, sway, limbs all micro-move in different radial directions at once.

So on the top-5 highest points, BOTH of these are LARGE for a person and SMALL for a ghost:
  * dispersion  -- 3D spatial spread of the top-5 (body extent vs a point-reflection knot)
  * doppler_std -- spread of per-point radial velocity (incoherent body motion vs rigid/still ghost)
  * bipolar     -- fraction of the min(+,-) doppler sign: a body has points approaching AND receding
                   at once (non-rigid); a rigid object/ghost is single-sign (or zero)

NOTE (user 2026-07-23): the earlier worry that sigma_pos misses a STANDING-STILL person is moot --
GTRACK's CFAR keeps a standing person tracked (no lost), and the tier-1 6 s persistence filters a
flicker, so a still person never reaches the 3001/cube path spuriously. This probe's only job is
GHOST rejection on tracks that DID reach it. Runs entirely server-side on the 3001 cloud + track the
server already receives every frame -- no firmware, works with old or new firmware.

Observation only: accumulate and log. A gate waits until the person/ghost distributions are visible
in real data (same observe-first discipline as the baseline-variance work). sigma_pos/zspread kept
as auxiliaries.
"""
from __future__ import annotations
from collections import deque
import math

WIN = 8                 # frames of history (matches the MLP's 8-frame window)
TOPK = 5                # top-5 highest points (matches TI's MLP feature)
_hist = {}              # tid -> deque of per-frame (top5 positions, top5 doppler)


def update(tid, pts):
    """pts = list of (x, y, z, doppler) for THIS track this frame. Returns a stats dict or None if
    not enough history yet."""
    if not pts:
        return None
    top = sorted(pts, key=lambda p: p[2], reverse=True)[:TOPK]     # by height z
    pos = [(p[0], p[1], p[2]) for p in top]
    dop = [p[3] for p in top]
    h = _hist.setdefault(tid, deque(maxlen=WIN))
    h.append((pos, dop))
    if len(h) < WIN:
        return None

    # ── dispersion: 3D spatial spread of the top-5 WITHIN a frame, averaged over the window.
    #    person = spread across the body; ghost = a concentrated knot -> small.
    disp = _mean([[_spread3d(p)] for p, _ in h], 0)

    # ── doppler_std: spread of the top-5 per-point radial velocity, over the whole window.
    #    person = incoherent micro-motion -> wide; ghost = rigid/still -> narrow.
    all_dop = [d for _, dd in h for d in dd]
    dstd = _std(all_dop) if len(all_dop) >= 3 else 0.0

    # ── bipolar: how much the doppler carries BOTH signs at once (non-rigid body) vs one sign
    #    (rigid object). fraction = min(#pos, #neg) / total, ignoring near-zero. person high, ghost ~0.
    nz = [d for d in all_dop if abs(d) > 0.02]
    if nz:
        npos = sum(1 for d in nz if d > 0)
        bipolar = min(npos, len(nz) - npos) / len(nz)
    else:
        bipolar = 0.0

    # ── auxiliaries (kept for cross-reference): sigma_pos = temporal wander of the top-5 centroid;
    #    zspread = mean per-frame height spread.
    cents = [(_mean(p, 0), _mean(p, 1), _mean(p, 2)) for p, _ in h]
    cx = _mean(cents, 0); cy = _mean(cents, 1); cz = _mean(cents, 2)
    sigma_pos = math.sqrt(sum((c[0] - cx) ** 2 + (c[1] - cy) ** 2 + (c[2] - cz) ** 2
                              for c in cents) / len(cents))
    zspread = _mean([[max(z for _, _, z in p) - min(z for _, _, z in p)] for p, _ in h], 0)

    return {"dispersion": round(disp, 3), "doppler_std": round(dstd, 3),
            "bipolar": round(bipolar, 2), "sigma_pos": round(sigma_pos, 3),
            "zspread": round(zspread, 3), "n_hist": len(h)}


def prune(alive_tids):
    for t in [t for t in _hist if t not in alive_tids]:
        _hist.pop(t, None)


def drop(tid):
    _hist.pop(tid, None)


def _spread3d(pts):
    """RMS distance of the points from their centroid (3D)."""
    if len(pts) < 2:
        return 0.0
    cx = _mean(pts, 0); cy = _mean(pts, 1); cz = _mean(pts, 2)
    return math.sqrt(sum((x - cx) ** 2 + (y - cy) ** 2 + (z - cz) ** 2
                         for x, y, z in pts) / len(pts))


def _mean(rows, i):
    return sum(r[i] for r in rows) / len(rows) if rows else 0.0


def _std(v):
    m = sum(v) / len(v)
    return math.sqrt(sum((x - m) ** 2 for x in v) / len(v))
