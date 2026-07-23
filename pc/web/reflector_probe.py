"""Per-track top-5 point STABILITY probe -- observation only, no gating.

GOAL (user 2026-07-23): separate a METAL-reflector ghost from a real person. A metal object
(corner reflector, table leg, door frame) is FIXED in position and geometry; when someone walks
past, they act as a secondary illuminator and swell the metal bin's power over the detection
threshold, so the tracker spawns a track ON the metal. The trap is POWER: the metal's return
fluctuates (looks alive), but its GEOMETRY is dead -- the top-5 highest points come from the same
metal facet every frame.

DISCRIMINATOR = the TEMPORAL variance of the top-5 points' POSITION, not their power:
    metal ghost : sigma_pos ~ 0   (fixed facet)   , sigma_snr large (modulated by passers-by)
    real person : sigma_pos large (body deforms, breathes, sways), sigma_snr large
So low sigma_pos + high sigma_snr = metal. This is the user's "the 5 points barely change" made
quantitative. The same sigma_pos also feeds the person-roster certification (a real person's cloud
is dynamic; a split fragment / ghost is degenerate), so it is computed once and used twice.

This module ONLY accumulates and reports. Gating waits until the two distributions are visible in
real data -- same observe-first discipline as the baseline-variance work.
"""
from __future__ import annotations
from collections import deque
import math

WIN = 8                 # frames of history (matches the MLP's 8-frame window)
TOPK = 5                # top-5 highest points (matches TI's MLP feature)
_hist = {}              # tid -> deque of per-frame (topk positions, topk snr)


def update(tid, pts):
    """pts = list of (x, y, z, snr) for THIS track this frame (snr may be None). Returns a dict of
    stability stats or None if not enough history yet."""
    if not pts:
        return None
    top = sorted(pts, key=lambda p: p[2], reverse=True)[:TOPK]     # by height z
    pos = [(p[0], p[1], p[2]) for p in top]
    snr = [p[3] for p in top if p[3] is not None]
    h = _hist.setdefault(tid, deque(maxlen=WIN))
    h.append((pos, snr))
    if len(h) < WIN:
        return None
    # sigma_pos: how much the top-5 CENTROID wanders frame-to-frame (a metal facet -> ~0). Using the
    # centroid (not per-point matching) avoids the top-5 ordering churn while still capturing "does
    # this geometry sit still". A real deforming body moves its high-point centroid; a fixed facet
    # does not.
    cents = [(_mean(p, 0), _mean(p, 1), _mean(p, 2)) for p, _ in h]
    cx = _mean(cents, 0); cy = _mean(cents, 1); cz = _mean(cents, 2)
    sigma_pos = math.sqrt(sum((c[0] - cx) ** 2 + (c[1] - cy) ** 2 + (c[2] - cz) ** 2
                              for c in cents) / len(cents))
    # sigma of the top-5 HEIGHT spread within a frame, averaged -> a person's cloud is tall/variable,
    # a facet is a tight knot. Complements sigma_pos (temporal) with a spatial spread term.
    zspread = _mean([[max(z for _, _, z in p) - min(z for _, _, z in p)] for p, _ in h], 0)
    # sigma_snr: power fluctuation (both metal and person vary, so this is the "is it modulated"
    # term, not a discriminator on its own -- logged so the metal signature low-pos/high-snr shows).
    allsnr = [s for _, ss in h for s in ss]
    sigma_snr = _std(allsnr) if len(allsnr) >= 3 else None
    return {"sigma_pos": round(sigma_pos, 3), "zspread": round(zspread, 3),
            "sigma_snr": (round(sigma_snr, 1) if sigma_snr is not None else None),
            "n_hist": len(h)}


def drop(tid):
    _hist.pop(tid, None)


def prune(alive_tids):
    for t in [t for t in _hist if t not in alive_tids]:
        _hist.pop(t, None)


def _mean(rows, i):
    return sum(r[i] for r in rows) / len(rows) if rows else 0.0


def _std(v):
    m = sum(v) / len(v)
    return math.sqrt(sum((x - m) ** 2 for x in v) / len(v))
