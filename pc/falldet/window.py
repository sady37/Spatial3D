"""Module 1 — SUSTAINED-WINDOW max-height fall trigger (the one that ports to the DSP).

Track-independent: works on the person's associated point HEIGHTS, not the track posZ
(which freezes/ghosts during a fall). A fallen body's highest point collapses to the
floor; a seated person's head stays ~1 m up.

Floor is POSITION-DEPENDENT (tilt makes the apparent floor slope with range, and x/y in
general), so we do NOT use a scalar floor. We reference every point to the LOCAL floor:

    rel_z_i = z_i - H_g(x_i, y_i)          # height above the floor AT that point
    H_s     = 2nd-highest rel_z            # robust top (drops one ghost-point spike)
    down    = H_s <= margin, held K frames # even the top of the body sits at the floor

Robust to: ghost spike (2nd-highest), one noisy seated frame (K-frame sustain), and a
sloping/mis-tilt-corrected floor (per-position H_g). Validated on TI's 5-person set.
Streaming, firmware-shaped: current frame + a tiny counter. O(n_points) per frame.
"""


class FloorMap:
    """H_g(x, y) from a quiet capture — the floor world-Z at each ground cell. Binned by
    range (tilt-dominant axis) and optionally x. Missing cells fall back to the median."""
    def __init__(self, cell=0.5):
        self.cell = cell
        self.hg = {}          # (ix, iy) -> floor z
        self.default = 0.0

    def _key(self, x, y):
        return (int(round(x / self.cell)), int(round(y / self.cell)))

    def fit(self, pts_xyz, pct=5.0):
        import numpy as np
        by = {}
        for x, y, z in pts_xyz:
            by.setdefault(self._key(x, y), []).append(z)
        self.hg = {k: float(np.percentile(v, pct)) for k, v in by.items() if len(v) >= 8}
        self.default = float(np.median(list(self.hg.values()))) if self.hg else 0.0
        return self

    def at(self, x, y):
        return self.hg.get(self._key(x, y), self.default)


class WindowDetector:
    def __init__(self, floor_map, margin=0.45, sustain=5, clear=5):
        """floor_map : FloorMap (or any object with .at(x,y)); a body is 'down' when its
        2nd-highest point sits <= margin above the LOCAL floor, held `sustain` frames."""
        self.fm = floor_map
        self.margin = margin
        self.sustain = sustain
        self.clear = clear
        self.low_run = 0
        self.hi_run = 0
        self.down = False

    def update(self, points_xyz):
        """points_xyz : the person's associated points THIS frame, each (x, y, z).
        Returns dict(down, h_s, low_run)  where h_s is the 2nd-highest height-above-floor."""
        h_s = None
        if points_xyz is not None and len(points_xyz) >= 2:
            rel = sorted((z - self.fm.at(x, y) for (x, y, z) in points_xyz), reverse=True)
            h_s = rel[1]                          # 2nd-highest above-floor height (robust)
        low = (h_s is not None) and (h_s <= self.margin)
        if low:
            self.low_run += 1; self.hi_run = 0
            if self.low_run >= self.sustain:
                self.down = True
        else:
            self.hi_run += 1; self.low_run = 0
            if self.hi_run >= self.clear:
                self.down = False
        return {"down": self.down, "h_s": h_s, "low_run": self.low_run}
