"""PC reference implementation of the ON-CHIP fall trigger — track-independent,
point-cloud max-height. Mirrors exactly what the firmware should compute each frame
from the MINOR point cloud, so the C port has fixed, validated parameters.

Idea (validated on case 20260716): a fallen body's HIGHEST point collapses to the
floor, while a seated person's head stays ~1.1 m. So PER PERSON (a ground-plane cluster
of the minor cloud — NOT a fixed range cell, which would split a body and false-fire on
a feet-only cell):

    H_s = 2nd-highest world-Z over the WHOLE cluster   (2nd = drop 1 ghost spike)
    low  = H_s <= H_g(range) + H_d
    trigger BURST when `low` holds for CONFIRM_S (aging/debounce), keyed by cluster location

No tracker, no track-Z (which freezes/ghosts during a fall — see case notes). The
firmware already has the minor cloud (micro-motion => a LIVING person, clutter removed),
so this is O(n_points) per frame and cheap on-chip.

    .venv/bin/python3 fall_trigger_ref.py record/live_scene_XXXX.npz
"""
import sys, math
import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

# ---- geometry (from sensorPosition) ------------------------------------------------
MOUNT, TILT = 2.0, 25.0          # sensor height (m), down-tilt (deg)

# ---- trigger parameters (this is what the C port copies) ---------------------------
CELL_M    = 0.5                  # range-cell width (m); a body spans a few cells
N_MIN     = 6                    # min minor-cloud points in a cell to count as a body
H_G       = 0.0                  # calibrated floor world-height (per-cell table refines this)
H_D       = 0.40                 # floor margin (m): the top point must sit within H_G+H_D
CONFIRM_S = 0.6                  # `low` must hold this long before BURST (aging/debounce)
CLEAR_S   = 0.5                  # `low` absent this long -> reset the cell (person got up/left)


def world_yz(y, z, tilt=TILT, mount=MOUNT):
    th = math.radians(tilt)
    wy = y * math.cos(th) + z * math.sin(th)          # world ground range
    wz = mount + z * math.cos(th) - y * math.sin(th)  # world height
    return wy, wz


class FallTrigger:
    """Streaming, firmware-shaped: current frame + tiny per-cell state only."""
    def __init__(self):
        self.low_since = {}   # cell -> time `low` began (0 = not currently low)
        self.last_low = {}    # cell -> last time it was low (for CLEAR aging)
        self.armed = {}       # cell -> currently bursting

    def update(self, t, minor_xyz):
        """minor_xyz: (N,3) minor-cloud points this frame, sensor frame.
        Returns list of (range_m, event, H_s) events this frame."""
        events = []
        # bin points into range cells, keep world-Z
        cells = {}
        for x, y, z in minor_xyz:
            wy, wz = world_yz(y, z)
            cells.setdefault(int(wy / CELL_M), []).append(wz)
        # evaluate every occupied cell
        for c, zs in cells.items():
            if len(zs) < N_MIN:
                continue
            zs.sort(reverse=True)
            H_s = zs[1] if len(zs) >= 2 else zs[0]        # 2nd-highest = robust top (drop 1 ghost)
            if H_s <= H_G + H_D:                          # body's top is at the floor -> low
                self.last_low[c] = t
                if self.low_since.get(c, 0) == 0:
                    self.low_since[c] = t
                if (t - self.low_since[c]) >= CONFIRM_S and not self.armed.get(c):
                    self.armed[c] = True
                    events.append((round(c * CELL_M + CELL_M / 2, 1), "BURST", round(H_s, 2)))
            else:                                         # top is high -> upright, this cell not low
                self.low_since[c] = 0
                if self.armed.get(c):
                    self.armed[c] = False
                    events.append((round(c * CELL_M + CELL_M / 2, 1), "STOP", round(H_s, 2)))
        # age out cells that lost their low points (person moved away / got up)
        for c in list(self.low_since):
            if t - self.last_low.get(c, 0) > CLEAR_S:
                self.low_since[c] = 0
                if self.armed.get(c):
                    self.armed[c] = False
                    events.append((round(c * CELL_M + CELL_M / 2, 1), "STOP", None))
        return events


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    d = np.load(sys.argv[1], allow_pickle=True)
    ts = d["ts"].astype(float); t0 = ts[0]
    pf, pxyz = d["p_frame"], d["pc_xyz"]
    print(f"# {sys.argv[1].split('/')[-1]}: {len(ts)} frames, {ts[-1]-t0:.0f}s")
    print(f"# params: cell={CELL_M}m N_min={N_MIN} H_g+H_d={H_G+H_D}m confirm={CONFIRM_S}s")
    trig = FallTrigger()
    bursting = False
    for fi in range(len(ts)):
        t = ts[fi] - t0
        sub = pxyz[pf == fi]
        evs = trig.update(t, sub)
        for rng, ev, hs in evs:
            print(f"  {t:6.1f}s  {ev:5s} @ D={rng}m  H_s={hs}")
            if ev == "BURST":
                bursting = True
    # summary: total BURST-active time
    print(f"# {'FALL DETECTED (cube would burst)' if bursting else 'no fall trigger'}")


if __name__ == "__main__":
    main()
