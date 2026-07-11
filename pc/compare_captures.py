"""Compare captures across 3 representations, global vs chair-region.

Reproduces the drift/detection tables in 空房基线测试-0708.md (§3.2, §8):
  - R      : per-bin covariance Frobenius relative diff, median over bins (data domain)
  - 点云   : MUSIC point-cloud voxel-count grid relative diff
  - 能量   : MUSIC normalised energy-density grid relative diff
each computed 全局 (all bins / voxels) and Chair (range bins 131-151, or their
room voxels via the point's bin column).

Captures are loaded by short name from the standard session files:
  A B C D E  <- reset_ab.npz (cov_A.. / mean_A..)
  G          <- seg_G.npz  (sit_detect empty base)  [== sit_detect cov_base]
  H          <- seg_H.npz  (sit_detect sit)          [== sit_detect cov_sit]
  I          <- cal_I.npz  (cube -> covariances)
Falls back to seg_<name>.npz if present.

    python compare_captures.py                      # R-only table (fast)
    python compare_captures.py --music              # + point-cloud + energy (slow)
    python compare_captures.py --pairs A-B,A-G,G-H,A-I,H-I
"""
import argparse

import numpy as np

CHAIR = range(131, 152)          # bins 3.07-3.54m
ALLB = range(87, 271)
DR = 0.0234375


# Optional {short-name: npz-path} override, set from --map. Lets the same table
# be run over arbitrary captures (e.g. this session's emptyJ / evK) without
# touching the hardcoded historical mapping below.
OVERRIDE: dict[str, str] = {}


def load_cov(name):
    """Return {bin: 16x16 cov} for a short capture name."""
    import os
    if name in OVERRIDE:
        d = np.load(OVERRIDE[name], allow_pickle=True)
        bins = d["bins"].astype(int)
        key = "cov" if "cov" in d else "covariances"
        return {int(b): d[key][i] for i, b in enumerate(bins)}
    if name in "ABCDE":
        d = np.load("reset_ab.npz", allow_pickle=True)
        bins = d["bins"].astype(int)
        return {int(b): d[f"cov_{name}"][i] for i, b in enumerate(bins)}
    if name == "I" and os.path.exists("cal_I.npz"):
        from spatial3d.cube import Cube
        return Cube.load("cal_I.npz").covariances()
    f = f"seg_{name}.npz"
    d = np.load(f, allow_pickle=True)
    bins = d["bins"].astype(int)
    key = "cov" if "cov" in d else "covariances"
    return {int(b): d[key][i] for i, b in enumerate(bins)}


def rel_cov(Ci, Cj, bins):
    bs = [b for b in bins if b in Ci and b in Cj]
    return float(np.median([np.linalg.norm(Ci[b] - Cj[b]) /
                            (np.linalg.norm(Ci[b]) + 1e-9) for b in bs]))


# --- MUSIC point-cloud / energy (optional, slow) ---------------------------
XR, YR, ZR, VS = (-3, 3), (0, 7), (-2.5, 2.5), 0.25


def music_cloud(covs):
    from spatial3d.music import awrl6844_array
    from spatial3d.range_music import covariances_to_points
    from spatial3d.music_collect import to_room
    array = awrl6844_array()
    pts = covariances_to_points(covs, array, method="music",
                                az_range=(-45, 45), el_range=(-45, 30),
                                resolution_deg=3, max_peaks_per_bin=2)
    return to_room(pts)


def grid(cloud, chair=False, energy=False):
    nx = int((XR[1] - XR[0]) / VS); ny = int((YR[1] - YR[0]) / VS)
    nz = int((ZR[1] - ZR[0]) / VS)
    c = cloud[(cloud[:, 4] >= 131) & (cloud[:, 4] <= 151)] if chair else cloud
    g = np.zeros((nx, ny, nz))
    if len(c) == 0:
        return g
    ix = np.clip(((c[:, 0] - XR[0]) / VS).astype(int), 0, nx - 1)
    iy = np.clip(((c[:, 1] - YR[0]) / VS).astype(int), 0, ny - 1)
    iz = np.clip(((c[:, 2] - ZR[0]) / VS).astype(int), 0, nz - 1)
    w = c[:, 3] if energy else np.ones(len(c))
    np.add.at(g, (ix, iy, iz), w)
    return g / g.sum() if (energy and g.sum() > 0) else g


def rel_grid(gi, gj):
    return float(np.linalg.norm(gi - gj) / (np.linalg.norm(gi) + 1e-9))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pairs", default="A-B,A-C,A-D,A-E,A-G,G-H,A-H,A-I,H-I")
    ap.add_argument("--music", action="store_true",
                    help="also compute MUSIC point-cloud + energy columns (slow)")
    ap.add_argument("--map", default="",
                    help="override capture files, e.g. A=empty_A.npz,J=emptyJ.npz")
    a = ap.parse_args()
    if a.map:
        OVERRIDE.update(dict(kv.split("=") for kv in a.map.split(",")))
    pairs = [tuple(p.split("-")) for p in a.pairs.split(",")]
    names = sorted({n for p in pairs for n in p})

    covs = {n: load_cov(n) for n in names}
    clouds = {n: music_cloud(covs[n]) for n in names} if a.music else {}

    if a.music:
        print(f"{'vs':>5}|{'R全':>6}|{'R-Ch':>6}|{'点云全':>7}|{'点云Ch':>7}|{'能量全':>7}|{'能量Ch':>7}")
    else:
        print(f"{'vs':>5}|{'R全局':>7}|{'R-Chair':>8}")
    for i, j in pairs:
        r1 = rel_cov(covs[i], covs[j], ALLB); r2 = rel_cov(covs[i], covs[j], CHAIR)
        if a.music:
            pc1 = rel_grid(grid(clouds[i]), grid(clouds[j]))
            pc2 = rel_grid(grid(clouds[i], chair=True), grid(clouds[j], chair=True))
            e1 = rel_grid(grid(clouds[i], energy=True), grid(clouds[j], energy=True))
            e2 = rel_grid(grid(clouds[i], True, True), grid(clouds[j], True, True))
            print(f"{i}-{j:>2}|{r1:>6.3f}|{r2:>6.3f}|{pc1:>7.3f}|{pc2:>7.3f}|{e1:>7.3f}|{e2:>7.3f}")
        else:
            print(f"{i}-{j:>2}|{r1:>7.3f}|{r2:>8.3f}")


if __name__ == "__main__":
    main()
