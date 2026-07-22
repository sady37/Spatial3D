"""Build the static-scene deliverable + #5 occlusion baseline from a per-bin
covariance capture (e.g. case/static_empty_20260721.npz).

What this sensor+data can actually deliver (validated against the room photo):
  - RANGE of every strong reflector (reliable; dr ~10.6cm)
  - SIDE of each object (LEFT/CENTER/RIGHT) via azimuth left-right antisymmetry
    -- validated: left partition -> LEFT @1.5m, boxes -> RIGHT @4.3m
  - a per-bin glint/surface/void label (range-domain trimmed classifier)
NOT deliverable here: fine azimuth (29deg beam), elevation/z (rank-1),
side-wall geometry (specular-away), a clean floor reflectivity curve
(furnished room too cluttered -- needs a bare-room sweep).

Outputs (into spatial3d/):
  scene_static_20260721.json / .svg / scene_static_A_preview.png   (deliverable A)
  static_baseline_20260721.npz                                     (#5 baseline)

NOTE (2026-07-21 consolidation): scene_layers.py is the single orchestrating
pipeline; it imports the calibrated primitives + detectors here (real_array,
to_ground, detect_coarse_to_fine, merge_boxes, per_bin_features, build_xy_grid)
AND calls the renderers below so one run regenerates BOTH the layered model and
these scene_static_* deliverables -- consistent, nothing deleted. Running this
module directly still works (legacy entry).
"""
from __future__ import annotations
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from .music import AntennaArray, LAMBDA, awrl6844_array

HERE = __file__.rsplit("/", 1)[0]

# TI AWRL6844 REAL virtual-array layout (antGeometryCfg), not the idealised
# row-major 4x4 UPA in music.awrl6844_array (which has a TODO to fix). Correct
# element ordering halves bin-to-bin azimuth jitter (26.9deg -> 16.1deg),
# making adjacent-bin bearings coherent -> far fewer spurious boxes.
_AZ_IDX = np.array([0, 1, 1, 0, 0, 1, 1, 0, 2, 3, 3, 2, 2, 3, 3, 2], float)
_EL_IDX = np.array([2, 2, 3, 3, 0, 0, 1, 1, 0, 0, 1, 1, 2, 2, 3, 3], float)


def real_array():
    hl = LAMBDA / 2.0
    return AntennaArray(_AZ_IDX * hl, _EL_IDX * hl, LAMBDA)
AZG = np.arange(-50, 50.001, 1.0)
AZ = np.deg2rad(AZG)
NEAR_GATE = 0.9         # m, drop DC/coupling near-field blob
H_MOUNT = 2.0
TILT_DEG = 25.0
# The radar is NOT a top-down camera: it hangs at H_MOUNT and pitches DOWN
# TILT_DEG, so a range bin is a SLANT distance along the tilted line-of-sight,
# not a floor depth. To get a true top-down X/Y we must de-project slant->ground:
# a target at slant range r whose centroid sits OBJ_H above the floor is at
# HORIZONTAL ground range G = sqrt(r^2 - (H_MOUNT - OBJ_H)^2). Azimuth (lateral)
# is a rotation about the horizontal axis and is unaffected by the down-pitch, so
# the fitted bearing cal applies directly to it. Elevation is rank-collapsed ->
# per-object height unknown; OBJ_H is one compromise (desk ~0.75, bed ~0.4,
# chair ~0.5, dresser ~0.8). Without this, near objects are pushed too far in.
OBJ_H = 0.8
H_EFF = H_MOUNT - OBJ_H         # 1.2 m radar->object-centroid vertical drop
# Radar azimuth-positive maps to world LEFT (validated against the room photo:
# without this the scene comes out left-right mirrored). World x = -r*sin(az).
X_SIGN = -1.0


def ground_range(r, h_eff=H_EFF):
    """Slant range -> horizontal ground range (fixes near-far foreshortening)."""
    return np.sqrt(np.maximum(np.asarray(r)**2 - h_eff**2, 0.09))


# Azimuth calibration, fit from two seated-person track anchors (2026-07-21):
# az = AZ_CAL_K * raw_az + AZ_CAL_OFF (deg), then x = r*sin(az), y = r*cos(az).
# DISPLAY FRAME = the USER's / photo view (chairR on the RIGHT, +x). The firmware
# track frame is its MIRROR (firmware x=-0.18 was the user's chairR); we keep the
# track MAGNITUDES but flip the sign so the map reads like reality. OFF~+18deg is
# the array boresight bias (the real cause of the lateral error, not scale).
# Range ~= ground range for these targets so slant r is used directly as depth.
# 3-ANCHOR fit (chairR +3.2, chairL -9.6, standR +23.7 deg true; standR is the
# wide-angle @wall anchor): the 2-anchor line failed badly at wide azimuth
# (predicted +43 vs true +24 -> slope drops 1.13->0.58 = sin-compression), so
# refit least-squares over 3 -> RMS position error 0.135m across the anchors.
# LEFT wide angle still only anchored to -9.6deg; far-left is extrapolated.
AZ_CAL_K = 0.684
AZ_CAL_OFF = 9.21


def to_ground(r, az, h_eff=H_EFF):
    """Radar tilted-polar (SLANT range, calibrated azimuth) -> top-down world
    (X, Y) on the floor plane. De-projects slant range to HORIZONTAL ground range
    G = sqrt(r^2 - h_eff^2) (radar-to-object vertical drop), because the radar
    pitches down from H_MOUNT and a range bin is a slant distance, not a depth.
    Azimuth is unaffected by the down-pitch so the fitted bearing cal applies."""
    aw = np.radians(AZ_CAL_K * np.degrees(np.asarray(az, float)) + AZ_CAL_OFF)
    g = np.sqrt(np.maximum(np.asarray(r, float)**2 - h_eff**2, 0.09))
    return g * np.sin(aw), g * np.cos(aw)


def per_bin_features(bins, cov, dr):
    """Return a dict of per-bin arrays: power/label/symmetry/side/dominant."""
    arr = real_array()
    rng = bins * dr
    n = len(bins)
    trace = np.array([np.real(np.trace(cov[i])) for i in range(n)])
    sym_level = np.zeros(n); antisym = np.zeros(n); side = np.zeros(n)
    dom_az = np.zeros(n); peak = np.zeros(n)
    for i in range(n):
        R = cov[i]
        P = np.array([np.real(a.conj() @ R @ a) for a in
                      (arr.steering_vector(az, 0.0) for az in AZ)])
        Pf = P[::-1]
        sym_level[i] = 0.5 * (P + Pf).mean()
        antisym[i] = np.sum(np.abs(P - Pf)) / (np.sum(np.abs(P)) + 1e-30)
        side[i] = X_SIGN * np.sum(AZG * P) / (np.sum(P) + 1e-30)
        k = int(np.argmax(P)); dom_az[i] = AZG[k]; peak[i] = P[k]

    pdb = 10 * np.log10(trace / trace.max() + 1e-12)
    # range-domain trimmed classifier (glint / surface / void) on CFAR residual
    keep = rng >= NEAR_GATE
    p = pdb.copy()
    trend = np.array([np.median(p[max(0, i-7):i+8]) for i in range(n)])
    resid = p - trend
    hi = np.percentile(resid[keep], 80); lo = np.percentile(resid[keep], 30)
    label = np.where(resid >= hi, "glint",
                     np.where(resid <= lo, "void", "surface")).astype(object)
    label[~keep] = "nearfield"
    return dict(bins=bins, range_m=rng, power_db=pdb, trend_db=trend,
                resid_db=resid, label=label, sym_level=sym_level,
                antisym_frac=antisym, side_deg=side, dom_az_deg=dom_az,
                peak=peak, keep=keep)


def side_bucket(deg):
    return "LEFT" if deg < -6 else ("RIGHT" if deg > 6 else "CENTER")


def build_xy_grid(bins, cov, dr, cell=0.15, xlim=3.5, ylim=7.0):
    """X/Y occupancy grids (the representation the scene actually lives in;
    a range bin is an arc, not a point). Returns xs, ys, raw, obj where:
      raw = each bin's az power spread over its arc (floor never empty)
      obj = ANTISYMMETRIC part only -> symmetric floor cancels, objects
            localize to their side, open floor reads empty.
    """
    arr = real_array()
    rng = bins * dr
    xs = np.arange(-xlim, xlim + cell, cell)
    ys = np.arange(0.0, ylim + cell, cell)
    raw = np.zeros((len(ys), len(xs))); obj = np.zeros((len(ys), len(xs)))
    for i in range(len(bins)):
        r = rng[i]
        if r < NEAR_GATE:
            continue
        R = cov[i]
        P = np.array([np.real(a.conj() @ R @ a) for a in
                      (arr.steering_vector(az, 0.0) for az in AZ)])
        P = P / (np.real(np.trace(R)) + 1e-9)
        Pa = np.maximum(0.0, P - P[::-1])
        gx, gy = to_ground(r, AZ)                # proper de-tilt (H, tilt)
        ix = np.round((gx - xs[0]) / cell).astype(int)
        iy = np.round((gy - ys[0]) / cell).astype(int)
        ok = (ix >= 0) & (ix < len(xs)) & (iy >= 0) & (iy < len(ys))
        for j in np.where(ok)[0]:
            raw[iy[j], ix[j]] += P[j]; obj[iy[j], ix[j]] += Pa[j]
    return xs, ys, raw, obj


# ---- scene model: 100% data-driven (no guessed/photo objects) -------------
def detect_front_wall(f):
    """Front wall depth = the farthest strong coherent range peak (data)."""
    rng = f["range_m"]; pdb = f["power_db"]
    band = (rng >= 4.0) & (rng <= 6.8)
    idx = np.arange(len(rng))[band]
    i = idx[np.argmax(pdb[band])]
    return round(float(rng[i]), 2)


def build_scene(f, round1, round2, front_wall):
    """Fully data-driven scene: front wall (range peak) + code-detected object
    boxes. NO hand-placed / photo-guessed objects -- everything below comes
    from detect_coarse_to_fine on the radar covariances.
    """
    def box_obj(b, i, level):
        cx, cy = b["center"]
        return {"id": f"{level}-{i+1}", "confidence": "radar", "level": level,
                "center": b["center"], "size": b["size"],
                "range_m": round(float(np.hypot(cx, cy)), 2),
                "side": side_bucket(np.degrees(np.arctan2(cx, max(cy, 0.1))) ),
                "energy": b["energy"]}
    scene = {
        "frame": {"origin": "radar", "units": "m", "x": "lateral,+right",
                  "y": "depth,+into-room", "radar_height_m": 2.0, "tilt_deg": 25.0,
                  "note": "rotate whole group to world; radar stays (0,0)"},
        "radar": {"pos": [0.0, 0.0], "look_deg": 0.0, "fov_deg": [-60, 60]},
        "walls": [
            {"id": "front", "confidence": "high", "source": "radar",
             "pts": [[-3.0, front_wall], [3.0, front_wall]],
             "note": f"depth radar-confirmed {front_wall}m (range peak); trim width"},
        ],
        "objects_round1": [box_obj(b, i, "R1") for i, b in enumerate(round1)],
        "objects_round2": [box_obj(b, i, "R2") for i, b in enumerate(round2)],
        "notes": {"floor": "empty plane = walkable; not drawn as object",
                  "side_walls": "radar cannot see side walls (specular-away) -> user draws",
                  "doors": "user draws", "z": "dropped (rank-1); boxes are X/Y footprints",
                  "method": "coarse-to-fine 70%-energy (15-85% marginal) boxes on "
                            "antisym-object X/Y map; peak-bearing per bin"},
        "todo_user": ["set front-wall width", "draw side/back walls",
                      "draw door(s)", "rotate group to world frame"],
    }
    # objects list for the renderers (round2 refined = the working detail level)
    scene["objects"] = scene["objects_round2"] or scene["objects_round1"]
    return scene


# ---------- SVG ----------
def write_svg(scene, path):
    SCALE = 90.0; X0, X1, Y0, Y1 = -3.6, 3.6, -0.8, 7.2
    W = (X1-X0)*SCALE; H = (Y1-Y0)*SCALE
    def px(x, y): return (x-X0)*SCALE, (y-Y0)*SCALE   # radar top, depth ↓
    def line(p0, p1, **kw):
        x0, y0 = px(*p0); x1, y1 = px(*p1)
        a = " ".join(f'{k.replace("_","-")}="{v}"' for k, v in kw.items())
        return f'<line x1="{x0:.1f}" y1="{y0:.1f}" x2="{x1:.1f}" y2="{y1:.1f}" {a}/>'
    s = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W:.0f}" height="{H:.0f}" '
         f'viewBox="0 0 {W:.0f} {H:.0f}" font-family="sans-serif">',
         f'<rect width="{W:.0f}" height="{H:.0f}" fill="#0d1117"/>']
    for gx in range(-3, 4):
        x, _ = px(gx, 0)
        s.append(f'<line x1="{x:.1f}" y1="0" x2="{x:.1f}" y2="{H:.0f}" stroke="#243040"/>')
        s.append(f'<text x="{x+2:.1f}" y="{H-4:.0f}" fill="#4a5a6a" font-size="10">{gx}m</text>')
    for gy in range(0, 8):
        _, y = px(0, gy)
        s.append(f'<line x1="0" y1="{y:.1f}" x2="{W:.0f}" y2="{y:.1f}" stroke="#243040"/>')
        s.append(f'<text x="3" y="{y-3:.1f}" fill="#4a5a6a" font-size="10">{gy}m</text>')
    # radar + fov
    s.append('<g id="radar">')
    for a in scene["radar"]["fov_deg"]:
        s.append(line((0, 0), (7*np.sin(np.deg2rad(a)), 7*np.cos(np.deg2rad(a))),
                      stroke="#c0392b", stroke_width=1, stroke_dasharray="4 4", opacity=0.5))
    rx, ry = px(0, 0)
    s.append(f'<polygon points="{rx:.1f},{ry+9:.1f} {rx-8:.1f},{ry-6:.1f} {rx+8:.1f},{ry-6:.1f}" fill="#e74c3c"/>')
    s.append(f'<text x="{rx+12:.1f}" y="{ry+6:.1f}" fill="#e74c3c" font-size="12">Radar (0,0) H2.0 tilt25</text></g>')
    # walls
    s.append('<g id="walls">')
    for w in scene["walls"]:
        if w["confidence"] == "high":
            s.append(line(w["pts"][0], w["pts"][1], stroke="#2ecc71", stroke_width=4))
        else:
            s.append(line(w["pts"][0], w["pts"][1], stroke="#7f8c9a", stroke_width=2, stroke_dasharray="8 6"))
    lx, ly = px(-3.0, 6.25)
    s.append(f'<text x="{lx:.1f}" y="{ly-6:.1f}" fill="#2ecc71" font-size="12">front wall 6.25m (confirmed depth; trim width)</text></g>')
    # objects
    ocol = {"radar": "#f1c40f", "med": "#3498db", "photo": "#5d6d7e"}
    s.append('<g id="objects">')
    for o in scene["objects"]:
        cx, cy = o["center"]; sw, sh = o["size"]; col = ocol[o["confidence"]]
        x, y = px(cx-sw/2, cy-sh/2)
        dash = "0" if o["confidence"] == "radar" else "4 3"
        s.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{sw*SCALE:.1f}" height="{sh*SCALE:.1f}" '
                 f'fill="none" stroke="{col}" stroke-width="1.6" stroke-dasharray="{dash}"/>')
        tag = f'{o["id"]} [{o.get("side","")} {o["range_m"]}m]' if o["confidence"] == "radar" else o["id"]
        s.append(f'<text x="{x:.1f}" y="{y-3:.1f}" fill="{col}" font-size="10">{tag}</text>')
    s.append('</g>')
    dx, dy = px(1.6, 4.9)
    s.append(f'<text x="{dx+6:.1f}" y="{dy:.1f}" fill="#e67e22" font-size="11">&#9660; draw door(s) here</text>')
    s.append('</svg>')
    with open(path, "w") as fh:
        fh.write("\n".join(s))


def write_preview(scene, path):
    ocol = {"radar": "#f1c40f", "med": "#3498db", "photo": "#5d6d7e"}
    fig, ax = plt.subplots(figsize=(8, 9))
    ax.set_facecolor("#0d1117"); fig.patch.set_facecolor("#0d1117")
    for a in scene["radar"]["fov_deg"]:
        ax.plot([0, 7*np.sin(np.deg2rad(a))], [0, 7*np.cos(np.deg2rad(a))],
                color="#c0392b", ls=":", lw=1, alpha=0.5)
    ax.plot(0, 0, "v", color="#e74c3c", ms=15)
    ax.text(0.25, 0.3, "Radar (0,0)", color="#e74c3c", fontsize=9)
    for w in scene["walls"]:
        (x0, y0), (x1, y1) = w["pts"]
        if w["confidence"] == "high":
            ax.plot([x0, x1], [y0, y1], color="#2ecc71", lw=3)
        else:
            ax.plot([x0, x1], [y0, y1], color="#7f8c9a", lw=1.6, ls="--")
    ax.text(-3.0, 6.35, "front wall 6.25m (confirmed)", color="#2ecc71", fontsize=9)
    for o in scene["objects"]:
        cx, cy = o["center"]; sw, sh = o["size"]; col = ocol[o["confidence"]]
        ls = "-" if o["confidence"] == "radar" else ":"
        ax.add_patch(Rectangle((cx-sw/2, cy-sh/2), sw, sh, fill=False, ec=col, lw=1.6, ls=ls))
        tag = f'{o["id"]}\n[{o.get("side","")} {o["range_m"]}m]' if o["confidence"] == "radar" else o["id"]
        ax.text(cx, cy+sh/2+0.05, tag, color=col, fontsize=7.5, ha="center")
    ax.text(1.65, 4.9, "◀ draw door(s)", color="#e67e22", fontsize=9)
    ax.set_xlim(-3.6, 3.6); ax.set_ylim(7.2, -0.8); ax.set_aspect("equal")  # radar top
    ax.set_xlabel("x lateral (m)", color="#aaa"); ax.set_ylabel("y depth (m) ↓ into room", color="#aaa")
    ax.tick_params(colors="#666"); ax.grid(alpha=0.15)
    ax.set_title("Static scene (100% data-driven, no guesses)\n"
                 "yellow=radar-detected 70%-energy boxes  green=front wall  orange=you draw",
                 color="#ddd", fontsize=10)
    fig.tight_layout(); fig.savefig(path, dpi=115, facecolor="#0d1117")


def detect_object_boxes(bins, cov, dr, cell=0.15, xlim=3.5, ylim=7.0,
                        energy_frac=0.70, pct=80):
    """Visual-detection-style object boxes on the X/Y plane.

    Poor azimuth is embraced, not fought: place each bin's antisymmetric
    (object) energy at its PEAK bearing only (one point per bin -> arcs
    collapse to compact blobs), rasterise, cluster, and draw the tightest box
    holding `energy_frac` of each cluster's energy -- like boxing a person in
    vision. Box width = the resolution uncertainty. Returns (grid, xs, ys, boxes).
    """
    from scipy import ndimage
    arr = real_array(); rng = bins * dr
    xs = np.arange(-xlim, xlim + cell, cell); ys = np.arange(0.0, ylim + cell, cell)
    grid = np.zeros((len(ys), len(xs)))
    for i in range(len(bins)):
        r = rng[i]
        if r < NEAR_GATE:
            continue
        R = cov[i]
        P = np.array([np.real(a.conj() @ R @ a) for a in
                      (arr.steering_vector(az, 0.0) for az in AZ)])
        P = P / (np.real(np.trace(R)) + 1e-9)
        Pa = np.maximum(0.0, P - P[::-1])
        if Pa.max() <= 0:
            continue
        th = AZ[int(np.argmax(Pa))]              # object bearing for this bin
        gx, gy = to_ground(r, th)
        ix = int(round((gx - xs[0]) / cell))
        iy = int(round((gy - ys[0]) / cell))
        if 0 <= ix < len(xs) and 0 <= iy < len(ys):
            grid[iy, ix] += float(Pa.sum())
    grid = ndimage.gaussian_filter(grid, 0.8)
    thr = np.percentile(grid[grid > 0], pct)
    lab, n = ndimage.label(ndimage.binary_dilation(grid >= thr))
    tail = (1.0 - energy_frac) / 2.0             # trim this frac off each side
    boxes = []
    for k in range(1, n + 1):
        cells = np.argwhere(lab == k); e = grid[cells[:, 0], cells[:, 1]]
        if e.sum() < 0.02 * grid.sum() or len(cells) < 2:
            continue
        # scan from both edges inward to the central energy_frac (70%): trim
        # `tail` off each side of the marginal cumulative energy along x and y.
        def band(axis_idx):
            coords = cells[:, axis_idx]
            lo_i, hi_i = coords.min(), coords.max()
            prof = np.array([e[coords == c].sum() for c in range(lo_i, hi_i + 1)])
            cdf = np.cumsum(prof) / prof.sum()
            lo = lo_i + int(np.searchsorted(cdf, tail))
            hi = lo_i + int(np.searchsorted(cdf, 1.0 - tail))
            return lo, max(hi, lo)
        y0, y1 = band(0); x0, x1 = band(1)
        cx = float(xs[(x0+x1)//2]); cy = float(ys[(y0+y1)//2])
        boxes.append(dict(center=[round(cx, 2), round(cy, 2)],
                          size=[round(float(xs[x1]-xs[x0]+cell), 2),
                                round(float(ys[y1]-ys[y0]+cell), 2)],
                          x0=float(xs[x0]), y0=float(ys[y0]),
                          energy=round(float(e.sum()), 2),
                          side=side_bucket(cx / max(cy, 0.1) * 57.3)))
    boxes.sort(key=lambda b: -b["energy"])
    return grid, xs, ys, boxes


def learn_floor_baseline(bins, cov, dr, trim=(20, 60), margin_db=6.0):
    """Self-learn the floor reflectivity curve from the OPEN FLOOR (user: the
    room has a large empty carpet, so this IS learnable now).

    NOT left-right symmetry (fails: objects on BOTH sides leak in) and NOT
    elevation/z (rank-collapsed, verified). Instead: at each range the arc is
    MOSTLY open floor with a few bright object spikes, so the ANGULAR
    TRIMMED-MEAN of the (per-bin power-normalised) az spectrum = diffuse floor,
    the peak above it = object. Iteratively robust-fit floor_dB vs log-range,
    dropping object-excess bins. Residual ~1.1 dB (vs ~4.1 for the symmetry
    method) = a clean self-built floor baseline.

    Returns range_m, floor_db (measured), floor_fit (curve), peak_db, dom_az_deg,
    object_bins (peak > floor_fit + margin_db), slope, resid_std.
    """
    arr = real_array(); rng = bins * dr; keep = rng >= NEAR_GATE
    getR = (lambda i: cov[int(bins[i])]) if isinstance(cov, dict) else (lambda i: cov[i])
    fov = (AZG >= -50) & (AZG <= 50)
    floor = np.full(len(bins), np.nan); peak = np.full(len(bins), np.nan)
    dom = np.zeros(len(bins))
    for i in range(len(bins)):
        R = getR(i)
        P = np.array([np.real(a.conj() @ R @ a) for a in
                      (arr.steering_vector(az, 0.0) for az in AZ)])
        P = P / (np.real(np.trace(R)) + 1e-9)
        lo, hi = np.percentile(P[fov], trim[0]), np.percentile(P[fov], trim[1])
        band = P[fov][(P[fov] >= lo) & (P[fov] <= hi)]      # trimmed = floor
        floor[i] = 10 * np.log10(band.mean() + 1e-12)
        k = int(np.argmax(P)); peak[i] = 10 * np.log10(P[k] + 1e-12); dom[i] = AZG[k]
    m = keep & np.isfinite(floor); fit = np.zeros(len(bins)); slope = 0.0
    for _ in range(3):                                       # robust re-fit
        A = np.column_stack([np.log10(rng[m]), np.ones(m.sum())])
        c, *_ = np.linalg.lstsq(A, floor[m], rcond=None); slope = float(c[0])
        fit = c[0] * np.log10(rng) + c[1]
        m = keep & (np.abs(floor - fit) < 2.0)
    obj = keep & ((peak - fit) > margin_db)
    return dict(range_m=rng, floor_db=floor, floor_fit=fit, peak_db=peak,
                dom_az_deg=dom, object_bins=obj, slope=slope,
                resid_std=float((floor - fit)[m].std()))


def detect_floor_relative(bins, cov, dr, margin_db=6.0, half_frac=0.5, merge_gap=0.4):
    """Per-bin object detection thresholded against the SELF-LEARNED FLOOR
    baseline (learn_floor_baseline), not each bin's own median. An object bin =
    az-peak power rises margin_db above the floor curve at that range; this
    catches STRAIGHT-AHEAD objects too (antisym only found off-centre ones).
    Box lateral = r * half-power width (near-small/far-large); radial = dr; then
    adjacent-bin detections merge. Returns boxes {center,size,x0,y0,energy,r}."""
    arr = real_array(); rng = bins * dr
    getR = (lambda i: cov[int(bins[i])]) if isinstance(cov, dict) else (lambda i: cov[i])
    fb = learn_floor_baseline(bins, cov, dr, margin_db=margin_db)
    dets = []
    for i in np.where(fb["object_bins"])[0]:
        r = rng[i]; R = getR(i)
        P = np.array([np.real(a.conj() @ R @ a) for a in
                      (arr.steering_vector(az, 0.0) for az in AZ)])
        P = P / (np.real(np.trace(R)) + 1e-9)
        # width from the OBJECT-EXCESS over the floor curve (not the full beam):
        # near/strong -> broad excess -> big box; far/weak -> narrow -> small box.
        floor_lin = 10 ** (fb["floor_fit"][i] / 10.0)
        exc = np.maximum(P - floor_lin, 0.0)
        k = int(np.argmax(exc)); pk = exc[k]
        half = pk * half_frac
        lo = k
        while lo > 0 and exc[lo-1] >= half:
            lo -= 1
        hi = k
        while hi < len(exc)-1 and exc[hi+1] >= half:
            hi += 1
        width_rad = np.deg2rad(max(AZG[hi] - AZG[lo], 1.0))
        cx, cy = to_ground(r, AZ[k])
        lateral = min(max(float(r * width_rad), 0.2), 1.2)   # cap runaway width
        dets.append(dict(center=[round(float(cx), 2), round(float(cy), 2)],
                         size=[round(lateral, 2), round(float(dr), 2)],
                         x0=float(cx - lateral/2), y0=float(cy - dr/2),
                         energy=round(float(fb["peak_db"][i] - fb["floor_fit"][i]), 2),
                         r=round(float(r), 2)))
    dets = merge_boxes(dets, gap=merge_gap)
    dets.sort(key=lambda d: -d["energy"])
    return dets, fb


def detect_per_bin(bins, cov, dr, thr_db=6.0, half_frac=0.5, merge_gap=0.4):
    """Per-BIN object detection — each range bin judged on ITS OWN terms, never a
    unified near/far standard (user, 2026-07-21).

    Why per-bin: a range bin is an arc at fixed range; far returns are weaker
    (~1/r^4) and one 29-deg beam covers a LATERAL span of r*dtheta -- small near,
    large far. A single global threshold + fixed-metre box (as in
    detect_coarse_to_fine) favours strong near objects and mis-sizes everything.

    Per bin i (r >= NEAR_GATE):
      - antisym (one-sided/object) az spectrum Pa, normalised by this bin's power
      - detect the peak bearing ONLY if it rises thr_db above THIS bin's own
        antisym floor (median of its positive part) -> a weak far bin competes
        against itself, not against strong near bins
      - lateral box width = r * (half-power angular width)  => near-small/far-large
      - radial depth = dr
    Adjacent-bin detections of one object are then merged (cross-bin combine, not
    a cross-bin threshold). Returns boxes {center,size,x0,y0,energy,r}.
    """
    arr = real_array(); rng = bins * dr
    dets = []
    for i in range(len(bins)):
        r = rng[i]
        if r < NEAR_GATE:
            continue
        R = cov[i]
        P = np.array([np.real(a.conj() @ R @ a) for a in
                      (arr.steering_vector(az, 0.0) for az in AZ)])
        Pa = np.maximum(0.0, P - P[::-1]) / (np.real(np.trace(R)) + 1e-9)
        pos = Pa[Pa > 0]
        if len(pos) < 2:
            continue
        floor = np.median(pos)                       # THIS bin's own az floor
        peak = Pa.max()
        if floor <= 0 or 10 * np.log10(peak / floor + 1e-12) < thr_db:
            continue                                 # no object above own floor
        k = int(np.argmax(Pa))
        half = peak * half_frac                       # half-power angular width
        lo = k
        while lo > 0 and Pa[lo-1] >= half:
            lo -= 1
        hi = k
        while hi < len(Pa)-1 and Pa[hi+1] >= half:
            hi += 1
        width_rad = np.deg2rad(max(AZG[hi] - AZG[lo], 1.0))
        cx, cy = to_ground(r, AZ[k])
        lateral = max(float(r * width_rad), 0.15)     # near-small / far-large
        dets.append(dict(center=[round(float(cx), 2), round(float(cy), 2)],
                         size=[round(lateral, 2), round(float(dr), 2)],
                         x0=float(cx - lateral/2), y0=float(cy - dr/2),
                         energy=round(float(Pa.sum()), 3), r=round(float(r), 2)))
    dets = merge_boxes(dets, gap=merge_gap)
    dets.sort(key=lambda d: -d["energy"])
    return dets


def detect_coarse_to_fine(bins, cov, dr, cell=0.15, energy_frac=0.70):
    """Coarse-to-fine: Round-1 big blocks, then refine each into Round-2 boxes.

    Round 0 (implicit): floor = the empty plane, walls = front-wall line + side
    placeholders. Round 1: heavy smoothing + low threshold -> a few BIG blocks.
    Round 2: within each big block, light smoothing + higher threshold -> finer
    sub-boxes. Both rounds use the same 70%-energy (15-85% marginal) box rule.
    """
    from scipy import ndimage
    arr = real_array(); rng = bins * dr
    xs = np.arange(-3.5, 3.5+cell, cell); ys = np.arange(0.0, 7.0+cell, cell)
    pts = np.zeros((len(ys), len(xs)))
    for i in range(len(bins)):
        r = rng[i]
        if r < NEAR_GATE:
            continue
        R = cov[i]
        P = np.array([np.real(a.conj()@R@a) for a in
                      (arr.steering_vector(az, 0.0) for az in AZ)])
        P = P/(np.real(np.trace(R))+1e-9); Pa = np.maximum(0.0, P-P[::-1])
        if Pa.max() <= 0:
            continue
        th = AZ[int(np.argmax(Pa))]; gx, gy = to_ground(r, th)
        ix = int(round((gx-xs[0])/cell)); iy = int(round((gy-ys[0])/cell))
        if 0 <= ix < len(xs) and 0 <= iy < len(ys):
            pts[iy, ix] += float(Pa.sum())

    tail = (1.0-energy_frac)/2.0
    def box_of(cells, e):
        def band(ax):
            c = cells[:, ax]; lo_i, hi_i = c.min(), c.max()
            prof = np.array([e[c == k].sum() for k in range(lo_i, hi_i+1)])
            cdf = np.cumsum(prof)/prof.sum()
            return lo_i+int(np.searchsorted(cdf, tail)), lo_i+int(np.searchsorted(cdf, 1-tail))
        y0, y1 = band(0); x0, x1 = band(1)
        return dict(center=[round(float(xs[(x0+x1)//2]), 2), round(float(ys[(y0+y1)//2]), 2)],
                    size=[round(float(xs[x1]-xs[x0]+cell), 2), round(float(ys[max(y1,y0)]-ys[y0]+cell), 2)],
                    x0=float(xs[x0]), y0=float(ys[y0]), energy=round(float(e.sum()), 2))

    def clusters(grid, sigma, pct):
        g = ndimage.gaussian_filter(grid, sigma)
        lab, n = ndimage.label(ndimage.binary_dilation(g >= np.percentile(g[g > 0], pct)))
        out = []
        for k in range(1, n+1):
            cells = np.argwhere(lab == k); e = g[cells[:, 0], cells[:, 1]]
            if e.sum() >= 0.03*g.sum() and len(cells) >= 2:
                out.append((cells, e))
        return out

    round1 = [box_of(c, e) for c, e in clusters(pts, 1.6, 65)]
    round1.sort(key=lambda b: -b["energy"])

    # Round 2: LOCAL edge-converging boxes. Seed on local peaks; take a 1.5x
    # local window (background->center); converge from left/right and top/bottom
    # to where the marginal energy exceeds `frac` of the local peak.
    g2 = ndimage.gaussian_filter(pts, 0.7)
    peaks = (g2 == ndimage.maximum_filter(g2, size=5)) & \
            (g2 > np.percentile(g2[g2 > 0], 85))
    NOM = 3               # nominal object half-size (cells ~0.45m); window = 1.5x
    WIN = int(round(NOM * 1.5))

    def local_box(sy, sx, frac=0.30):
        y0 = max(0, sy-WIN); y1 = min(g2.shape[0], sy+WIN+1)
        x0 = max(0, sx-WIN); x1 = min(g2.shape[1], sx+WIN+1)
        W = g2[y0:y1, x0:x1]
        rp = W.sum(1); cp = W.sum(0)
        ri = np.where(rp >= frac*rp.max())[0]; ci = np.where(cp >= frac*cp.max())[0]
        return y0+ri[0], y0+ri[-1], x0+ci[0], x0+ci[-1]

    round2, seen = [], set()
    for sy, sx in sorted(np.argwhere(peaks), key=lambda p: -g2[p[0], p[1]]):
        yy0, yy1, xx0, xx1 = local_box(sy, sx)
        key = ((xx0+xx1)//2, (yy0+yy1)//2)
        if key in seen:
            continue
        seen.add(key)
        e = g2[yy0:yy1+1, xx0:xx1+1]
        round2.append(dict(
            center=[round(float(xs[(xx0+xx1)//2]), 2), round(float(ys[(yy0+yy1)//2]), 2)],
            size=[round(float(xs[xx1]-xs[xx0]+cell), 2), round(float(ys[yy1]-ys[yy0]+cell), 2)],
            x0=float(xs[xx0]), y0=float(ys[yy0]), energy=round(float(e.sum()), 2)))
    round2 = merge_boxes(round2, gap=0.4)     # fuse adjacent-bin detections
    round2.sort(key=lambda b: -b["energy"])
    return xs, ys, pts, round1, round2


def edge_from_discontinuity(bins, cov, dr, min_disc=0.3):
    """Find EDGES / FOLDS / CORNERS as spatial DISCONTINUITIES (user: a 棱线 is a
    跃变 and a jump is always findable, immune to the coarse absolute azimuth).

    A wall FACE varies smoothly range-to-range; at a fold/edge/corner the whole
    azimuth-spectrum SHAPE changes abruptly. So disc[i] = 1 - correlation of
    adjacent-range az-spectra; local maxima above min_disc = edges. Using the
    spectrum SHAPE (not the noisy argmax peak) rejects multipath ridge-hopping.
    Validated: standR's outward wall-fold (4.45m) ranks among the top disc.

    Returns range_m, disc, and edges = list of (range, x, y, disc, power_db).
    """
    arr = real_array(); rng = bins * dr
    getR = (lambda i: cov[int(bins[i])]) if isinstance(cov, dict) else (lambda i: cov[i])
    Paz = []
    for i in range(len(bins)):
        P = np.array([np.real(a.conj() @ getR(i) @ a) for a in
                      (arr.steering_vector(az, 0.0) for az in AZ)])
        Paz.append(P / (np.linalg.norm(P) + 1e-12))
    Paz = np.array(Paz)
    disc = np.zeros(len(bins))
    for i in range(1, len(bins)):
        disc[i] = 1.0 - float(np.dot(Paz[i], Paz[i-1]))
    edges = []
    for i in range(1, len(bins)-1):
        if rng[i] < NEAR_GATE:
            continue
        if disc[i] >= min_disc and disc[i] >= disc[i-1] and disc[i] >= disc[i+1]:
            k = int(np.argmax(Paz[i])); x, y = to_ground(rng[i], AZ[k])
            pdb = 10*np.log10(np.real(np.trace(getR(i))) + 1e-9)
            edges.append((round(float(rng[i]), 2), round(float(x), 2),
                          round(float(y), 2), round(float(disc[i]), 2), round(float(pdb), 1)))
    edges.sort(key=lambda e: -e[3])
    return dict(range_m=rng, disc=disc, edges=edges)


def merge_boxes(boxes, gap=0.4):
    """Fuse boxes of the same object split across adjacent range bins. Two boxes
    merge if their footprints overlap (within `gap` m on both axes); the merged
    box unions the extent and its center is the ENERGY-WEIGHTED centroid (the
    strength-weighted angle estimate: A on bin R2 + B on bin R3 -> one bearing).
    """
    B = [dict(b) for b in boxes]
    def bounds(b):
        return b["x0"], b["y0"], b["x0"]+b["size"][0], b["y0"]+b["size"][1]
    changed = True
    while changed:
        changed = False
        for i in range(len(B)):
            for j in range(i+1, len(B)):
                ax0, ay0, ax1, ay1 = bounds(B[i]); bx0, by0, bx1, by1 = bounds(B[j])
                if (ax0-gap <= bx1 and bx0-gap <= ax1 and
                        ay0-gap <= by1 and by0-gap <= ay1):
                    ea, eb = B[i]["energy"], B[j]["energy"]; e = ea+eb
                    cx = (B[i]["center"][0]*ea + B[j]["center"][0]*eb)/e
                    cy = (B[i]["center"][1]*ea + B[j]["center"][1]*eb)/e
                    nx0, ny0 = min(ax0, bx0), min(ay0, by0)
                    nx1, ny1 = max(ax1, bx1), max(ay1, by1)
                    B[i] = dict(center=[round(cx, 2), round(cy, 2)],
                                size=[round(nx1-nx0, 2), round(ny1-ny0, 2)],
                                x0=nx0, y0=ny0, energy=round(e, 2))
                    B.pop(j); changed = True; break
            if changed:
                break
    return B


def _render_c2f(xs, ys, grid, round1, round2, path):
    from matplotlib.patches import Rectangle
    def db(a): return 10*np.log10(a/(a.max()+1e-12)+1e-4)
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.imshow(db(grid), origin="lower", extent=[xs[0], xs[-1], ys[0], ys[-1]],
              aspect="equal", cmap="inferno", vmin=-25, vmax=0)
    ax.plot(0, 0, "cv", ms=14); ax.axhline(6.25, color="#2ecc71", ls="--", lw=2)
    ax.text(-3.3, 6.15, "wall 6.25m (面)", color="#2ecc71", fontsize=10)
    # Round1 big blocks intentionally NOT drawn: they enclose open floor between
    # objects (user: "R1-1 not needed, too much empty space"). Keep only R2 boxes.
    for b in round2:                      # refined objects
        ax.add_patch(Rectangle((b["x0"], b["y0"]), b["size"][0], b["size"][1],
                               fill=False, ec="cyan", lw=1.2, ls="--"))
    ax.set_xlabel("x lateral (m)"); ax.set_ylabel("y depth (m) ↓ into room")
    ax.set_xlim(xs[0], xs[-1]); ax.set_ylim(ys[-1], 0)   # radar top, room down
    ax.set_title("coarse→fine: orange=Round1 big blocks, cyan dash=Round2 refined\n"
                 "floor=empty plane, front wall=面")
    fig.tight_layout(); fig.savefig(path, dpi=110); plt.close(fig)


def _render_boxes(grid, xs, ys, boxes, path):
    from matplotlib.patches import Rectangle
    def db(a): return 10*np.log10(a/(a.max()+1e-12)+1e-4)
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.imshow(db(grid), origin="lower", extent=[xs[0], xs[-1], ys[0], ys[-1]],
              aspect="equal", cmap="inferno", vmin=-25, vmax=0)
    ax.plot(0, 0, "c^", ms=13); ax.axhline(6.25, color="#2ecc71", ls="--", lw=2)
    ax.text(-3.3, 6.32, "front wall 6.25m (面)", color="#2ecc71", fontsize=10)
    for i, b in enumerate(boxes):
        ax.add_patch(Rectangle((b["x0"], b["y0"]), b["size"][0], b["size"][1],
                               fill=False, ec="cyan", lw=2))
        ax.text(b["center"][0], b["y0"]+b["size"][1]+0.06,
                f'obj{i+1}\n{b["size"][0]}x{b["size"][1]}', color="cyan",
                fontsize=8, ha="center")
    ax.set_xlabel("x lateral (m)"); ax.set_ylabel("y depth (m)")
    ax.set_xlim(xs[0], xs[-1]); ax.set_ylim(0, ys[-1])
    ax.set_title("static objects: 70%-energy boxes (arcs removed)\n"
                 "floor=empty, front wall=面, boxes=static objects")
    fig.tight_layout(); fig.savefig(path, dpi=110); plt.close(fig)


def _render_xy(xs, ys, raw, obj, scene, path):
    def db(a):
        return 10 * np.log10(a / (a.max() + 1e-12) + 1e-4)
    ext = [xs[0], xs[-1], ys[0], ys[-1]]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(15, 8))
    a1.imshow(db(raw), origin="lower", extent=ext, aspect="equal", cmap="inferno", vmin=-25, vmax=0)
    a1.set_title("RAW energy X/Y (arcs smear → floor never empty)")
    im = a2.imshow(db(obj), origin="lower", extent=ext, aspect="equal", cmap="inferno", vmin=-25, vmax=0)
    a2.set_title("OBJECT energy (antisym) X/Y (floor cancels → open space shows)")
    for ax in (a1, a2):
        ax.plot(0, 0, "cv", ms=13); ax.axhline(6.25, color="#2ecc71", ls="--", lw=1)
        ax.set_xlabel("x lateral (m)"); ax.set_ylabel("y depth (m) ↓ into room")
        ax.set_xlim(xs[0], xs[-1]); ax.set_ylim(ys[-1], 0)   # radar top, room down
    for o in scene["objects"]:
        if o["confidence"] == "radar":
            a2.annotate(f'{o["id"]}\n[{o["side"]}]', o["center"], color="cyan", fontsize=8,
                        bbox=dict(boxstyle="round", fc="black", alpha=0.4))
    fig.colorbar(im, ax=a2, shrink=0.6, label="dB")
    fig.tight_layout(); fig.savefig(path, dpi=105); plt.close(fig)


def main():
    import sys
    npz = sys.argv[1] if len(sys.argv) > 1 else f"{HERE}/../case/empty_20260721.npz"
    print(f"baseline: {npz}")
    d = np.load(npz, allow_pickle=True)
    bins = d["bins"].astype(int); cov = d["covariances"]; dr = float(d["dr_m"])
    f = per_bin_features(bins, cov, dr)

    # 100% data-driven: coarse->fine boxes + front-wall range peak. No guesses.
    front = detect_front_wall(f)
    xs, ys, cgrid, round1, round2 = detect_coarse_to_fine(bins, cov, dr)
    scene = build_scene(f, round1, round2, front)

    with open(f"{HERE}/scene_static_20260721.json", "w") as fh:
        json.dump(scene, fh, indent=2)
    write_svg(scene, f"{HERE}/scene_static_20260721.svg")
    write_preview(scene, f"{HERE}/scene_static_A_preview.png")
    _render_c2f(xs, ys, cgrid, round1, round2, f"{HERE}/scene_static_c2f.png")

    xr, yr, raw, obj = build_xy_grid(bins, cov, dr)
    _render_xy(xr, yr, raw, obj, scene, f"{HERE}/scene_static_xy_map.png")

    np.savez(f"{HERE}/static_baseline_20260721.npz",
             bins=f["bins"], range_m=f["range_m"],
             power_db=f["power_db"], trend_db=f["trend_db"], resid_db=f["resid_db"],
             label=f["label"].astype(str),
             sym_level=f["sym_level"], antisym_frac=f["antisym_frac"],
             side_deg=f["side_deg"], dom_az_deg=f["dom_az_deg"],
             xy_xs=xr, xy_ys=yr, xy_raw=raw, xy_obj=obj, xy_cell=0.15,
             front_wall_m=front, mount_m=2.0, tilt_deg=25.0, dr_m=dr)

    print("wrote scene json/svg, A_preview, c2f, xy_map, baseline npz")
    print(f"front wall (data): {front}m")
    print(f"Round1 big blocks ({len(round1)}) — all code-detected, no guesses:")
    for o in scene["objects_round1"]:
        print(f"  {o['id']} {o['side']:6s} @{o['range_m']}m center={o['center']} size={o['size']}")
    print(f"Round2 refined ({len(round2)}):")
    for o in scene["objects_round2"]:
        print(f"  {o['id']} {o['side']:6s} @{o['range_m']}m center={o['center']} size={o['size']}")


if __name__ == "__main__":
    main()
