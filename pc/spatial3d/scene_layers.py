"""Layered static-scene pipeline (coarse->fine), built one puzzle piece at a time.

L0 background screen  : floor + walls (range-primary, most reliable axis)
L1 region layer       : residual-vs-background -> object/anomaly boxes
L2 fine layer         : per-box accumulate + CFAR corners + MUSIC

Reuses the CALIBRATED primitives from build_static_scene (real TI array manifold,
to_ground az calibration az=0.684*raw+9.21, AZ grid). Each piece is a standalone
function + a figure so we can confirm before adding the next.
"""
from __future__ import annotations
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .build_static_scene import (real_array, to_ground, AZ, AZG, NEAR_GATE,
                                 AZ_CAL_K, AZ_CAL_OFF, detect_coarse_to_fine,
                                 detect_per_bin, detect_floor_relative,
                                 learn_floor_baseline, edge_from_discontinuity,
                                 per_bin_features, build_xy_grid)

HERE = __file__.rsplit("/", 1)[0]
# World-forward direction in RAW array-azimuth (az_world = K*raw+OFF = 0):
RAW_FWD = -AZ_CAL_OFF / AZ_CAL_K                 # ~ -13.5 deg
_ARR = real_array()


def _spectrum(R):
    return np.array([np.real(a.conj() @ R @ a) for a in
                     (_ARR.steering_vector(az, 0.0) for az in AZ)])


def load(npz):
    d = np.load(npz, allow_pickle=True)
    bins = d["bins"].astype(int)
    cov = {int(b): d["covariances"][i] for i, b in enumerate(bins)}
    return bins, cov, float(d["dr_m"])


# ============================ PIECE 1 ============================
def piece1_range_profile(bins, cov, dr, cfar_win=7, cfar_db=3.0):
    """Radial intensity profile: power=trace(R) vs range. Gate the near-field
    DC/coupling blob, estimate a local (CFAR) noise floor, and return the range
    bins that hold a real reflector (power >= local floor + cfar_db).

    Returns dict: range_m, power_db, floor_db, reflector_bins, reflector_ranges.
    """
    rng = bins * dr
    power = np.array([np.real(np.trace(cov[int(b)])) for b in bins])
    pdb = 10 * np.log10(power / power.max() + 1e-12)
    keep = rng >= NEAR_GATE                      # drop DC/coupling blob
    # CFAR-ish local floor = local median (excluding the cell), reflector if above
    floor = np.full(len(bins), -99.0)
    for i in range(len(bins)):
        lo, hi = max(0, i - cfar_win), min(len(bins), i + cfar_win + 1)
        neigh = np.concatenate([pdb[lo:i], pdb[i+1:hi]])
        floor[i] = np.median(neigh) if len(neigh) else pdb[i]
    is_refl = keep & (pdb >= floor + cfar_db)
    # keep only local maxima among CFAR hits (one bin per reflector)
    refl = []
    for i in np.where(is_refl)[0]:
        if pdb[i] >= pdb[max(0, i-1)] and pdb[i] >= pdb[min(len(bins)-1, i+1)]:
            refl.append(i)
    return dict(range_m=rng, power_db=pdb, floor_db=floor, keep=keep,
                reflector_bins=bins[refl], reflector_ranges=rng[refl],
                reflector_idx=np.array(refl))


def _render_piece1(bins, dr, p1, path):
    rng = p1["range_m"]; pdb = p1["power_db"]
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(rng, pdb, "-o", ms=3, color="#888", label="power trace(R)")
    ax.plot(rng[p1["keep"]], (p1["floor_db"] + 3.0)[p1["keep"]], "--",
            color="#3498db", lw=1, label="CFAR floor +3dB")
    ri = p1["reflector_idx"]
    ax.scatter(rng[ri], pdb[ri], c="#e74c3c", s=80, zorder=5,
               label=f"reflectors ({len(ri)})")
    for i in ri:
        ax.annotate(f"{rng[i]:.2f}m", (rng[i], pdb[i]), (rng[i], pdb[i] + 1.5),
                    fontsize=7, color="#c0392b", ha="center")
    ax.axvspan(0, NEAR_GATE, color="gray", alpha=0.2, label="near-field gated")
    ax.set_xlabel("range (m)"); ax.set_ylabel("power (dB rel peak)")
    ax.set_title("PIECE 1 — radial intensity profile + CFAR reflector detection")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(path, dpi=115); plt.close(fig)


# ============================ PIECE 2 ============================
def piece2_floor_reflectivity(bins, cov, dr):
    """Same-range left-right symmetry about the WORLD-forward axis: the matched
    symmetric part = base-surface (floor) diffuse reflectivity; the one-sided
    excess = object. Returns per-bin floor_level (dB) + object_level (dB).

    P_sym(theta) = min(P(theta), P(mirror theta)) with mirror about RAW_FWD;
    floor_level = median over the symmetric-assessable angles; object_level =
    peak of the one-sided excess P - P_sym.
    """
    rng = bins * dr
    floor = np.full(len(bins), -99.0); obj = np.full(len(bins), -99.0)
    mirror_deg = 2 * RAW_FWD - AZG                # mirror angle for each grid pt
    valid = (mirror_deg >= AZG[0]) & (mirror_deg <= AZG[-1])
    for i, b in enumerate(bins):
        if rng[i] < NEAR_GATE:
            continue
        P = _spectrum(cov[int(b)])
        Pmir = np.interp(mirror_deg, AZG, P)      # power at mirrored angle
        Psym = np.minimum(P, Pmir)                # matched (floor) part
        excess = P - Psym                         # one-sided (object) part
        tr = np.real(np.trace(cov[int(b)]))
        floor[i] = 10*np.log10(np.median(Psym[valid]) / tr + 1e-12)
        obj[i] = 10*np.log10(excess.max() / tr + 1e-12)
    return dict(range_m=rng, floor_db=floor, object_db=obj, keep=rng >= NEAR_GATE)


def _render_piece2(p2, p1, path):
    r = p2["range_m"]; k = p2["keep"]
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(r[k], p2["floor_db"][k], "-o", ms=4, color="#2ecc71",
            label="floor reflectivity (symmetric)")
    ax.plot(r[k], p2["object_db"][k], "-o", ms=4, color="#e74c3c", alpha=0.7,
            label="object excess (one-sided)")
    for i in p1["reflector_idx"]:
        ax.axvline(r[i], color="#888", ls=":", lw=0.8)
    ax.set_xlabel("range (m)"); ax.set_ylabel("level (dB rel per-bin power)")
    ax.set_title("PIECE 2 — symmetric floor reflectivity vs one-sided object "
                 "excess (dotted = piece-1 reflectors)")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(path, dpi=115); plt.close(fig)


# ============================ PIECE 3 ============================
def piece3_floor_wall(bins, cov, dr, p1, p2):
    """Radial-gradient fit: fit a smooth falloff to the FLOOR (symmetric) level
    on floor-dominated bins; bins matching it = floor. The WALL = the farthest
    strong perpendicular reflector (piece-1). Returns floor_fit curve + wall range.
    """
    r = p1["range_m"]; floor = p2["floor_db"]; obj = p2["object_db"]; keep = p2["keep"]
    # floor-dominated bins: symmetric floor not much below the one-sided excess
    floor_dom = keep & (floor > obj - 6)
    A = np.column_stack([np.log10(r[floor_dom]), np.ones(floor_dom.sum())])
    coef, *_ = np.linalg.lstsq(A, floor[floor_dom], rcond=None)
    floor_fit = coef[0] * np.log10(r) + coef[1]
    on_floor = keep & (np.abs(floor - floor_fit) < 3) & floor_dom
    wall_range = float(p1["reflector_ranges"].max()) if len(p1["reflector_ranges"]) else None
    return dict(range_m=r, floor_fit=floor_fit, on_floor=on_floor,
                floor_dom=floor_dom, wall_range=wall_range, slope=coef[0])


# ============================ PIECE 6/7 (L2) ============================
def piece6_accumulate(bin_idx, capfiles, person_guard=3):
    """Accumulate a bin's covariance across captures (excl. person-contaminated)
    -> cancels random noise-floor burrs, lifts weak edges. Returns integrated R."""
    Rs = []
    for fn in capfiles:
        d = np.load(fn, allow_pickle=True)
        cb = {int(b): d["covariances"][i] for i, b in enumerate(d["bins"])}
        py = float(d["person_xyz"][1]) if "person_xyz" in d else np.nan
        pbin = int(round(py / 0.106)) if np.isfinite(py) else -99
        if bin_idx in cb and abs(bin_idx - pbin) > person_guard:
            Rs.append(cb[bin_idx])
    return np.mean(Rs, axis=0) if Rs else None


def piece7_cfar_corners(bins, cov, dr, capfiles, edge_db=4.0):
    """CFAR on the accumulated range profile: a CORNER/edge = a bin whose power
    JUMPS above its range neighbours (dihedral retroreflector, the most stable
    static feature). Returns corner (range, x, y) via calibrated to_ground."""
    rng = bins * dr
    # accumulated power per bin (noise-burr-cancelled)
    pw = []
    for b in bins:
        R = piece6_accumulate(int(b), capfiles)
        pw.append(np.real(np.trace(R)) if R is not None else np.nan)
    pw = np.array(pw); pdb = 10*np.log10(pw/np.nanmax(pw)+1e-12)
    corners = []
    for i in range(1, len(bins)-1):
        if rng[i] < NEAR_GATE or not np.isfinite(pdb[i]):
            continue
        lo, hi = max(0, i-5), min(len(bins), i+6)
        loc = np.nanmedian(np.concatenate([pdb[lo:i], pdb[i+1:hi]]))
        if pdb[i] >= loc + edge_db and pdb[i] >= pdb[i-1] and pdb[i] >= pdb[i+1]:
            R = piece6_accumulate(int(bins[i]), capfiles)
            P = _spectrum(R); az = AZG[int(np.argmax(P))]
            x, y = to_ground(rng[i], np.deg2rad(az))
            corners.append((float(rng[i]), float(x), float(y), float(pdb[i])))
    return corners


def _render_scene(bins, cov, dr, p1, p2, p3, boxes, corners, path):
    from matplotlib.patches import Rectangle
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.set_facecolor("#0d1117"); fig.patch.set_facecolor("#0d1117")
    ax.plot(0, 0, "wv", ms=14); ax.annotate("Radar", (0, 0), (0.2, 0.35), color="w", fontsize=9)
    # L0 wall
    if p3["wall_range"]:
        ax.axhline(p3["wall_range"], color="#2ecc71", ls="--", lw=2)
        ax.text(-3.3, p3["wall_range"]-0.15, f"wall {p3['wall_range']:.2f}m (L0)", color="#2ecc71", fontsize=9)
    # L1 object boxes (calibrated)
    for i, b in enumerate(boxes):
        ax.add_patch(Rectangle((b["x0"], b["y0"]), b["size"][0], b["size"][1],
                               fill=False, ec="#f39c12", lw=1.6, ls="--"))
    # L2 corners
    if corners:
        cx = [c[1] for c in corners]; cy = [c[2] for c in corners]
        ax.scatter(cx, cy, c="#e74c3c", s=90, marker="P", edgecolor="w", lw=0.4,
                   label=f"L2 CFAR corners ({len(corners)})", zorder=6)
    ax.set_xlim(-3.5, 3.5); ax.set_ylim(7, -0.3); ax.set_aspect("equal")
    ax.set_xlabel("x lateral (m)", color="#aaa"); ax.set_ylabel("y depth (m) down", color="#aaa")
    ax.tick_params(colors="#888")
    ax.set_title("LAYERED SCENE — green=L0 wall  orange=L1 object boxes  red+=L2 corners",
                 color="#ddd", fontsize=10)
    ax.legend(loc="lower left", fontsize=8)
    fig.tight_layout(); fig.savefig(path, dpi=115, facecolor="#0d1117"); plt.close(fig)


def _motion_anchors():
    """Known person/chair positions (user-view world frame), highest confidence."""
    out = []
    for name in ("ChairR", "ChairL", "standR"):
        d = np.load(f"{HERE}/../case/{name}_20260721.npz", allow_pickle=True)
        t = d["person_xyz"]
        out.append({"id": name, "x": round(float(-t[0]), 2), "y": round(float(t[1]), 2),
                    "z": round(float(t[2]), 2)})
    return out


def build_final_model(bins, cov, dr, p1, p2, p3, boxes, corners, edges=None, margin=0.4):
    """Consolidate L0/L1/L2 + motion anchors into ONE scene model (user-view,
    radar at origin +x=right +y=depth).

    Support-based pruning: every L1 box is cross-checked against the two
    independent evidence layers, within `margin` m of the box:
      - L2 CFAR corners  (the most stable static feature)  -> 'corner' support
      - motion anchors   (a person occupied that spot)     -> 'anchor' support
    Verdict: corner-anchored > anchor-only > UNVERIFIED (no independent support
    -> likely a static-azimuth multipath ghost). Nothing is dropped; unverified
    objects are kept + flagged so the renderer can dim them.
    """
    anchors = _motion_anchors()

    def support(b):
        cx, cy = b["center"]; w, h = b["size"]
        x0, y0 = cx - w/2 - margin, cy - h/2 - margin
        x1, y1 = cx + w/2 + margin, cy + h/2 + margin
        inside = lambda px, py: x0 <= px <= x1 and y0 <= py <= y1
        nc = [(round(c[1], 2), round(c[2], 2)) for c in corners if inside(c[1], c[2])]
        na = [a["id"] for a in anchors if inside(a["x"], a["y"])]
        # radar-only = detected but no INDEPENDENT cross-check. NOT a ghost:
        # the photo confirmed several of these are real furniture (left desks,
        # dresser) that simply had no sitter and give diffuse (not dihedral)
        # returns. Only objects that CONTRADICT the photo are true ghosts.
        verdict = ("corner-anchored" if nc else "anchor-only" if na else "radar-only")
        return dict(corners=nc, anchors=na, verified=bool(nc or na), verdict=verdict)

    objects = []
    for i, b in enumerate(boxes):
        sup = support(b)
        objects.append({"id": f"obj{i+1}", "center": b["center"], "size": b["size"],
                        "range_m": round(float(np.hypot(*b["center"])), 2),
                        "energy": b.get("energy"), "height_band_m": [0.0, 1.2],
                        "confidence": sup["verdict"], "support": sup})

    model = {
        "frame": {"origin": "radar", "x": "+right", "y": "+depth", "z": "+up",
                  "units": "m", "view": "user/photo (chairR on right)",
                  "radar_height_m": 2.0, "tilt_deg": 25.0},
        "walls": [{"id": "front", "y": p3["wall_range"], "confidence": "radar",
                   "note": "range peak; trim width / draw sides & doors by hand (FOV-limited)"}],
        "floor": {"model": "plane z=0", "falloff_dB_per_decade": round(float(p3["slope"]), 1)},
        "objects": objects,
        "corners": [{"x": round(c[1], 2), "y": round(c[2], 2), "power_db": round(c[3], 1)}
                    for c in corners],
        "edges": [{"x": e[1], "y": e[2], "range_m": e[0], "disc": e[3], "power_db": e[4]}
                  for e in (edges or [])],
        "motion_anchors": anchors,
        "todo_user": ["draw side/back walls (FOV-limited)", "draw doors",
                      "verify object positions (static azimuth is coarse)"],
    }
    return model


def export_ply(model, path):
    """3D model: extrude footprints. Objects -> boxes to height_band; front wall
    -> vertical quad; floor -> plane. z unreliable so heights are the ≤1.2m human
    band (spec). ASCII PLY for CloudCompare/MATLAB pcshow."""
    V, F = [], []
    def box(x0, y0, x1, y1, z0, z1):
        base = len(V)
        for (x, y) in [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]:
            V.append((x, y, z0))
        for (x, y) in [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]:
            V.append((x, y, z1))
        q = [(0,1,2),(0,2,3),(4,6,5),(4,7,6),(0,4,5),(0,5,1),
             (1,5,6),(1,6,2),(2,6,7),(2,7,3),(3,7,4),(3,4,0)]
        for a, b, c in q:
            F.append((base+a, base+b, base+c))
    for o in model["objects"]:                        # all real per photo x-check
        cx, cy = o["center"]; w, h = o["size"]; z0, z1 = o["height_band_m"]
        box(cx-w/2, cy-h/2, cx+w/2, cy+h/2, z0, z1)
    yw = model["walls"][0]["y"]                      # front wall vertical quad
    box(-3.0, yw-0.05, 3.0, yw+0.05, 0.0, 2.0)
    box(-3.5, 0.0, 3.5, min(yw, 6.5), -0.02, 0.0)     # floor plane
    with open(path, "w") as f:
        f.write(f"ply\nformat ascii 1.0\nelement vertex {len(V)}\n"
                "property float x\nproperty float y\nproperty float z\n"
                f"element face {len(F)}\nproperty list uchar int vertex_indices\nend_header\n")
        for v in V:
            f.write(f"{v[0]:.3f} {v[1]:.3f} {v[2]:.3f}\n")
        for fc in F:
            f.write(f"3 {fc[0]} {fc[1]} {fc[2]}\n")


def render_model_2d(model, path):
    from matplotlib.patches import Rectangle
    fig, ax = plt.subplots(figsize=(8.5, 9.5))
    ax.set_facecolor("#0d1117"); fig.patch.set_facecolor("#0d1117")
    ax.plot(0, 0, "wv", ms=15); ax.annotate("Radar (0,0)", (0, 0), (0.2, 0.35), color="w", fontsize=9)
    for a in (-60, 60):
        ax.plot([0, 7*np.sin(np.deg2rad(a))], [0, 7*np.cos(np.deg2rad(a))], "r:", lw=0.6, alpha=0.4)
    yw = model["walls"][0]["y"]
    ax.plot([-3, 3], [yw, yw], color="#2ecc71", lw=3)
    ax.text(-3.2, yw-0.2, f"front wall {yw:.2f}m", color="#2ecc71", fontsize=9)
    for o in model["objects"]:
        cx, cy = o["center"]; w, h = o["size"]
        col = "#f1c40f" if o["confidence"] == "corner-anchored" else "#7f8c9a"
        ax.add_patch(Rectangle((cx-w/2, cy-h/2), w, h, fill=False, ec=col, lw=1.8,
                               ls="-" if col == "#f1c40f" else "--"))
        ax.text(cx, cy-h/2-0.08, o["id"], color=col, fontsize=8, ha="center")
    for c in model["corners"]:
        ax.plot(c["x"], c["y"], "P", color="#e74c3c", ms=11, mec="w", mew=0.4)
    for m in model["motion_anchors"]:
        ax.plot(m["x"], m["y"], "*", color="#3498db", ms=20, mec="w")
        ax.annotate(m["id"], (m["x"], m["y"]), (m["x"]+0.15, m["y"]), color="#3498db", fontsize=8)
    ax.text(1.7, 5.0, "◀ draw doors / sides (FOV-limited)", color="#e67e22", fontsize=8)
    ax.set_xlim(-3.5, 3.5); ax.set_ylim(7, -0.4); ax.set_aspect("equal")
    ax.set_xlabel("x lateral (m)  →right", color="#aaa"); ax.set_ylabel("y depth (m) ↓into room", color="#aaa")
    ax.tick_params(colors="#888")
    ax.set_title("FINAL 2D SCENE MODEL (calibrated, user view)\n"
                 "green=wall  yellow=corner-anchored obj  grey=box-only  red+=corner  blue★=motion anchor",
                 color="#ddd", fontsize=9.5)
    fig.tight_layout(); fig.savefig(path, dpi=120, facecolor="#0d1117"); plt.close(fig)


def _render_floor_baseline(fb, path):
    """Self-learned floor reflectivity curve + object-excess bins."""
    r = fb["range_m"]; k = np.isfinite(fb["floor_db"])
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(r[k], fb["floor_db"][k], "o", ms=4, color="#2ecc71", alpha=0.6,
            label="floor level (angular trimmed-mean)")
    ax.plot(r[k], fb["floor_fit"][k], "-", color="#27ae60", lw=2,
            label=f"floor curve fit ({fb['slope']:.2f}dB/dec, resid {fb['resid_std']:.2f}dB)")
    ax.plot(r[k], (fb["floor_fit"] + 6.0)[k], "--", color="#95a5a6", lw=1, label="floor +6dB (object gate)")
    ax.plot(r[k], fb["peak_db"][k], ".", ms=5, color="#7f8c9a", label="az peak")
    ob = fb["object_bins"]
    ax.scatter(r[ob], fb["peak_db"][ob], c="#e74c3c", s=60, zorder=5,
               label=f"object-excess bins ({int(ob.sum())})")
    ax.axvspan(0, NEAR_GATE, color="gray", alpha=0.2)
    ax.set_xlabel("range (m)"); ax.set_ylabel("power (dB, per-bin normalised)")
    ax.set_title("L0.5 — SELF-LEARNED FLOOR BASELINE from open floor "
                 "(angular trimmed-mean; red = object above floor curve)")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(path, dpi=115); plt.close(fig)


def render_room_layout(model, path):
    """THE deliverable: a top-down ROOM — floor (walkable empty space), walls
    (near radar-wall + far front-wall confirmed; sides = editable placeholders),
    and static objects drawn as FILLED footprints = the space each occupies.
    Architectural style (not an energy scatter). Confidence: corner-anchored
    objects are solid, box-only are hatched. z is dropped (footprints only)."""
    from matplotlib.patches import Rectangle
    yw = model["walls"][0]["y"]                     # front wall depth (far)
    XW = 3.4                                         # room half-width placeholder
    fig, ax = plt.subplots(figsize=(8.5, 9.6))
    fig.patch.set_facecolor("#f4f1ea"); ax.set_facecolor("#f4f1ea")

    # ---- floor = walkable empty space between the walls ----
    ax.add_patch(Rectangle((-XW, 0.0), 2*XW, yw, facecolor="#e7e0d0",
                           edgecolor="none", zorder=0))
    ax.text(0, yw*0.5, "floor  (walkable empty space)", color="#b8ab8c",
            fontsize=11, ha="center", va="center", style="italic", zorder=1)

    # ---- walls ----
    ax.plot([-XW, XW], [yw, yw], color="#2e7d46", lw=6, solid_capstyle="butt", zorder=3)
    ax.text(0, yw+0.16, f"front wall  {yw:.2f} m  (radar-confirmed depth)",
            color="#2e7d46", fontsize=10, ha="center")
    ax.plot([-XW, XW], [0, 0], color="#7a6f57", lw=6, solid_capstyle="butt", zorder=3)
    ax.text(-XW+0.1, -0.22, "near wall (radar mounted here)", color="#7a6f57", fontsize=8.5)
    for sx in (-XW, XW):                             # side walls = placeholders
        ax.plot([sx, sx], [0, yw], color="#b0a598", lw=2, ls=(0, (6, 5)), zorder=3)
    ax.text(XW-0.1, yw*0.62, "side wall\n(draw / adjust)", color="#9c8f74",
            fontsize=8.5, ha="right", rotation=90, va="center")

    # ---- static objects = filled footprints (space occupied), 3 tiers ----
    # corner-anchored (L2 CFAR) strongest; anchor-only (person occupied);
    # radar-only (detected, no independent cross-check -- STILL real per photo,
    # just lower confidence, verify vs photo). None are dropped.
    STYLE = {  # facecolor, edgecolor, alpha, hatch
        "corner-anchored": ("#c0663a", "#6d3418", 0.85, None),
        "anchor-only":     ("#d9a24a", "#8a5a12", 0.78, None),
        "radar-only":      ("#cdbfa6", "#9c8f74", 0.55, None),
    }
    # object identities are the USER's to assign (radar gives position +
    # confidence only; my photo guesses were wrong twice). Left blank on purpose.
    PHOTO_ID = {}
    for o in model["objects"]:
        cx, cy = o["center"]; w, h = o["size"]
        x0 = cx - w/2; y0 = cy - h/2
        h = max(min(cy + h/2, yw) - y0, 0.05)         # clip to front wall
        fc, ec, al, ht = STYLE.get(o["confidence"], STYLE["radar-only"])
        strong = o["confidence"] != "radar-only"
        ax.add_patch(Rectangle((x0, y0), w, h, facecolor=fc, edgecolor=ec,
                               lw=1.4, alpha=al, hatch=ht,
                               ls="-" if strong else (0, (4, 2)), zorder=4))
        area = round(w * h, 2)                            # OCCUPIED footprint area (m^2)
        ax.text(cx, cy, o["id"], color="white" if strong else "#5a4d34",
                fontsize=8, ha="center", va="center", zorder=5,
                fontweight="bold" if strong else "normal")
        ax.text(cx, cy+0.16, f"{area} m²", color="white" if strong else "#6a5d44",
                fontsize=6.5, ha="center", va="center", zorder=5)

    # ---- edges / folds / corners (spatial discontinuities = 棱线) ----
    for e in model.get("edges", []):
        s = 30 + 90 * min(e["disc"], 0.6) / 0.6           # marker size by discontinuity
        ax.plot(e["x"], e["y"], marker="x", color="#8e44ad", ms=7, mew=1.6, zorder=5)
    if model.get("edges"):
        ax.plot([], [], "x", color="#8e44ad", label="edge/fold (跃变)")

    # ---- radar + FOV ----
    for a in (-60, 60):
        ax.plot([0, 7*np.sin(np.deg2rad(a))], [0, 7*np.cos(np.deg2rad(a))],
                color="#c0392b", ls=":", lw=0.8, alpha=0.35, zorder=2)
    ax.plot(0, 0, marker="v", color="#c0392b", ms=16, zorder=6)
    ax.text(0.2, -0.22, "Radar (0,0)  H2.0m tilt25°", color="#c0392b", fontsize=8.5)

    # ---- motion anchors = the MOST accurate objects (0.14m, breathing
    # fluctuation): draw the space they occupy as a footprint box, highest
    # confidence (solid blue). Seated -> chair footprint; standing -> person. ----
    for m in model["motion_anchors"]:
        seated = m["id"].lower().startswith("chair")
        w = 0.5 if seated else 0.4                    # chair vs standing-person footprint
        ax.add_patch(Rectangle((m["x"]-w/2, m["y"]-w/2), w, w, facecolor="#2f6fb0",
                               edgecolor="#173a5e", lw=1.5, alpha=0.9, zorder=6))
        ax.plot(m["x"], m["y"], "*", color="white", ms=9, zorder=7)
        ax.text(m["x"], m["y"]+w/2+0.08, m["id"], color="#2f6fb0", fontsize=7.5,
                ha="center", va="bottom", zorder=7, fontweight="bold")

    ax.set_xlim(-XW-0.2, XW+0.2); ax.set_ylim(yw+0.6, -0.5); ax.set_aspect("equal")
    ax.set_xlabel("x lateral (m)  → right", color="#555")
    ax.set_ylabel("y depth (m)  ↓ into room", color="#555")
    ax.tick_params(colors="#888")
    for sp in ax.spines.values():
        sp.set_color("#cfc7b4")
    ax.set_title("STATIC ROOM LAYOUT — floor · walls · static-object footprints\n"
                 "blue=person-verified (0.14m, best)  orange=L2-corner  tan=person-anchored  pale=radar-only",
                 color="#3a3320", fontsize=9)
    fig.tight_layout(); fig.savefig(path, dpi=120, facecolor="#f4f1ea"); plt.close(fig)


def render_xy_basemap(model, xs, ys, raw, path):
    """Overlay the interpreted layout (walls/boxes/edges/anchors) ON the real X/Y
    energy heatmap (底图) so each static object box can be eyeballed against the
    actual radar energy blob it claims to be. Same user-view frame as the layout."""
    from matplotlib.patches import Rectangle
    def db(a):
        return 10 * np.log10(a / (a.max() + 1e-12) + 1e-4)
    yw = model["walls"][0]["y"]; XW = 3.0; YMAX = 6.5     # 6 (wide) x 6.5 (deep) m base
    fig, ax = plt.subplots(figsize=(9.0, 9.6))
    ax.imshow(db(raw), origin="lower", extent=[xs[0], xs[-1], ys[0], ys[-1]],
              aspect="equal", cmap="inferno", vmin=-25, vmax=0, zorder=0)
    # walls
    ax.plot([-XW, XW], [yw, yw], color="#2ecc71", lw=3, zorder=3)
    ax.text(0, yw+0.15, f"front wall {yw:.2f}m", color="#2ecc71", fontsize=9, ha="center")
    for sx in (-XW, XW):
        ax.plot([sx, sx], [0, yw], color="#7f8c9a", lw=1.5, ls=(0, (6, 5)), zorder=3)
    ax.plot([-XW, XW], [0, 0], color="#95a5a6", lw=2, zorder=3)
    # object boxes (outline only, so the energy shows through) + area
    for o in model["objects"]:
        cx, cy = o["center"]; w, h = o["size"]
        col = {"corner-anchored": "#f39c12", "anchor-only": "#e67e22"}.get(o["confidence"], "#ecf0f1")
        ax.add_patch(Rectangle((cx-w/2, cy-h/2), w, min(cy+h/2, yw)-(cy-h/2),
                     fill=False, ec=col, lw=1.8,
                     ls="-" if o["confidence"] != "radar-only" else (0, (4, 2)), zorder=4))
        ax.text(cx, cy, f'{o["id"]}\n{round(w*h,2)}m²', color=col, fontsize=7.5,
                ha="center", va="center", zorder=5, fontweight="bold")
    # edges + person anchors
    for e in model.get("edges", []):
        ax.plot(e["x"], e["y"], "x", color="#9b59b6", ms=8, mew=1.8, zorder=5)
    for m in model["motion_anchors"]:
        ax.plot(m["x"], m["y"], "*", color="#3498db", ms=17, mec="w", mew=0.6, zorder=6)
        ax.annotate(m["id"], (m["x"], m["y"]), (m["x"]+0.12, m["y"]-0.05), color="#5dade2", fontsize=8)
    ax.plot(0, 0, "wv", ms=13, zorder=6); ax.text(0.2, -0.2, "Radar", color="w", fontsize=8)
    ax.set_xlim(-XW, XW); ax.set_ylim(YMAX, -0.4); ax.set_aspect("equal")
    ax.set_xlabel("x lateral (m) → right"); ax.set_ylabel("y depth (m) ↓ into room")
    ax.set_title("STATIC LAYOUT over the real X/Y ENERGY (底图) — check each box sits on a blob\n"
                 "orange=corner/anchor obj  white-dash=radar-only  ✕=edge  ★=person anchor", fontsize=9)
    fig.tight_layout(); fig.savefig(path, dpi=120); plt.close(fig)


def write_baseline(bins, cov, dr, path):
    """#5 occlusion baseline: per-bin features + antisym X/Y grids. A separate
    STAGE (not a box duplicate) — the reference an incoming person is diffed
    against. Kept in the single pipeline so it regenerates with everything else.
    """
    covl = [cov[int(b)] for b in bins]
    f = per_bin_features(bins, covl, dr)
    xr, yr, raw, obj = build_xy_grid(bins, covl, dr)
    np.savez(path, bins=f["bins"], range_m=f["range_m"],
             power_db=f["power_db"], trend_db=f["trend_db"], resid_db=f["resid_db"],
             label=f["label"].astype(str), sym_level=f["sym_level"],
             antisym_frac=f["antisym_frac"], side_deg=f["side_deg"],
             dom_az_deg=f["dom_az_deg"], xy_xs=xr, xy_ys=yr, xy_raw=raw, xy_obj=obj,
             xy_cell=0.15, mount_m=2.0, tilt_deg=25.0, dr_m=dr)


if __name__ == "__main__":
    # ============================================================
    # THE static-scene pipeline (single entry point). Different methods,
    # chained coarse->fine like a vision pipeline — each STAGE solves what the
    # previous one cannot, feeding the next; NO parallel duplicate box path.
    #   L0 coarse : range-CFAR + symmetry-floor + floor/wall fit  (radial axis)
    #   L1 mid    : coarse->fine 70%-energy boxes                 (object X/Y)
    #   L2 fine   : accumulated CFAR corners                      (stable anchors)
    #   fuse      : motion anchors (only exact localizer) -> model + 2D/3D
    #   baseline  : #5 occlusion reference grid
    # ============================================================
    import json
    EMPTY = f"{HERE}/../case/empty_20260721.npz"
    CAPS = [f"{HERE}/../case/{n}_20260721.npz" for n in ("empty", "ChairR", "ChairL", "standR")]
    bins, cov, dr = load(EMPTY)

    # ---- L0 coarse: reliable radial axis ----
    p1 = piece1_range_profile(bins, cov, dr)
    _render_piece1(bins, dr, p1, f"{HERE}/layer_piece1.png")
    p2 = piece2_floor_reflectivity(bins, cov, dr)
    _render_piece2(p2, p1, f"{HERE}/layer_piece2.png")
    p3 = piece3_floor_wall(bins, cov, dr, p1, p2)
    print(f"L0 — floor falloff {p3['slope']:.1f}dB/dec, "
          f"{int(p3['on_floor'].sum())} floor bins, wall @ {p3['wall_range']:.2f}m")

    # ---- L0.5 floor baseline: self-learn floor reflectivity from open floor ----
    fb = learn_floor_baseline(bins, cov, dr)
    print(f"L0.5 — floor baseline: slope {fb['slope']:.2f}dB/dec, "
          f"residual {fb['resid_std']:.2f}dB, {int(fb['object_bins'].sum())} object-excess bins")
    _render_floor_baseline(fb, f"{HERE}/layer_floor_baseline.png")

    # ---- L1 mid: per-bin object boxes, thresholded vs the LEARNED FLOOR curve
    # (near-big/far-small per-bin threshold; slant->ground; merge bins) ----
    d = np.load(EMPTY, allow_pickle=True)
    boxes, _ = detect_floor_relative(d["bins"].astype(int), d["covariances"], dr,
                                     margin_db=6.0, merge_gap=0.25)
    print(f"L1 — {len(boxes)} floor-relative object boxes")

    # ---- L2 fine: accumulated CFAR corners (most stable static feature) ----
    corners = piece7_cfar_corners(bins, cov, dr, CAPS)
    print(f"L2 — {len(corners)} CFAR corners:", [f"({c[1]:+.1f},{c[2]:.1f})" for c in corners])
    _render_scene(bins, cov, dr, p1, p2, p3, boxes, corners, f"{HERE}/layer_scene.png")

    # ---- L2.5 edges/folds: spatial discontinuities (跃变 = 棱线/折/角) ----
    ed = edge_from_discontinuity(bins, cov, dr)
    edges = ed["edges"]
    print(f"L2.5 — {len(edges)} edges/folds (disc):", [f"({e[1]:+.1f},{e[2]:.1f}|{e[3]})" for e in edges[:6]])

    # ---- fuse: one model (2D json + png + 3D ply) ----
    model = build_final_model(bins, cov, dr, p1, p2, p3, boxes, corners, edges=edges)
    with open(f"{HERE}/scene_model_20260721.json", "w") as f:
        json.dump(model, f, indent=2)
    render_model_2d(model, f"{HERE}/scene_model_2d.png")
    render_room_layout(model, f"{HERE}/scene_room_layout.png")   # THE deliverable
    xr, yr, xy_raw, _ = build_xy_grid(bins, {int(b): cov[int(b)] for b in bins}, dr)
    render_xy_basemap(model, xr, yr, xy_raw, f"{HERE}/scene_xy_basemap.png")  # layout over energy 底图
    export_ply(model, f"{HERE}/scene_model_3d.ply")
    print(f"FUSE — {len(model['objects'])} objects "
          f"({sum(o['confidence']=='corner-anchored' for o in model['objects'])} corner-anchored), "
          f"{len(model['corners'])} corners, {len(model['motion_anchors'])} motion anchors, "
          f"wall {model['walls'][0]['y']:.2f}m")

    # ---- baseline: #5 occlusion reference ----
    write_baseline(bins, cov, dr, f"{HERE}/static_baseline_20260721.npz")
    print("saved layer_piece1/2.png, layer_scene.png, scene_model_2d.png, "
          "scene_model_20260721.json, scene_model_3d.ply, static_baseline_20260721.npz")
