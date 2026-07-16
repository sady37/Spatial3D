"""Offline replay of a People_Tracking scene recording (radar_source._flush_scene npz).

Reconstructs — WITHOUT the radar — exactly what web/radar_server._scene computes per
frame, so you can validate the fall pipeline on saved walking / lie data:

  * per-track boxes from the 3001 cloud  (connectivity cluster -> nearest track)
  * box pose STAND / SIT / LIE           via  zspan / max(xspan, yspan)   [the LIE fix]
  * the track -> fall-trigger(320) -> fall state machine (SuspectedFall / Fall)

Prints a compact timeline (state changes + ~1 Hz ticks) and a summary. Old recordings
that predate the point-cloud/320 columns still replay the track-Z timeline.

    .venv/bin/python3 replay_fall.py record/live_scene_XXXXXX.npz
    .venv/bin/python3 replay_fall.py record/live_scene_XXXXXX.npz --png   # + side-view PNG at the first Fall

Geometry + thresholds are kept in lock-step with radar_server.py; if you tune them
there, mirror them here (constants below).
"""
import sys, os, math
import numpy as np

# ---- keep these identical to web/radar_server.py ----
MOUNT, TILT = 2.0, 35.0            # sensor height (m), down-tilt (deg)
EPS = 0.4                          # cloud connectivity cluster radius (m)
MIN_PTS = 4                        # drop clusters smaller than this
NEAR_TRK = 0.8                     # cluster must be within this of a track (m)
HOLD_S, CONFIRM_S = 3.0, 2.5       # fall state machine (see _scene)
NN_DISCRETE = 5.0                  # discrete-point removal: nn > 5x mean -> drop


def load(path):
    d = np.load(path, allow_pickle=True)
    have = set(d.files)
    return d, have


def frame_rows(frame_col, fi, *cols):
    m = frame_col == fi
    return tuple(c[m] for c in cols)


def pose_of(x0, x1, y0, y1, z0, z1):
    """Mirror of radar_server._pose_of: L=footprint(XY), Zv=vertical(XZ/YZ); flat+spread=LIE."""
    L = max(x1 - x0, y1 - y0); Zv = z1 - z0
    if L >= 0.9 and Zv < 0.6:
        return "LIE"
    if Zv >= 1.0:
        return "STAND"
    asp = Zv / max(L, 0.05)
    return "STAND" if asp > 1.5 else ("LIE" if asp < 0.7 else "SIT")


def cluster_boxes(px, py, pz, tracks):
    """Mirror of _scene's block-person clustering. tracks = list of (tid, x, y, z).
    Returns list of dicts {tid, pose, x0..z1, n} and the world (wy, wz) arrays."""
    from scipy.spatial import cKDTree
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    th = math.radians(TILT)
    wz = MOUNT + pz * math.cos(th) - py * math.sin(th)     # world height
    wy = py * math.cos(th) + pz * math.sin(th)             # world ground range
    P = np.stack([px, wy, wz], axis=1)
    if len(P) >= 4:                                        # discrete-point removal
        nn = cKDTree(P).query(P, k=2)[0][:, 1]
        keep = nn <= NN_DISCRETE * float(nn.mean())
        px, py, wy, wz, P = px[keep], py[keep], wy[keep], wz[keep], P[keep]
    n = len(P)
    if n == 0 or not tracks:
        return [], wy, wz
    pr = cKDTree(P).query_pairs(EPS, output_type='ndarray')
    if len(pr):
        r = np.concatenate([pr[:, 0], pr[:, 1]]); c = np.concatenate([pr[:, 1], pr[:, 0]])
        _, labels = connected_components(
            coo_matrix((np.ones(len(r)), (r, c)), shape=(n, n)), directed=False)
    else:
        labels = np.arange(n)
    q = lambda a: (float(np.percentile(a, 5)), float(np.percentile(a, 95)))
    tx = np.array([t[1] for t in tracks]); ty = np.array([t[2] for t in tracks])
    tid = np.array([t[0] for t in tracks])
    bytid = {}                                            # ti -> ONE merged box per track
    for lab in np.unique(labels):
        m = labels == lab
        if int(m.sum()) < MIN_PTS:
            continue
        cx, cy = float(px[m].mean()), float(py[m].mean())
        d2 = (tx - cx) ** 2 + (ty - cy) ** 2; ti = int(d2.argmin())
        if float(d2[ti]) > NEAR_TRK ** 2:
            continue
        x0, x1 = q(px[m]); y0, y1 = q(wy[m]); z0, z1 = q(wz[m])
        b = bytid.get(ti)
        if b is None:
            bytid[ti] = dict(tid=int(tid[ti]), ti=ti, n=int(m.sum()),
                             x0=x0, x1=x1, y0=y0, y1=y1, z0=z0, z1=z1)
        else:                                             # merge same-track fragments (torso+legs)
            b["x0"] = min(b["x0"], x0); b["x1"] = max(b["x1"], x1)
            b["y0"] = min(b["y0"], y0); b["y1"] = max(b["y1"], y1)
            b["z0"] = min(b["z0"], z0); b["z1"] = max(b["z1"], z1)
            b["n"] += int(m.sum())
    boxes = []
    for b in sorted(bytid.values(), key=lambda b: b["ti"]):
        b["pose"] = pose_of(b["x0"], b["x1"], b["y0"], b["y1"], b["z0"], b["z1"])
        b["zspan"] = b["z1"] - b["z0"]; b["dspan"] = max(b["x1"] - b["x0"], b["y1"] - b["y0"])
        boxes.append(b)
    return boxes, wy, wz


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    flags = {a for a in sys.argv[1:] if a.startswith("--")}
    if not args:
        sys.exit(__doc__)
    path = args[0]
    d, have = load(path)
    ts = d["ts"].astype(float)
    t0 = ts[0] if len(ts) else 0.0
    nfr = len(ts)
    has_cloud = "pc_xyz" in have and "p_frame" in have and len(d["pc_xyz"])
    has_320 = "e_frame" in have and len(d["e_frame"])
    print(f"# {os.path.basename(path)}: {nfr} frames, {(ts[-1]-t0 if nfr>1 else 0):.1f}s"
          f"  cloud={'yes' if has_cloud else 'NO (old recording)'}  320={'yes' if has_320 else 'none'}")
    if not has_cloud:
        print("#   -> no point cloud saved: boxes/side-morphology unavailable, "
              "track-Z + fall-trigger timeline only.")

    tf = d["t_frame"]; ttid = d["t_tid"]; tx = d["t_x"]; ty = d["t_y"]; tz = d["t_z"]
    ef = d["e_frame"] if has_320 else np.empty(0, int)
    pf = d["p_frame"] if has_cloud else np.empty(0, int)
    pxyz = d["pc_xyz"] if has_cloud else np.empty((0, 3), np.float32)

    cube_hold_ts, lie_since = 0.0, 0.0
    state = "none"; prev_state = None; prev_poses = None
    first = {"suspected": None, "fall": None}
    dwell = {"none": 0, "suspected": 0, "fall": 0}
    last_tick = -1e9
    png_done = False

    for fi in range(nfr):
        t = ts[fi]; trel = t - t0
        (tid_i, x_i, y_i, z_i) = frame_rows(tf, fi, ttid, tx, ty, tz)
        tracks = list(zip(tid_i.tolist(), x_i.tolist(), y_i.tolist(), z_i.tolist()))
        n320 = int((ef == fi).sum()) if has_320 else 0

        # --- boxes / poses FIRST (merged per track): pose drives the decision ---
        boxes = []
        if has_cloud and tracks:
            sub = pxyz[pf == fi]
            if len(sub):
                boxes, _, _ = cluster_boxes(sub[:, 0], sub[:, 1], sub[:, 2], tracks)
        poses = [f"T{b['tid']}:{b['pose']}(顶Z{b['z1']:.2f} 展{b['dspan']:.2f})" for b in boxes]
        prim = next((b for b in boxes if b["ti"] == 0), None) or (
            max(boxes, key=lambda b: b["n"]) if boxes else None)
        primary_pose = prim["pose"] if prim else None
        pose_str = " ".join(poses) if poses else (f"{len(tracks)}trk" if tracks else "-")

        # --- fall state machine (mirror _scene): pose LIE sustained -> fall; 320 = aux trigger ---
        if n320 > 0:
            cube_hold_ts = t
        lying = primary_pose == "LIE"
        if lying:
            if lie_since == 0.0:
                lie_since = t
        else:
            lie_since = 0.0
        lie_dur = (t - lie_since) if lie_since else 0.0
        trig320 = bool(tracks) and cube_hold_ts > 0 and (t - cube_hold_ts) < HOLD_S
        if lying and lie_dur >= CONFIRM_S:
            state = "fall"
        elif lying or trig320:
            state = "suspected"
        else:
            state = "none"
        dwell[state] += 1
        if state in first and first[state] is None and state != "none":
            first[state] = trel

        changed = (state != prev_state) or (poses != prev_poses)
        tick = trel - last_tick >= 1.0
        if changed or tick:
            tag = {"none": " ", "suspected": "⚠S", "fall": "⚑F"}[state]
            print(f"{trel:6.1f}s {tag:>3} {state:<9} 320:{n320:<2} | {pose_str}")
            last_tick = trel
            prev_state, prev_poses = state, poses

        # optional PNG at the first confirmed fall
        if ("--png" in flags) and state == "fall" and has_cloud and not png_done:
            _side_png(path, trel, pxyz[pf == fi])
            png_done = True

    print("\n# summary")
    fps = nfr / (ts[-1] - t0) if nfr > 1 else 0
    print(f"#   fps≈{fps:.1f}  dwell: none={dwell['none']} suspected={dwell['suspected']} fall={dwell['fall']} frames")
    if not has_cloud:
        print("#   no point cloud -> no pose -> fall stayed 'none'. Re-record with the updated "
              "recorder; fall is now pose(LIE)-driven and works in ANY cfg (320 = aux trigger).")
    else:
        print(f"#   first SuspectedFall @ {first['suspected']}   first Fall @ {first['fall']}")


def _side_png(path, trel, sub):
    """Side view D(range) x Z(height) of the fall-frame cloud (fall morphology)."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        print("#   (matplotlib unavailable, skipping --png)"); return
    th = math.radians(TILT)
    px, py, pz = sub[:, 0], sub[:, 1], sub[:, 2]
    wz = MOUNT + pz * math.cos(th) - py * math.sin(th)
    wy = py * math.cos(th) + pz * math.sin(th)
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.axhspan(-0.4, 0.5, color="#f85149", alpha=0.08)
    ax.axhline(0, color="#888", lw=1)
    ax.scatter(wy, wz, s=10, c="#f08c00")
    ax.set_xlim(0, 5); ax.set_ylim(-0.4, 2.0)
    ax.set_xlabel("D range (m)"); ax.set_ylabel("Z height (m)")
    ax.set_title(f"Fall morphology @ {trel:.1f}s  topZ={wz.max():.2f}m  spread={wy.ptp():.2f}m")
    out = f"{os.path.splitext(path)[0]}_fall_{trel:.0f}s.png"
    fig.tight_layout(); fig.savefig(out, dpi=110); plt.close(fig)
    print(f"#   side-view PNG -> {out}")


if __name__ == "__main__":
    main()
