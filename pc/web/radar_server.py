"""SERVER layer — DISPLAY ONLY. Holds a Source, runs the compute layer on the
latest window, serves the dashboard + JSON. No vitals math lives here; swap the
algorithm in radar_pipeline.py and this file is unchanged.

    python3 radar_server.py live
    python3 radar_server.py chairL_hr_val_20260713/chairL_sit_20260713_225001.npz@chairL_hr_val_20260713/watch_hr_0713.csv
    # then open http://localhost:8765
"""
from __future__ import annotations
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # pc/ -> falldet
import radar_pipeline as pipe
from radar_source import make_source
from falldet.window import FloorMap, WindowDetector
from falldet.clean import Cleaner

WIN_S = 20.0            # analysis window (user 2026-07-14, per sleepad validation 20-30s) —
                        # shorter window = faster real-time response to holds / person leaving
HERE = os.path.dirname(os.path.abspath(__file__))

# Presence PERSISTENCE: living_window (single window) false-positives ~11% on an
# elevated-noise empty room (12um clutter passes the concentration/cluster test now and
# then), which leaked 'RR=18' with nobody present. Require a MAJORITY of the last
# PERSIST_S of per-window verdicts (the validated living_present logic, applied to the
# real-time stream) — a lone flicker no longer declares a person.
from collections import deque
PERSIST_S = 25.0
PERSIST_FRAC = 0.5
_present_hist = deque()   # (t, present_bool) from each fresh compute
# Presence: RR (breathing) OPENS THE DOOR (living_window establishes a person via the RR-band
# spatial concentration, and rejects empty-room noise). Once open, the static BODY REFLECTION
# HOLDS presence for up to HOLD_MAX_S during a breath-hold (person there, not breathing) — a
# breath-hold otherwise reads 'absent' and shuts off the pause counter when it should climb.
# Reflection lost, or the hold exceeds HOLD_MAX_S -> 'left'.
HOLD_REFL_FRAC = 0.6
HOLD_MAX_S = 60.0         # body reflection holds presence 60s after breathing stops (user spec)
_refl_ema = None
_last_present_t = 0.0
_person_bin = None       # remembered range-bin of the person (for range-specific hold-over)

_src = None
_meta = None
_shutdown = None     # set in main() to the graceful stop fn (for /api/quit)
# Mounting geometry for the height (H) computation only. Fixed physical install
# (--tilt/--mount override for a different rig). NO fall/posture logic lives here —
# the scene reports H (world height of the track) and the raw track coords; fall
# detection is done elsewhere (energy-density pipeline), not in this server.
TILT = 25.0          # radar DOWN-tilt from horizontal, deg (measured install 20260716)
MOUNT = 2.0          # sensor height above floor, m
FLOOR_Z = 0.4        # m; floor band top (matches radar_pipeline FLOOR_Z) — fall = energy below this
# Elevation angular ACCURACY (single-target localization), NOT the 29° two-target resolution.
# One person's height blur is governed by accuracy (~3-6° on-axis), not the ability to separate
# two targets. Beam blur grows with range: a point at slant range R has vertical uncertainty
# ~R·sin(acc). The SCENE local floor (true zero) at range R drops by HALF a cell: -0.5·rad(acc)·R,
# so a point/marker is "below floor" (=> real Fall) only if it sinks past that. Validated on the
# fall clip: at 6° the fall/crawl segment puts 5.3% of energy below the local floor vs 1.4% while
# walking (3.8x separation); the doc's 29° RESOLUTION kills it (0.2% vs 0.5%, no separation).
# Lower (3°) = more sensitive, higher = stricter. Exposed to dashboard for the sloped zero line.
ELEV_ACC_DEG = 6.0
_cache = {"t": 0.0, "key": None, "state": None}
_lock = threading.Lock()

# ---- live falldet pipeline (server-side reference; mirrors the on-chip Phase-3 window) ----
RANGE_STEP = 0.085          # m per range bin (probe: bin36 = 3.07 m)
_floor = FloorMap(cell=0.5)                                   # rolling floor map H_g(x,y)
_floor_pts = []                                              # recent world points for calibration
_window = WindowDetector(_floor, margin=0.45, sustain=3, clear=3)  # sustain in SCENE-calls (~3/s)
_cleaner = Cleaner(mlp_trig=0.5, persist=2, floor_frac_min=0.7)
_cube_busy = [False]         # a request_cube fetch is in flight (1-elem list = mutable flag)
_last_query_t = [0.0]        # last cubeQuery wall time (rate-limit: 1 per fall episode + refresh)
QUERY_REFRESH_S = 4.0        # min seconds between cubeQuery bursts while down
_real_since = [0.0]          # last time the real-person gate was instantaneously true
REAL_GRACE_S = 2.0           # hold real-person through brief point-count dips (see below)
_fall_latch_until = [0.0]    # a confirmed red Fall LATCHES the display until this wall time
FALL_HOLD_S = 30.0           # keep showing red Fall this long after the last confirmation
                             # (a caregiver must SEE it; it must not clear when the person
                             # stirs/gets up). Cleared by /api/fall/reset.


_cube_result = {"rr": None, "strength": 0.0, "t": 0.0, "floor_frac": 0.0}   # latest cube 2nd-check


def _rr_from_cube(entries, fps=10.0):
    """Breathing-band RR straight from a fetched 320 burst (the cube second-check —
    self-contained, no vitals presence gate). Per range bin, the ant-0 slow-time -> FFT ->
    fraction of power in 0.15-0.5 Hz; the strongest bin wins. A LIVING body on the floor
    has clear breathing here; a DROPPED OBJECT does not -> RR None -> not a red Fall.
    Returns (rr_bpm | None, strength 0..1)."""
    import numpy as _np
    from collections import defaultdict
    if not entries:
        return None, 0.0
    byb = defaultdict(list)
    for e in entries:
        byb[int(e.range_bin)].append(complex(e.vec[0]))
    best = (0.0, None)
    for series in byb.values():
        if len(series) < 12:
            continue
        x = _np.asarray(series, complex); x = x - x.mean()
        F = _np.abs(_np.fft.fft(x)) ** 2; f = _np.fft.fftfreq(len(x), 1.0 / fps)
        keep = _np.abs(f) > 0.05; band = keep & (_np.abs(f) >= 0.15) & (_np.abs(f) <= 0.5)
        tot = F[keep].sum() + 1e-9
        ratio = F[band].sum() / tot
        if ratio > best[0]:
            pk = abs(f[band][_np.argmax(F[band])]) if band.any() else 0.0
            best = (ratio, pk * 60.0)
    strength, rr = best
    return (round(rr, 1) if strength > 0.25 else None), round(strength, 2)


def _fetch_cube_bg(range_bin, floor_frac):
    """Background: burst 320 at range_bin, then compute RR from it (the cube second-check).
    Non-blocking for /api/scene."""
    try:
        if hasattr(_src, "request_cube"):
            ents = _src.request_cube(range_bin, n_frames=60, half_win=3, timeout=9.0)  # ~6s = ~2 breaths
            rr, strength = _rr_from_cube(ents)
            _cube_result.update(rr=rr, strength=strength, t=time.time(),
                                floor_frac=round(float(floor_frac), 2))
    except Exception:
        pass
    finally:
        _cube_busy[0] = False


def _fall_range_bin(sc):
    """Range bin to cubeQuery = the fallen body's WORLD GROUND range wy (= py*cos(tilt) +
    pz*sin(tilt)). VERIFIED on 233000: the 320 breathing bins (29-44, median 35) match the
    low cloud's GROUND range (bin 34), NOT its SLANT range (bin 44, ~1 m too far). So 320
    fires at the ground-projected range, and we compute it straight from the cloud —
    track-INDEPENDENT and ALWAYS computable (this is what fixes the recent-320-bin bootstrap
    deadlock: a fall with no prior 320 — e.g. 235000 — now still gets a valid query range,
    so cubeQuery fires and bootstraps 320). Returns None only if there is no cloud."""
    import numpy as _np, math as _m
    pcx = sc.get("pc_xyz")
    if pcx is None or not len(pcx):
        return None
    th = _m.radians(TILT or 0.0)
    py, pz = pcx[:, 1], pcx[:, 2]
    wy = py * _m.cos(th) + pz * _m.sin(th)                # world GROUND range (matches 320)
    wz = MOUNT + pz * _m.cos(th) - py * _m.sin(th)        # world height -> pick the fallen body
    low = wz < (_floor.default + 0.5)
    sel = low if int(low.sum()) >= 4 else _np.ones(len(pcx), bool)
    return int(round(float(_np.median(wy[sel])) / RANGE_STEP))


def _pose_of(x0, x1, y0, y1, z0, z1):
    """Pose from the two projections' extents of a per-track MERGED box:
    L = horizontal footprint (XY, longest side), Zv = vertical extent (XZ/YZ).
    A fallen body lies FLAT -> L long + Zv small (平铺). Ratio is scale-free (robust
    to the ~1 m track-Z drift, since it compares extents not absolute height)."""
    L = max(x1 - x0, y1 - y0)
    Zv = z1 - z0
    if L >= 0.9 and Zv < 0.6:
        return "LIE"                   # flat + spread out = 平铺倒地
    if Zv >= 1.0:
        return "STAND"                 # clearly tall column
    asp = Zv / max(L, 0.05)
    return "STAND" if asp > 1.5 else ("LIE" if asp < 0.7 else "SIT")


def _state(bin_lo, bin_hi):
    """Compute (cached ~0.5s) the current state for the given HR bin window."""
    key = (bin_lo, bin_hi)
    now = time.time()
    with _lock:
        if key == _cache["key"] and now - _cache["t"] < 0.5:
            return _cache["state"]
    win = _src.window(WIN_S)
    if win is None:
        st = {"present": None, "warming": True}
    else:
        cube_win, bins, dr, fps, t_wall, watch_hr = win
        st = pipe.analyze(cube_win, bins, dr, fps, hr_bin_lo=bin_lo, hr_bin_hi=bin_hi,
                          tilt_deg=TILT, h_mount=MOUNT)
        st["t_wall"] = round(t_wall, 2)
        st["watch_hr"] = watch_hr
        st["warming"] = False
    with _lock:
        # NOTE: a static-reflection 'breath-hold hold-over' was tried and REMOVED — the
        # background/empty-chair reflects ~the same as the still body, so it can't tell a
        # breath-HOLD from a person LEAVING (both = no breathing), and it kept RR showing
        # for 60s after the person left. Presence now strictly follows BREATHING (below).
        # Consequence: a breath-hold also reads 'no breathing' — this radar cannot separate
        # hold from departure by static reflection. Prioritize: no breathing => no RR.
        # presence PERSISTENCE: a single living_window verdict flickers to 'present' on
        # an elevated-noise empty room (leaked RR=18 with nobody there). Keep the last
        # PERSIST_S of verdicts and require a majority — a lone flicker no longer counts.
        p = st.get("present")
        if p is not None and not st.get("warming"):
            _present_hist.append((now, bool(p)))
            while _present_hist and now - _present_hist[0][0] > PERSIST_S:
                _present_hist.popleft()
            flags = [f for _, f in _present_hist]
            frac = sum(flags) / len(flags) if flags else 0.0
            st["present_frac"] = round(frac, 2)
            if len(flags) >= 5 and frac < PERSIST_FRAC and st.get("present"):
                st.update(present=False, hr=None, rr=None, hr_strength=None,
                          hr_confident=False, hr_level="none", hr_reason="empty",
                          fall=False, pose="empty")
        _cache.update(t=now, key=key, state=st)
    return st


def _scene():
    """Live People_Tracking scene: points + tracks + world height H of the track.
    H = mount + posZ*cos(tilt) - posY*sin(tilt) (TI returns posX/Y/Z; H is this
    deterministic rotation to world coords). NO posture/fall classification here."""
    if not hasattr(_src, "scene"):
        return {"live": False}
    import math, numpy as _np
    sc = _src.scene()
    pts_raw = sc.get("points")
    tgts = sc.get("targets") or []
    mount_m = (MOUNT if MOUNT is not None else 1.0)
    mount_cm = mount_m * 100.0
    th = math.radians(TILT or 0.0)          # radar DOWN-tilt, deg (0 = looking horizontal)

    def h_cm(y, z):                          # world height above floor, cm (tilt-corrected)
        return (mount_m + z * math.cos(th) - y * math.sin(th)) * 100.0

    pts = ([[round(float(p[0]), 3), round(float(p[1]), 3), round(float(p[2]), 3)]
            for p in pts_raw] if pts_raw is not None and len(pts_raw) else [])
    poses = sc.get("poses") or {}            # {tid: Pose} from firmware TLV 321
    tg = [{"tid": int(t.tid), "x": round(t.x, 3), "y": round(t.y, 3),
           "z": round(t.z, 3), "speed": round(t.speed, 2)} for t in tgts]
    for d in tg:                             # attach per-track fall legs (TLV 321)
        p = poses.get(d["tid"])
        if p is not None and p.valid:
            d["pose"] = p.label
            d["falling_prob"] = round(p.falling_prob, 3)
        else:
            d["pose"] = None
        if p is not None and p.win_valid:    # window leg (sustained down-state)
            d["down"] = bool(p.down)
            d["h_s_cm"] = p.h_s_cm
        else:
            d["down"] = None
    if tg:
        zsm = sc.get("z_smooth")
        zval = zsm if zsm is not None else tg[0]["z"]
        z_cm = h_cm(sc.get("y0") if sc.get("y0") is not None else tg[0]["y"], zval)
        src = f"tid{tg[0]['tid']}"
    elif pts:
        z_cm = float(_np.percentile([h_cm(p[1], p[2]) for p in pts], 90)); src = "pts"
    else:
        z_cm = None; src = None
    # raw diagnostics: is the TRACK Z floating (per fall-design ~1m blur), or sane?
    diag = None
    if tg:
        diag = {"raw_y": tg[0]["y"], "raw_z": tg[0]["z"]}   # radar-frame (untilted)
    if pts:
        pzs = [p[2] for p in pts]                            # point-cloud radar-frame Z
        phs = [h_cm(p[1], p[2]) for p in pts]                # per-point world height cm
        diag = (diag or {})
        diag.update(pz_min=round(min(pzs), 2), pz_max=round(max(pzs), 2),
                    ph_min=round(min(phs)), ph_max=round(max(phs)), npts=len(pts))
    # BLOCK-PERSON: voxelize the accumulated 3001 minor point cloud in the ROOM frame
    # (20 cm, SNR-weighted). world Z=height, Y=ground range, X=lateral.
    # 3001 point cloud, PER TRACK: assign points near each track, box them, tag by
    # track index (for per-track colour). Points not near ANY track (stray/background
    # discrete points) are DROPPED — not shown, not boxed.
    boxes = []; pc_pts = []
    pcx = sc.get("pc_xyz")
    if pcx is not None and len(pcx) and tg:
        px, py, pz = pcx[:, 0], pcx[:, 1], pcx[:, 2]
        wz = mount_m + pz * math.cos(th) - py * math.sin(th)   # world height
        wy = py * math.cos(th) + pz * math.sin(th)             # world ground range
        from scipy.spatial import cKDTree
        from scipy.sparse import coo_matrix
        from scipy.sparse.csgraph import connected_components
        P = _np.stack([px, wy, wz], axis=1)
        # discrete-point removal: drop points whose nearest-neighbour > 5x the mean.
        if len(P) >= 4:
            nn = cKDTree(P).query(P, k=2)[0][:, 1]
            keep = nn <= 5.0 * float(nn.mean())
            px, py, wy, wz, P = px[keep], py[keep], wy[keep], wz[keep], P[keep]
        # CLUSTER by connectivity: points closer than EPS chain into one cluster; a gap
        # bigger than EPS (e.g. two people ~1 m apart) splits them into separate boxes.
        EPS = 0.4
        n = len(P)
        if n >= 1:
            pr = cKDTree(P).query_pairs(EPS, output_type='ndarray')
            if len(pr):
                r = _np.concatenate([pr[:, 0], pr[:, 1]]); c = _np.concatenate([pr[:, 1], pr[:, 0]])
                _, labels = connected_components(
                    coo_matrix((_np.ones(len(r)), (r, c)), shape=(n, n)), directed=False)
            else:
                labels = _np.arange(n)
        else:
            labels = _np.zeros(0, int)
        q = lambda a: (round(float(_np.percentile(a, 5)), 2),
                       round(float(_np.percentile(a, 95)), 2))
        tx = _np.array([t["x"] for t in tg]); ty = _np.array([t["y"] for t in tg])
        bytid = {}                                        # ti -> ONE merged box per track
        for lab in _np.unique(labels):
            m = labels == lab
            if int(m.sum()) < 4:                          # drop tiny clusters (noise)
                continue
            cx = float(px[m].mean()); cy = float(py[m].mean())   # radar-frame centroid
            d2 = (tx - cx) ** 2 + (ty - cy) ** 2; ti = int(d2.argmin())
            if float(d2[ti]) > 0.8 ** 2:                  # cluster not near any track -> skip
                continue
            x0, x1 = q(px[m]); y0, y1 = q(wy[m]); z0, z1 = q(wz[m])
            below = int((wz[m] < FLOOR_Z).sum()); tot = int(m.sum())    # cloud floor-band count
            b = bytid.get(ti)
            if b is None:
                bytid[ti] = {"tid": int(tg[ti]["tid"]), "ti": ti, "x0": x0, "x1": x1,
                             "y0": y0, "y1": y1, "z0": z0, "z1": z1, "n": tot, "_below": below}
            else:                                         # MERGE same-track fragments: a body
                b["x0"] = min(b["x0"], x0); b["x1"] = max(b["x1"], x1)   # split by an EPS gap
                b["y0"] = min(b["y0"], y0); b["y1"] = max(b["y1"], y1)   # (torso vs legs) is
                b["z0"] = min(b["z0"], z0); b["z1"] = max(b["z1"], z1)   # rejoined -> pose sees
                b["n"] += tot; b["_below"] += below                     # the WHOLE person
            idx = _np.where(m)[0]
            for i in idx[::max(1, len(idx) // 150)]:
                pc_pts.append([round(float(px[i]), 2), round(float(wy[i]), 2),
                               round(float(wz[i]), 2), ti])
        for b in bytid.values():                          # per TRACK (merged whole body)
            b["pose"] = _pose_of(b["x0"], b["x1"], b["y0"], b["y1"], b["z0"], b["z1"])
            b["floor_frac"] = round(b.pop("_below") / max(b["n"], 1), 2)   # 3001 cloud floor-band fraction
            boxes.append(b)
    # ---- fall via the falldet pipeline (Module 1 window + Module 3 clean) --------------
    # Server-side reference for the on-chip Phase-3 window trigger; also a fallback until
    # the firmware window ships. MLP leg = firmware Phase 2 (absent here). Red Fall needs
    # the cube second-check (RR + floor energy); without it the best verdict is Suspected.
    global _floor_pts
    prim = next((b for b in boxes if b["ti"] == 0), None) or \
        (max(boxes, key=lambda b: b["n"]) if boxes else None)
    primary_pose = prim["pose"] if prim else None
    prim_ffrac = float(prim.get("floor_frac", 0.0)) if prim else 0.0

    # rolling floor calibration H_g(x,y) from the scene cloud (world x, wy, wz)
    if pc_pts:
        _floor_pts.extend((p[0], p[1], p[2]) for p in pc_pts)
        _floor_pts = _floor_pts[-4000:]
        if len(_floor_pts) >= 200 and len(_floor.hg) == 0:
            _floor.fit(_floor_pts)

    # Module 1: sustained max-height window on the primary track's points.
    # Prefer the FIRMWARE window leg (TLV 321, true 10 fps sustain) when it's
    # emitting; else fall back to this server-side reference (~3 scene-calls/s).
    prim_pts = [(p[0], p[1], p[2]) for p in pc_pts if prim and p[3] == prim["ti"]]
    wout = _window.update(prim_pts)                    # keep running (floor calib + fallback)
    fw = poses.get(prim["tid"]) if prim else None      # firmware legs for the primary track
    if fw is not None and fw.win_valid:
        w_down, w_hs = bool(fw.down), fw.h_s_cm / 100.0
        w_src = "fw"
    else:
        w_down, w_hs = bool(wout["down"]), wout["h_s"]
        w_src = "srv"

    now = time.time()
    # REAL-PERSON gate: reject ghost tracks (jumpy, out-of-room, few points) so a low
    # ghost / stray floor clutter never triggers a fall or spams cubeQuery. DEBOUNCED:
    # the 3001-cloud point count near a STILL/lying track oscillates a lot (measured
    # 0..36 frame-to-frame), so an instantaneous n>=12 gate flickers and keeps resetting
    # the fall trigger + `run` counter -> the red Fall only ever caught a 1-frame sliver
    # the dashboard missed. The firmware winDown is ALREADY sustained (sustain=5), so we
    # only need real-person to ARM; hold it for REAL_GRACE_S through the n dips.
    real_inst = bool(prim and prim.get("n", 0) >= 12
                     and abs((prim["x0"] + prim["x1"]) / 2.0) < 3.5
                     and prim["y1"] < 5.5)
    if real_inst:
        _real_since[0] = now
    real_person = (now - _real_since[0]) < REAL_GRACE_S     # debounced
    down = bool(w_down and real_person)                # gated trigger

    # server-triggered cube fetch: only on a REAL down, rate-limited (1 per episode + a
    # ~4 s refresh) — not every scene call. Range from the cloud GROUND wy (_fall_range_bin).
    if down and not _cube_busy[0] and (now - _last_query_t[0]) > QUERY_REFRESH_S:
        rb = _fall_range_bin(sc)
        if rb is not None:
            _cube_busy[0] = True
            _last_query_t[0] = now
            threading.Thread(target=_fetch_cube_bg, args=(rb, prim_ffrac), daemon=True).start()

    # cube second-check evidence = RR + floor energy computed FROM the fetched 320 burst
    # (self-contained; a living body on the floor breathes -> RR -> red Fall; a dropped
    # object does not -> no red). A confirmed RR is HELD 12 s (the person stays down and
    # breathing; a single 3 s burst is < 1 breath cycle, so bridge the gaps) — the ~4 s
    # re-query keeps it refreshed while down.
    cube_ev = None
    if now - _cube_result["t"] < 12.0:
        cube_ev = {"rr": _cube_result["rr"], "floor_frac": _cube_result["floor_frac"]}

    # MLP leg from the firmware (Phase 2): falling motion + pose. None until its
    # 8-frame window fills. OR-fused with the window leg inside the cleaner.
    mlp_out = ({"pose": fw.label, "falling_p": fw.falling_prob}
               if (fw is not None and fw.valid) else None)
    dec = _cleaner.decide({"down": down, "h_s": w_hs}, mlp_out, cube=cube_ev, geom=None)
    fall_state = "fall" if dec["fall"] else ("suspected" if (dec["suspected"] or dec["trigger"]) else "none")
    # LATCH a confirmed red Fall so it stays visible on the dashboard for FALL_HOLD_S even
    # after the person stirs/gets up (a ~6 s red that clears the instant they move is easy
    # to miss). Cleared by GET /api/fall/reset.
    if dec["fall"]:
        _fall_latch_until[0] = now + FALL_HOLD_S
    if now < _fall_latch_until[0]:
        fall_state = "fall"
    # DIAG: whenever a fall trigger is active, log the full gate breakdown so a missed
    # red-Fall can be pinned to the exact failing gate. Goes to stdout AND
    # record/fall_debug.log (so it can be read back without copy-paste). Remove once tuned.
    if down or dec["trigger"] or cube_ev is not None:
        cr = _cube_result
        _line = (f"[fall] {fall_state:9s} down={int(down)}(w={int(w_down)}/{w_src},"
                 f"real_inst={int(real_inst)},real={int(real_person)}) "
                 f"hs={w_hs if w_hs is None else round(w_hs,2)} prim=tid{prim['tid'] if prim else '-'} "
                 f"n={prim.get('n') if prim else '-'} pffrac={prim_ffrac:.2f} busy={int(_cube_busy[0])} "
                 f"cube_res(rr={cr['rr']},str={cr['strength']},ffrac={cr['floor_frac']},age={now-cr['t']:.1f}s) "
                 f"run={_cleaner.run} conf={dec['confidence']} reason={dec['reason']} cleaned={dec['cleaned']}")
        print(_line, flush=True)
        try:
            with open(os.path.join(os.path.dirname(__file__), "..", "record", "fall_debug.log"), "a") as _fh:
                _fh.write(_line + "\n")
        except Exception:
            pass
    return {"live": True, "points": pts, "targets": tg,
            "height_cm": None if z_cm is None else round(z_cm),
            "src": src, "cube_entries": int(sc.get("n_cube", 0)),
            "mount_cm": round(mount_cm), "tilt_deg": TILT, "diag": diag,
            "boxes": boxes, "pc": pc_pts, "fall_state": fall_state,
            "fall_p": dec["confidence"], "primary_pose": primary_pose,
            "fall_ev": {"window": bool(w_down), "real": real_person, "win_src": w_src,
                        "h_s": (round(w_hs, 2) if w_hs is not None else None),
                        "reason": dec["reason"], "cleaned": dec["cleaned"],
                        "cube": (cube_ev is not None),
                        "rr": (cube_ev.get("rr") if cube_ev else None),
                        "floor_frac": prim_ffrac, "floor_cells": len(_floor.hg)},
            "elev_acc_deg": ELEV_ACC_DEG,
            "age_s": round(time.time() - sc.get("t", 0), 1)}


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, body, ctype="application/json"):
        try:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body if isinstance(body, bytes) else body.encode())
        except (BrokenPipeError, ConnectionResetError):
            pass   # client (browser poll) closed mid-response — harmless

    def log_message(self, *a):
        pass       # quiet the per-request access log

    def do_GET(self):
        u = urlparse(self.path)
        if u.path in ("/", "/index.html"):
            with open(os.path.join(HERE, "dashboard.html"), "rb") as f:
                return self._send(200, f.read(), "text/html; charset=utf-8")
        if u.path == "/api/meta":
            return self._send(200, json.dumps(_meta))
        if u.path == "/api/fall/reset":              # clear a latched red Fall
            _fall_latch_until[0] = 0.0
            return self._send(200, json.dumps({"fall_reset": True}))
        if u.path == "/api/scene":
            try:
                return self._send(200, json.dumps(_scene()))
            except Exception as e:
                return self._send(200, json.dumps({"live": False, "error": str(e)}))
        if u.path == "/api/cube":                    # server-triggered cube fetch (fall 2nd-check)
            q = parse_qs(u.query)
            rb = int(q.get("bin", [36])[0]); n = int(q.get("n", [30])[0]); hw = int(q.get("hw", [3])[0])
            if not hasattr(_src, "request_cube"):
                return self._send(200, json.dumps({"error": "source has no request_cube (not live)"}))
            ents = _src.request_cube(rb, n_frames=n, half_win=hw)
            bins = sorted({int(e.range_bin) for e in ents})
            return self._send(200, json.dumps({"bin": rb, "half_win": hw, "n_frames": n,
                                                "entries": len(ents), "range_bins": bins,
                                                "n_ant": (len(ents[0].vec) if ents else 0)}))
        if u.path == "/api/state":
            q = parse_qs(u.query)
            bl = int(q["bin_lo"][0]) if "bin_lo" in q else None
            bh = int(q["bin_hi"][0]) if "bin_hi" in q else None
            try:
                return self._send(200, json.dumps(_state(bl, bh)))
            except Exception as e:
                return self._send(200, json.dumps({"error": str(e)}))
        if u.path in ("/api/rec/on", "/api/rec/off"):
            # SAVE switch: the stream/serve keep running; only disk persistence
            # toggles. ON => 5-min rolling files (0-4,5-9,...) unchanged. OFF =>
            # stop immediately, flushing the current partial (last record = now).
            on = u.path.endswith("/on")
            r = _src.rec_set(on) if hasattr(_src, "rec_set") else {"error": "not a live source"}
            return self._send(200, json.dumps(r))
        if u.path == "/api/rec/status":
            r = _src.rec_status() if hasattr(_src, "rec_status") else {"saving": False}
            return self._send(200, json.dumps(r))
        if u.path == "/api/quit":
            # graceful programmatic shutdown (== Ctrl+C): flush + release the
            # read-only serial attach without stopping the sensor.
            self._send(200, json.dumps({"quitting": True}))
            if _shutdown:
                threading.Thread(target=_shutdown, daemon=True).start()
            return
        self._send(404, "{}")

    def log_message(self, *a):
        pass                                    # quiet


def main():
    global _src, _meta, TILT, MOUNT
    argv = sys.argv[1:]
    for flag, setter in (("--tilt", "TILT"), ("--mount", "MOUNT")):
        if flag in argv:
            i = argv.index(flag)
            globals()[setter] = float(argv[i + 1]); del argv[i:i + 2]
    # Recording is available as part of the live service but the WRITE switch starts
    # OFF (see --save-on below). Custom file name via --record NAME; --no-record
    # disables the recorder entirely (WRITE btn shows unavailable).
    rec_name = "live"
    if "--record" in argv:
        i = argv.index("--record"); rec_name = argv[i + 1]; del argv[i:i + 2]
    no_record = "--no-record" in argv
    if no_record:
        argv.remove("--no-record")
    # Recording (WRITE) is OFF by default — stream/serve only until the user turns
    # it on (dashboard WRITE btn or /api/rec/on). Use --save-on to start recording
    # immediately. (--save-off still accepted as a no-op = the default.)
    save_on = "--save-on" in argv
    if save_on:
        argv.remove("--save-on")
    if "--save-off" in argv:
        argv.remove("--save-off")
    spec = argv[0] if argv else "live"
    port = int(argv[1]) if len(argv) > 1 else 8765
    record_prefix = None
    if spec == "live" and not no_record:
        # recordings ALWAYS land in pc/record/ (raw, gitignored) as
        # pc/record/<name>_<5min-stamp>.npz; cp what validation needs into pc/case/.
        rec_dir = os.path.join(os.path.dirname(HERE), "record")
        os.makedirs(rec_dir, exist_ok=True)
        record_prefix = rec_name if os.path.sep in rec_name else os.path.join(rec_dir, rec_name)
    _src = make_source(spec, record_prefix=record_prefix, save_on=save_on)
    _src.start()
    # live source needs a moment for the reader thread to see the first frames
    # (bins are empty until then); wait so measurable_range has a range to report.
    for _ in range(150):
        m = _src.meta()
        if m["bins"]:
            break
        time.sleep(0.1)
    m = _src.meta()
    if not m["bins"]:
        print("WARN: no radar frames yet (sensor streaming? ports free?). Serving anyway.",
              flush=True)
    _meta = dict(source=m, win_s=WIN_S, mount_calibrated=(TILT is not None and MOUNT is not None),
                 tilt_deg=TILT, h_mount=MOUNT,
                 range=pipe.measurable_range(m["bins"], m["dr"]) if m["bins"] else None)
    rec = f"  recording 5-min files -> {record_prefix}_*.npz" if record_prefix else ""
    print(f"source={m['kind']} ({m['name']})  bins {min(m['bins']) if m['bins'] else '?'}-"
          f"{max(m['bins']) if m['bins'] else '?'}  serving http://127.0.0.1:{port}{rec}", flush=True)
    srv = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    # graceful stop on Ctrl-C AND SIGTERM (plain `pkill`) so the read-only serial
    # attach is released cleanly and never wedges the firmware. NOTE: `kill -9`
    # (SIGKILL) is uncatchable and CAN wedge the UART -> then power-cycle the radar.
    import signal
    global _shutdown
    def _graceful(*_a):
        _src.stop(); srv.shutdown()
    _shutdown = _graceful                                 # exposed via /api/quit
    signal.signal(signal.SIGINT, _graceful)
    signal.signal(signal.SIGTERM, _graceful)
    try:
        srv.serve_forever()
    finally:
        _src.stop()


if __name__ == "__main__":
    main()
