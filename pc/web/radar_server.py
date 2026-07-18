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
from falldet.floor_track import FloorTracker

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
QUERY_REFRESH_S = 12.0       # min seconds between cubeQuery bursts while down. A 60-frame
                             # burst is ~6 s -> 6 s on + 6 s idle = 50% DATA-UART duty. The
                             # firmware wedges when 320 floods the DATA UART for MINUTES; a
                             # single 6 s burst is safe, but back-to-back 6 s bursts at ~60%
                             # duty accumulate and wedge it. The fix is a BIGGER idle gap
                             # (>=burst), NOT a tiny 0.4 s micro-gap (that raises duty ->
                             # worse). See [[fall-detection-design]] / the wedge in fall log.
STALE_GATE_S = 3.0           # NEVER cubeQuery when the scene is this stale: the sensor is
                             # wedged/stalled, so 320 bursts hit a DEAD firmware -- useless,
                             # and they can keep it from recovering. Gate every probe on this.
MAX_CUBE_BURSTS = 3          # HARD cap on cubeQuery bursts PER fall/lost episode -- NOT reset by
                             # finding RR. A fall grabs cube a FEW times to verify person-vs-
                             # object (RR + the fused MLP), then STOPS. The old "reset on RR"
                             # was the wedge cause: a breathing body returns RR every burst ->
                             # counter never advances -> 320 floods the DATA UART for the whole
                             # (100 s+) fall -> firmware WEDGES. Strictly bounded now: <=3 bursts
                             # (~18 s of cube) per episode, then silent; the fall stays latched
                             # via the window / floor-fall / sustained legs, no more cube needed.
FALL_FFRAC_MIN = 0.15        # sustained-down -> red Fall ONLY if the cloud is really below the
                             # floor line. A ~0.45 m furniture cluster (floor_frac~0.02) must
                             # NOT latch a permanent fall (that was the 22-minute false latch).
_cube_episode = [0]          # cube bursts fired in the CURRENT physical fall episode, UNIFIED
                             # across down-probe + lost-probe. NOT per floor-track id -- those
                             # CHURN with fragmentation, so a per-id cap leaked (each new id got
                             # a fresh 3-burst budget -> dozens of bursts -> re-wedge). Hard cap.
_cube_last_active = [0.0]    # last wall time a fall was active (down / floor-fall / latched)
CUBE_RESET_S = 5.0           # after this long with NO active fall (person up & gone), the cube
                             # episode budget resets -> the next distinct fall gets its own bursts
_real_since = [0.0]          # last time the real-person gate was instantaneously true
REAL_GRACE_S = 2.0           # hold real-person through brief point-count dips (see below)
_fall_latch_until = [0.0]    # a confirmed red Fall LATCHES the display until this wall time
FALL_HOLD_S = 30.0           # keep showing red Fall this long after the last confirmation
                             # (a caregiver must SEE it; it must not clear when the person
                             # stirs/gets up). Cleared by /api/fall/reset or a clear recovery.
_recover_since = [0.0]       # wall time the person has been continuously upright & recovered
RECOVER_S = 2.0              # a CLEAR recovery this long CLEARS the latch early: a tracked real
                             # person whose WHOLE-cloud median height is up (RECOVER_ZMED). Uses
                             # the robust cloud centroid, NOT the per-box pose/ffrac -- those
                             # thrash as the walking-away body FRAGMENTS (000000: ffrac 0<->1,
                             # pose SIT/STAND/LIE flicker, a stray fragment even re-latched the
                             # red). The centroid stays ~+0.7 m once up. Only clears when up.
RECOVER_ZMED = 0.4           # whole-cloud median world height above this = the mass is UP (a
                             # lying body is -0.2..-0.5; standing/sitting is +0.5..+0.9).
CANCEL_R = 0.5               # 30 s monitor: if a GTRACK track stands UP within this radius of
                             # where the body fell (the person got up unaided at the fall spot),
                             # DISCARD the fall -- a stumble that self-recovers is not an
                             # emergency. Faster + more specific than the cloud-centroid clear.
_last_low_xy = [None]        # radar-frame (x, y) of the most recent below-floor cloud mass
_down_since = [0.0]          # wall time the current sustained-down episode started (0 = none)
_down_last = [0.0]           # last wall time `down` was true (to bridge brief flicker gaps)
FALL_SUSTAIN_S = 10.0        # sustained window-down this long (real person, can't get up) ->
                             # red Fall EVEN without cube RR. Catches a kid / weak-breathing
                             # body the cube-RR second-check can't lock onto. The cube still
                             # gets its ~6 s first; this is the fallback for real sustained down.
DOWN_GAP_S = 2.5             # `down` may drop out this long without resetting the sustain timer
# ---- Track-INDEPENDENT floor-fall leg -------------------------------------------------
# window/real/cube all hang on `prim` (a GTRACK box or a FloorTracker person id). A spread
# lying body FRAGMENTS into churning orphan bits, so FloorTracker can't hold its identity
# (id churns, person stays False) and prim=None -> the whole pipeline goes dark on a clean
# floor fall (231000-A @2.5m: 500 pts, 100% below the floor line, 60 s -> MISSED). This leg
# reads the AGGREGATE below-floor cloud directly, armed by a GTRACK death nearby (a person
# went DOWN here, not walked past), and stays sticky while the below-floor blob persists --
# so it also holds far falls (222500-mid / 231500-#2 @4.5m) through the cloud collapse that
# drops the n>=12 real-person gate. Furniture never arms it (no one ever fell there).
FALL_LEG_MIN_PTS = 12        # min below-floor points to call it a body (rejects standing feet)
FALL_LEG_FRAC = 0.8          # the local region must be >=this fraction below the floor band. A
                             # LYING body is ~0.9-1.0; a SITTER's torso is above (frac ~0.4-0.6)
                             # so 0.8 rejects sitting/standing-passing-through, accepts lying.
FALL_LEG_ZMED = 0.15         # AND the local region's MEDIAN world height must be below this: the
                             # body mass is truly on the floor. FLOOR_Z=0.4 alone is knee-height
                             # -- a STANDING/walking person at range has a big below-0.4 leg cloud
                             # (231000 100-139s: z_med +0.2..+0.9 yet below_n 18-87) and dragged
                             # the armed region across the room. A lying body's z_med is -0.2..-0.5.
FALL_REGION_M = 1.6          # region radius (a lying body spreads ~1.5 m; arm + sustain gate)
FALL_UPRIGHT_M = 0.4         # a LIVE GTRACK track in the region whose world height is above this
                             # = a person standing / getting up here, NOT a fallen body -> VETO
                             # the leg. Kills the "got up from a prior lie" residual-cloud false
                             # trigger (231000 open: residual floor pts + churning track deaths
                             # armed floor_fall for 7 s -> hit sustained -> 30 s false red latch).
FALL_DEATH_S = 8.0           # a GTRACK track that died this recently near the blob = fell here
FALL_EXIT_GRACE_S = 3.0      # the below-floor blob may vanish this long before the leg disarms
_gtrack_prev = {}            # tid -> (x, y) last frame, for GTRACK-death detection
_fall_deaths = []            # [(t, x, y)] recent GTRACK deaths (a person may have gone down)
_fall_region = {"since": 0.0, "last": 0.0, "x": 0.0, "y": 0.0}  # sticky armed fall region
# Lost-track RR probe: when GTRACK drops a still person's track (FloorTracker inherits it),
# actively cubeQuery that spot to get RR -- confirms a living body (vs furniture) and shows
# the RR for a sitting/fallen still person. WAIT 2 s first: most track losses are brief
# flickers that re-acquire, and we must not spend a ~6 s cube burst on those.
LOST_WAIT_S = 2.0            # a track must stay lost this long (not a flicker) before probing
LOST_QUERY_REFRESH_S = 12.0 # min seconds between lost-probe cubeQuery bursts per track (50% duty)
_lost_since = {}            # floor-track id -> wall time it became inherited (lost); cleared on re-acquire
_lost_query_t = {}          # floor-track id -> last lost-probe cubeQuery wall time


_cube_result = {"rr": None, "strength": 0.0, "t": 0.0, "floor_frac": 0.0, "bin": None}  # latest cube 2nd-check
# FloorTracker: gives a GTRACK-dropped STILL body (fallen OR just sitting motionless) a
# continuous track_id from its point cloud (inherits the lost tid; furniture rejected via
# no-history + no-RR). death_grace_s=30 because a sitting person's GTRACK track can flicker
# off for 15+ s -- a spot tracked within the last 30 s is still that person, not furniture.
# See falldet/floor_track.py.
_floor_tracker = FloorTracker(death_grace_s=30.0)


def _rr_from_cube(entries, fps=10.0):
    """Breathing RR from a fetched 320 burst, using the SAME estimator as the vitals
    path -- ONE RR module (bcg_vitals.demod_channels -> estimate_rr), not a second
    ad-hoc FFT. Per range bin, stack the 16-antenna zero-Doppler vectors into a (T, nAnt)
    slow-time cube; demod_channels coherently combines the antennas and phase-demodulates
    to a mm-displacement channel; estimate_rr picks the SQI-top bins and takes the median
    breathing-band peak. Breathing 'present' (the fall gate) = the top bins AGREE on the
    RR (low spread); a dropped object gives scattered per-bin peaks. Returns
    (rr_bpm | None, strength 0..1)."""
    import numpy as _np
    from collections import defaultdict
    from bcg_vitals import demod_channels, estimate_rr
    if not entries:
        return None, 0.0
    byb = defaultdict(list)
    for e in entries:
        byb[int(e.range_bin)].append(_np.asarray(e.vec, complex))   # 16-ant vec per frame
    bins = [b for b, s in byb.items() if len(s) >= 12]
    if not bins:
        return None, 0.0
    T = min(len(byb[b]) for b in bins)                  # align lengths across bins
    C = [_np.stack(byb[b][:T]) for b in bins]           # C[i] = (T, nAnt) per bin
    chans = demod_channels(C, bins)                     # (nbin, T) mm displacement
    # interp=True: parabolic sub-bin peak so RR isn't quantized to the window's FFT grid
    # (a 6 s burst -> 10 rpm bins made RR look stuck at 10/20). A longer burst still helps
    # SNR + resolution; interp removes the quantization at any length.
    rr, _f0, spread, per_bin = estimate_rr(chans, fps, interp=True)
    if rr is None or not per_bin:
        return None, 0.0
    strength = max(0.0, 1.0 - spread / 12.0)            # bins agree (low spread) -> confident
    return (round(rr, 1) if strength > 0.2 else None), round(strength, 2)


def _fetch_cube_bg(range_bin, floor_frac, n_frames=60):
    """Background: burst 320 at range_bin, then compute RR from it (the cube second-check).
    n_frames sets the integration window: 60 (~6s, ~2 breaths) for a quick fall confirm;
    the lost-track/still-person probe uses a LONGER window (~15s) -- a still body is not
    time-limited, and longer coherent integration lifts weak breathing above the ~1um
    noise floor (SNR ~ sqrt(T)) AND sharpens RR resolution. Non-blocking for /api/scene."""
    try:
        if hasattr(_src, "request_cube"):
            ents = _src.request_cube(range_bin, n_frames=n_frames, half_win=3,
                                     timeout=n_frames / 10.0 + 3.0)
            rr, strength = _rr_from_cube(ents)
            _cube_result.update(rr=rr, strength=strength, t=time.time(),
                                floor_frac=round(float(floor_frac), 2), bin=int(range_bin))
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
    # UNIFIED cube budget: reset the per-episode burst count once the scene has been quiet
    # (no active fall) for CUBE_RESET_S -> the next distinct fall gets its own MAX_CUBE_BURSTS.
    if time.time() - _cube_last_active[0] > CUBE_RESET_S:
        _cube_episode[0] = 0
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
    cloud_wz_med = None          # robust whole-cloud median world height (recovery / up signal)
    pcx = sc.get("pc_xyz")
    # NOTE: no "and tg" here -- the block MUST run even with zero GTRACK tracks, because a
    # fallen still body is exactly when GTRACK drops every track yet the 3001 cloud persists.
    # Gating on tg was why the scene went blank after a fall (no cloud, no id near the floor).
    if pcx is not None and len(pcx):
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
        if len(wz):
            cloud_wz_med = float(_np.median(wz))       # robust up/down signal (all points)
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
        orphans = []                                       # clusters near NO GTRACK track (any height)
        for lab in _np.unique(labels):
            m = labels == lab
            if int(m.sum()) < 4:                          # drop tiny clusters (noise)
                continue
            cx = float(px[m].mean()); cy = float(py[m].mean())   # radar-frame centroid
            d2 = (tx - cx) ** 2 + (ty - cy) ** 2
            ti = int(d2.argmin()) if len(tx) else -1
            if ti < 0 or float(d2[ti]) > 0.8 ** 2:        # cluster not near any GTRACK track
                # Orphan blob GTRACK left unassigned. Keep it (ANY height, not just low):
                # a STILL person -- sitting OR fallen -- gets dropped by GTRACK's motion-based
                # allocator, and the FloorTracker below re-attaches the person's identity
                # (inherited from the flickering track). Low-only would miss a sitting body
                # (world-z ~0.65) and make it flicker in/out of the scene.
                x0o, x1o = q(px[m]); y0o, y1o = q(wy[m]); z0o, z1o = q(wz[m])
                io = _np.where(m)[0]
                orphans.append({
                    "cx": cx, "cy": cy, "n": int(m.sum()),
                    "x0": x0o, "x1": x1o, "y0": y0o, "y1": y1o, "z0": z0o, "z1": z1o,
                    "below": int((wz[m] < FLOOR_Z).sum()),
                    "pts": [[round(float(px[i]), 2), round(float(wy[i]), 2),
                             round(float(wz[i]), 2)] for i in io[::max(1, len(io) // 60)]]})
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

        # ---- FloorTracker: a fallen body GTRACK dropped keeps a CONTINUOUS id ----------
        # The low orphan blobs above are the fallen person's cloud that GTRACK abandoned
        # (it allocates from motion; a still body reads as furniture). Give each a track_id:
        # inherited from the just-lost GTRACK tid ("track lost + floor blob here" = fell), or
        # a fresh negative id. DISPLAY is decoupled from the alert: the floor cloud is ALWAYS
        # drawn (so it never vanishes after a fall); the person-vs-furniture call (inherited
        # real tid OR cube RR) only gates whether it gets a fall BOX/alert.
        def _rr_at(cx, cy):
            if _cube_result["rr"] in (None, 0) or _cube_result.get("bin") is None:
                return False
            return abs(math.hypot(cx, cy) / RANGE_STEP - _cube_result["bin"]) <= 4
        gtracks = {int(t["tid"]): (float(t["x"]), float(t["y"])) for t in tg}
        # GTRACK-death memory for the floor-fall leg: a tid present last frame, gone now, went
        # DOWN here (a still body GTRACK drops), unless it walked off (its cloud goes with it).
        global _gtrack_prev, _fall_deaths
        _fdnow = time.time()
        for _tid, _xy in _gtrack_prev.items():
            if _tid not in gtracks:
                _fall_deaths.append((_fdnow, _xy[0], _xy[1]))
        _gtrack_prev = dict(gtracks)
        _fall_deaths = [(t, x, y) for (t, x, y) in _fall_deaths if _fdnow - t < FALL_DEATH_S]
        ftracks = _floor_tracker.update(time.time(), gtracks,
                                        [(o["cx"], o["cy"], o["n"]) for o in orphans],
                                        rr_at=_rr_at)
        ft_by_xy = {(round(t.x, 3), round(t.y, 3)): t for t in ftracks if t.age == 0}
        boxed = {b["tid"] for b in boxes}
        for o in orphans:
            ftk = ft_by_xy.get((round(o["cx"], 3), round(o["cy"], 3)))
            fid = int(ftk.id) if ftk else -999            # -999 = unassigned floor cloud
            for p in o["pts"]:                            # ALWAYS draw the floor cloud
                pc_pts.append([p[0], p[1], p[2], fid])
            if ftk and ftk.person and ftk.id not in boxed:   # person -> add a fall box/alert
                fb = {"tid": int(ftk.id), "ti": int(ftk.id), "x0": o["x0"], "x1": o["x1"],
                      "y0": o["y0"], "y1": o["y1"], "z0": o["z0"], "z1": o["z1"], "n": o["n"],
                      "floor_frac": round(o["below"] / max(o["n"], 1), 2), "floor_src": ftk.source}
                fb["pose"] = _pose_of(fb["x0"], fb["x1"], fb["y0"], fb["y1"], fb["z0"], fb["z1"])
                boxes.append(fb)
                boxed.add(ftk.id)

        # ---- lost-track RR probe: a GTRACK-dropped person (inherited) that has stayed lost
        # past the flicker window (LOST_WAIT_S) gets an active cubeQuery at its range, to
        # read RR (confirm living body + show it). GTRACK re-acquiring clears the timer, so a
        # brief flicker never triggers a burst.
        _now = time.time()
        alive_ids = set()
        for ftk in ftracks:
            alive_ids.add(ftk.id)
            # a LIVE GTRACK track near this floor-track = the person is TRACKED, not lost.
            # FloorTracker fragments a spread body into churning orphan bits that briefly
            # inherit an old tid, so a NORMALLY-tracked (even standing) person spawns phantom
            # "inherited" floor-tracks. lost-probing those floods 320 forever -- and because
            # the person breathes, the RR reset above keeps the dry-cap from ever stopping it
            # -> re-wedge. So NEVER lost-probe while GTRACK still has a track nearby.
            near_live = any((gx - ftk.x) ** 2 + (gy - ftk.y) ** 2 < 1.5 ** 2
                            for gx, gy in gtracks.values())
            if ftk.source == "gtrack" or near_live:
                _lost_since.pop(ftk.id, None)                # GTRACK has it -> not lost
            elif ftk.person and ftk.source == "inherited":
                _lost_since.setdefault(ftk.id, _now)         # mark when it became lost
                # NO reset-on-RR here: a breathing body returns RR every burst, and resetting
                # the counter kept it probing forever -> 320 flood -> WEDGE. Hard cap instead.
                # GATED: sensor FRESH (never flood a wedged firmware) AND under the per-episode
                # HARD burst cap (verify person-vs-object a few times, then STOP).
                if (_now - _lost_since[ftk.id] >= LOST_WAIT_S
                        and _now - _lost_query_t.get(ftk.id, 0) > LOST_QUERY_REFRESH_S
                        and not _cube_busy[0]
                        and (_now - sc.get("t", _now)) < STALE_GATE_S
                        and _cube_episode[0] < MAX_CUBE_BURSTS):
                    rb = int(round(math.hypot(ftk.x, ftk.y) / RANGE_STEP))
                    _cube_busy[0] = True
                    _lost_query_t[ftk.id] = _now
                    _cube_episode[0] += 1
                    # 60 frames (~6s): a single LONG cubeQuery (150/~15s) WEDGES the firmware
                    # -- the sustained 320 flood over DATA UART kills it ([NO-Done] + no frames).
                    # Long integration for a still body must come from stacking SHORT bursts
                    # into a server-side sliding buffer (option A), not one long burst.
                    threading.Thread(target=_fetch_cube_bg, args=(rb, 1.0, 60),
                                     daemon=True).start()
        for d in [i for i in _lost_since if i not in alive_ids]:   # forget gone tracks
            _lost_since.pop(d, None); _lost_query_t.pop(d, None)

        # ---- floor-fall leg: ARM from the aggregate below-floor cloud (track-independent) ---
        below = wz < FLOOR_Z
        if int(below.sum()) >= FALL_LEG_MIN_PTS:
            bx = float(_np.median(px[below])); by = float(_np.median(py[below]))  # radar-frame
            near = ((px - bx) ** 2 + (py - by) ** 2) < FALL_REGION_M ** 2
            reg_below = int((near & below).sum()); reg_tot = int(near.sum())
            reg_med_z = float(_np.median(wz[near])) if reg_tot else 1.0   # local mass height
            # VETO: a LIVE GTRACK track in the region that is UPRIGHT (world height above the
            # fall band) = a person standing / getting up here, not a fallen body.
            veto_up = any(((t["x"] - bx) ** 2 + (t["y"] - by) ** 2 < FALL_REGION_M ** 2)
                          and (mount_m + t["z"] * math.cos(th) - t["y"] * math.sin(th))
                          > FALL_UPRIGHT_M for t in tg)
            if veto_up:
                _fall_region["since"] = 0.0        # someone is upright here -> drop it now
            elif (reg_below >= FALL_LEG_MIN_PTS and reg_below >= FALL_LEG_FRAC * reg_tot
                    and reg_med_z < FALL_LEG_ZMED):
                # a substantial, floor-DOMINATED blob. Arm ONLY if a person went DOWN here: a
                # GTRACK track died nearby recently (a still body drops off GTRACK). NOT on a
                # live track nearby -- a standing/sitting tracked person must never arm it, and
                # when GTRACK holds the track the normal window/real pipeline handles it; this
                # leg exists for exactly the prim=None case. `armed_here` = sticky sustain.
                near_death = any(_now - dt < FALL_DEATH_S
                                 and (dx - bx) ** 2 + (dy - by) ** 2 < FALL_REGION_M ** 2
                                 for (dt, dx, dy) in _fall_deaths)
                armed_here = (_fall_region["since"] > 0.0 and
                              (bx - _fall_region["x"]) ** 2 + (by - _fall_region["y"]) ** 2
                              < FALL_REGION_M ** 2)
                if near_death or armed_here:
                    if _fall_region["since"] == 0.0:
                        _fall_region["since"] = _now
                    _fall_region["last"] = _now
                    if near_death:                 # (re)anchor ONLY on a fresh fall here;
                        _fall_region["x"] = bx     # sticky sustain keeps the ORIGINAL anchor
                        _fall_region["y"] = by     # so the region can't WALK with a person who
                                                   # got up and moved off (the 2.5->4.6m drift).
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
    # floor-fall leg (armed above): disarm when the below-floor blob is gone (person got up /
    # moved off). It is track-INDEPENDENT, so it triggers `down` even when prim=None (GTRACK
    # dropped a still body and FloorTracker fragmented its identity) -> catches the clean floor
    # falls the prim pipeline misses, and sustains far falls through the cloud collapse.
    if _fall_region["since"] and (now - _fall_region["last"]) > FALL_EXIT_GRACE_S:
        _fall_region["since"] = 0.0
    floor_fall = _fall_region["since"] > 0.0
    down = bool((w_down and real_person) or floor_fall)     # gated trigger (+ track-free leg)

    # server-triggered cube fetch: only on a REAL down, rate-limited (50% duty refresh) — not
    # every scene call. Range from the cloud GROUND wy (_fall_range_bin). GATED on: sensor is
    # FRESH (never flood a wedged firmware) AND under the per-episode HARD burst cap
    # (MAX_CUBE_BURSTS: grab cube a few times to verify person-vs-object, then STOP -- an
    # unbounded refresh floods 320 for the whole fall and WEDGES the EVM).
    fresh = (now - sc.get("t", now)) < STALE_GATE_S
    if (down and not _cube_busy[0] and fresh
            and (now - _last_query_t[0]) > QUERY_REFRESH_S
            and _cube_episode[0] < MAX_CUBE_BURSTS):
        rb = _fall_range_bin(sc)
        if rb is not None:
            _cube_busy[0] = True
            _last_query_t[0] = now
            _cube_episode[0] += 1
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

    # SUSTAINED-DOWN escalation: track how long the person has continuously been down
    # (bridging brief DOWN_GAP_S flicker gaps). If down that long AND a real person, call it
    # a red Fall even if the cube never found breathing -- catches a kid / weak-breathing
    # body. The cube-RR path (above) still confirms faster for a clear adult.
    if down:
        if _down_since[0] == 0.0:
            _down_since[0] = now
        _down_last[0] = now
    elif _down_since[0] and (now - _down_last[0]) > DOWN_GAP_S:
        _down_since[0] = 0.0                       # down truly gone -> reset the sustain timer
    down_dur = (now - _down_since[0]) if _down_since[0] else 0.0
    # ffrac guard: a real fall puts the cloud BELOW the floor line (high floor_frac). A
    # ~0.45 m furniture cluster (floor_frac~0.02) reads "down" via the window leg but is NOT
    # on the floor -> must never latch a permanent fall (that was the 22-minute false latch).
    # floor_fall already IS a below-floor body, so it satisfies both the real-person and the
    # ffrac guard on its own (that is the whole point -- it carries the falls where n<12 / the
    # prim box is absent). OR it in for the sustained -> red escalation.
    sustained_fall = bool(_down_since[0] and down_dur >= FALL_SUSTAIN_S
                          and (real_person or floor_fall)
                          and (prim_ffrac >= FALL_FFRAC_MIN or floor_fall))
    if sustained_fall and not dec["fall"]:
        fall_state = "fall"
        dec["reason"] = list(dec.get("reason") or []) + [f"sustained{int(down_dur)}s"]

    # LATCH a confirmed red Fall so it stays visible on the dashboard for FALL_HOLD_S even
    # after the person stirs/gets up (a ~6 s red that clears the instant they move is easy
    # to miss). Cleared by GET /api/fall/reset or a sustained clear recovery below.
    # `cloud_up` = the WHOLE-cloud centroid is up (robust to the per-box fragmentation that
    # thrashes pose/ffrac as a body walks away). Don't RE-LATCH on a stray low fragment while
    # the mass is clearly up -- that was extending the red long after the person got up.
    cloud_up = cloud_wz_med is not None and cloud_wz_med > RECOVER_ZMED
    if (dec["fall"] or sustained_fall) and not cloud_up:
        _fall_latch_until[0] = now + FALL_HOLD_S
    # RECOVERY clears the latch early: a tracked real person whose cloud centroid is up, held
    # RECOVER_S -> they clearly got up and moved on. A still-fallen body's centroid stays low.
    if real_inst and cloud_up:
        if _recover_since[0] == 0.0:
            _recover_since[0] = now
        if now - _recover_since[0] >= RECOVER_S:
            _fall_latch_until[0] = 0.0
    else:
        _recover_since[0] = 0.0
    if now < _fall_latch_until[0]:
        fall_state = "fall"
    # DIAG: whenever a fall trigger is active, log the full gate breakdown so a missed
    # red-Fall can be pinned to the exact failing gate. Goes to stdout AND
    # record/fall_debug.log (so it can be read back without copy-paste). Remove once tuned.
    if down or dec["trigger"] or cube_ev is not None:
        cr = _cube_result
        _line = (f"[fall] {fall_state:9s} down={int(down)}(w={int(w_down)}/{w_src},"
                 f"real_inst={int(real_inst)},real={int(real_person)},floor={int(floor_fall)}) "
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
    # mark the cube episode active while a fall is in play, so the budget only resets after
    # CUBE_RESET_S of genuine quiet (person up & gone) -- see the reset at the top of _scene.
    if fall_state == "fall" or down or floor_fall:
        _cube_last_active[0] = now
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
                        "floor_fall": floor_fall,
                        "floor_frac": prim_ffrac, "floor_cells": len(_floor.hg)},
            "elev_acc_deg": ELEV_ACC_DEG,
            # cube RR (breathing from the fall cube second-check, SAME estimator as the
            # vitals RR: bcg_vitals demod_channels + estimate_rr). Surfaced top-level so the
            # dashboard shows it even when the vitals /api/state RR is idle (no still track).
            "cube_rr": _cube_result["rr"], "cube_rr_str": _cube_result["strength"],
            "cube_rr_age": (round(time.time() - _cube_result["t"], 1) if _cube_result["t"] else None),
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
        if u.path == "/api/replay/list":
            # npz recordings available to replay (case/ + record/), newest first.
            import glob
            root = os.path.join(HERE, "..")
            files = []
            for sub in ("case", "record"):
                for p in glob.glob(os.path.join(root, sub, "*.npz")):
                    try:
                        st = os.stat(p)
                    except OSError:
                        continue
                    files.append({"path": f"{sub}/{os.path.basename(p)}",
                                  "label": f"{sub}/{os.path.basename(p)}",
                                  "size": st.st_size, "mtime": st.st_mtime})
            files.sort(key=lambda f: f["mtime"], reverse=True)
            return self._send(200, json.dumps({"files": files}))
        if u.path == "/api/replay":
            # Run the npz back through the REAL fall pipeline in a SUBPROCESS (fall_replay.py
            # monkeypatches module globals -> must NOT run in this live process). Returns the
            # code-of-record fall verdict + timeline. file must sit under case/ or record/.
            import subprocess
            q = parse_qs(u.query)
            rel = (q.get("file", [""])[0]).replace("\\", "/")
            mnt = q.get("mount", [str(MOUNT if MOUNT is not None else 2.0)])[0]
            tlt = q.get("tilt", [str(TILT or 25.0)])[0]
            if (not rel or ".." in rel or rel.split("/")[0] not in ("case", "record")
                    or not rel.endswith(".npz")):
                return self._send(400, json.dumps({"error": "bad file (want case/*.npz or record/*.npz)"}))
            root = os.path.abspath(os.path.join(HERE, ".."))
            fp = os.path.abspath(os.path.join(root, rel))
            if not fp.startswith(root + os.sep) or not os.path.exists(fp):
                return self._send(404, json.dumps({"error": "file not found"}))
            try:
                out = subprocess.run(
                    [sys.executable, os.path.join(HERE, "fall_replay.py"), fp,
                     "--mount", str(mnt), "--tilt", str(tlt), "--json"],
                    capture_output=True, text=True, timeout=180, cwd=root)
                if out.returncode != 0:
                    return self._send(200, json.dumps({"error": "replay failed",
                                                       "stderr": out.stderr[-2000:]}))
                return self._send(200, out.stdout.strip() or "{}")
            except subprocess.TimeoutExpired:
                return self._send(200, json.dumps({"error": "replay timed out"}))
            except Exception as e:
                return self._send(200, json.dumps({"error": str(e)}))
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
