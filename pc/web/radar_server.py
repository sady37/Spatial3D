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
import radar_pipeline as pipe
from radar_source import make_source

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
TILT = 35.0          # radar DOWN-tilt from horizontal, deg
MOUNT = 2.0          # sensor height above floor, m
_cache = {"t": 0.0, "key": None, "state": None}
_lock = threading.Lock()


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
    tg = [{"tid": int(t.tid), "x": round(t.x, 3), "y": round(t.y, 3),
           "z": round(t.z, 3), "speed": round(t.speed, 2)} for t in tgts]
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
        for lab in _np.unique(labels):
            m = labels == lab
            if int(m.sum()) < 4:                          # drop tiny clusters (noise)
                continue
            cx = float(px[m].mean()); cy = float(py[m].mean())   # radar-frame centroid
            d2 = (tx - cx) ** 2 + (ty - cy) ** 2; ti = int(d2.argmin())
            if float(d2[ti]) > 0.8 ** 2:                  # cluster not near any track -> skip
                continue
            x0, x1 = q(px[m]); y0, y1 = q(wy[m]); z0, z1 = q(wz[m])
            boxes.append({"tid": int(tg[ti]["tid"]), "ti": ti, "x0": x0, "x1": x1,
                          "y0": y0, "y1": y1, "z0": z0, "z1": z1, "n": int(m.sum())})
            idx = _np.where(m)[0]
            for i in idx[::max(1, len(idx) // 150)]:
                pc_pts.append([round(float(px[i]), 2), round(float(wy[i]), 2),
                               round(float(wz[i]), 2), ti])
    return {"live": True, "points": pts, "targets": tg,
            "height_cm": None if z_cm is None else round(z_cm),
            "src": src, "cube_entries": int(sc.get("n_cube", 0)),
            "mount_cm": round(mount_cm), "tilt_deg": TILT, "diag": diag,
            "boxes": boxes, "pc": pc_pts,
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
        if u.path == "/api/scene":
            try:
                return self._send(200, json.dumps(_scene()))
            except Exception as e:
                return self._send(200, json.dumps({"live": False, "error": str(e)}))
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
