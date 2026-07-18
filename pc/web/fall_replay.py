"""Replay an npz scene recording through the REAL server fall pipeline.

The point: the fall verdict here is the LIVE CODE's (radar_server._scene), NOT an
ad-hoc re-implementation that drifts every time I re-analyse. We drive the actual
_scene() function frame-by-frame with:
  * a fake _src whose .scene() rebuilds each frame from the npz (tracks/poses/cloud),
  * the recorded per-frame timestamp as the clock (srv.time.time patched),
  * synchronous cube fetch (threading.Thread patched) that returns the RECORDED
    TLV-320 vectors at the queried range/time -> the real _rr_from_cube runs on them.

So every fall_state printed is exactly what the server would have decided live. Change
the server code, re-run this, and the count changes with the CODE, not with me.

Usage:  python3 web/fall_replay.py case/fall_222500.npz [--mount 2.0 --tilt 25] [--frames]
"""
import os, sys, math, argparse, types, json
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # pc/
import web.radar_server as srv
from spatial3d.tlv import Target, Pose


# ---- replay clock: srv uses `time.time()` everywhere in the fall path -------------
class _Clock:
    def __init__(self): self.t = 0.0
    def time(self): return self.t
    def sleep(self, *a): pass


# ---- synchronous cube fetch: run the target inline instead of on a thread ---------
class _SyncThread:
    def __init__(self, target=None, args=(), kwargs=None, daemon=None):
        self._t, self._a, self._k = target, args, (kwargs or {})
    def start(self):
        if self._t: self._t(*self._a, **self._k)


class _Sink:
    """Swallow _scene()'s per-frame [fall] diagnostic file-append during replay so it
    doesn't pollute the real record/fall_debug.log."""
    def write(self, *a): pass
    def __enter__(self): return self
    def __exit__(self, *a): return False


class FakeSource:
    """Feeds one recorded frame at a time to srv._scene(), and answers request_cube
    from the recorded TLV-320 burst at the queried range/time."""
    def __init__(self, d):
        self.d = d
        self.fi = 0
        self.nfr = int(d["ts"].shape[0])
        # per-frame index maps
        self.tf = d["t_frame"]; self.tid = d["t_tid"]
        self.tx = d["t_x"]; self.ty = d["t_y"]; self.tz = d["t_z"]
        self.tpose = d["t_pose"]; self.tfp = d["t_fprob"]
        self.tdown = d["t_down"]; self.ths = d["t_hs"]
        self.pf = d["p_frame"]; self.pxyz = d["pc_xyz"]
        self.ef = d["e_frame"]; self.eb = d["e_bin"]; self.evec = d["e_vec"]
        self.etid = d["e_tid"]; self.erange = d["e_range"]; self.evel = d["e_vel"]

    def scene(self):
        fi = self.fi
        tm = self.tf == fi
        tgts = [Target(int(self.tid[i]), float(self.tx[i]), float(self.ty[i]),
                       float(self.tz[i]), 0.0, 0.0, 0.0)
                for i in np.where(tm)[0]]
        poses = {}
        for i in np.where(tm)[0]:
            tid = int(self.tid[i]); pv = int(self.tpose[i]); hs = int(self.ths[i])
            poses[tid] = Pose(tid, pv, float(self.tfp[i]),
                              valid=(pv != 0xFF),           # MLP leg
                              down=bool(self.tdown[i]), h_s_cm=hs, low_run=0,
                              win_valid=(hs != 0 or bool(self.tdown[i])))  # window leg emitted
        pcx = self.pxyz[self.pf == fi]
        return {"points": None, "targets": tgts, "poses": poses,
                "t": float(self.d["ts"][fi]), "n_cube": int((self.ef == fi).sum()),
                "z_smooth": None,
                "y0": float(tgts[0].y) if tgts else None,
                "pc_xyz": pcx.astype(np.float32) if len(pcx) else np.empty((0, 3), np.float32)}

    def request_cube(self, range_bin, n_frames=60, half_win=3, timeout=6.0):
        """Return the recorded TLV-320 vectors at bins within half_win of range_bin over
        the forward n_frames window (the burst the live query would have received)."""
        lo, hi = range_bin - half_win, range_bin + half_win
        m = (self.eb >= lo) & (self.eb <= hi) & \
            (self.ef >= self.fi) & (self.ef < self.fi + n_frames)
        idx = np.where(m)[0]
        if len(idx) == 0:            # no recorded burst there -> live would also get nothing
            return []
        return [types.SimpleNamespace(range_bin=int(self.eb[i]), vec=self.evec[i],
                                      tid=int(self.etid[i]), vel_mmps=int(self.evel[i]),
                                      range_m=float(self.erange[i]))
                for i in idx]


def _reset_state():
    """Zero the module timers so a replay starts clean (not carrying a prior run)."""
    srv._real_since[0] = 0.0
    srv._last_query_t[0] = 0.0
    srv._cube_busy[0] = False
    srv._down_since[0] = 0.0
    srv._down_last[0] = 0.0
    srv._fall_latch_until[0] = 0.0
    srv._cube_result.update(rr=None, strength=0.0, t=0.0, floor_frac=0.0, bin=None)
    srv._floor_pts = []
    srv._lost_since.clear(); srv._lost_query_t.clear()
    srv._probe_dry[0] = 0; srv._lost_probe_dry.clear()
    srv._gtrack_prev = {}; srv._fall_deaths = []
    srv._fall_region.update(since=0.0, last=0.0, x=0.0, y=0.0)
    srv._recover_since[0] = 0.0
    from falldet.window import FloorMap, WindowDetector
    from falldet.clean import Cleaner
    from falldet.floor_track import FloorTracker
    srv._floor = FloorMap()
    srv._window = WindowDetector(srv._floor, margin=0.45, sustain=3, clear=3)
    srv._cleaner = Cleaner()
    srv._floor_tracker = FloorTracker(death_grace_s=30.0)


def run(path, mount=2.0, tilt=25.0):
    """Drive the REAL srv._scene() over every frame of the npz. Returns (rows, eps, meta).
    rows: per-frame decision dicts; eps: [(start_s, end_s, [reasons])] fall episodes."""
    d = np.load(path)
    srv.MOUNT = mount; srv.TILT = tilt
    clock = _Clock(); srv.time = clock
    srv.threading.Thread = _SyncThread          # cube fetch runs inline
    srv.print = lambda *a, **k: None            # silence _scene()'s stdout [fall] spam
    srv.open = lambda *a, **k: _Sink()          # silence its fall_debug.log append
    fake = FakeSource(d); srv._src = fake
    _reset_state()

    t0 = float(d["ts"][0])
    rows = []
    for fi in range(fake.nfr):
        fake.fi = fi
        clock.t = float(d["ts"][fi])            # THE clock the server sees this frame
        out = srv._scene()
        rows.append({
            "fi": fi, "t": round(clock.t - t0, 2),
            "fall_state": out["fall_state"],
            "pose": out.get("primary_pose"),
            "w_down": bool(out["fall_ev"]["window"]), "w_src": out["fall_ev"]["win_src"],
            "real": bool(out["fall_ev"]["real"]),
            "floor_fall": bool(out["fall_ev"].get("floor_fall")),
            "down_dur": round(srv._down_since[0] and (clock.t - srv._down_since[0]) or 0.0, 1),
            "cube_rr": out["cube_rr"], "cube_str": out["cube_rr_str"],
            "reason": out["fall_ev"]["reason"],
        })

    eps = []
    cur = None
    for r in rows:
        if r["fall_state"] == "fall":
            if cur is None: cur = [r["t"], r["t"], []]
            cur[1] = r["t"]
            for x in (r["reason"] or []):
                if x not in cur[2]: cur[2].append(x)
        else:
            if cur is not None: eps.append(cur); cur = None
    if cur is not None: eps.append(cur)
    meta = {"file": os.path.basename(path), "frames": fake.nfr,
            "dur": rows[-1]["t"] if rows else 0.0, "mount": mount, "tilt": tilt}
    return rows, eps, meta


def to_json(path, mount=2.0, tilt=25.0, step=3):
    """Structured replay result for the dashboard: episodes + a downsampled timeline
    (every `step` frames -> ~0.3 s) so the front-end can draw a fall-state strip."""
    rows, eps, meta = run(path, mount, tilt)
    tl = [{"t": r["t"], "s": r["fall_state"], "pose": r["pose"], "wd": int(r["w_down"]),
           "ws": r["w_src"], "re": int(r["real"]), "ff": int(r["floor_fall"]),
           "dur": r["down_dur"], "rr": r["cube_rr"], "rs": r["cube_str"]}
          for r in rows[::max(1, step)]]
    return {**meta,
            "episodes": [{"a": a, "b": b, "dur": round(b - a, 1),
                          "reason": sorted(set(x.split("sustained")[0] or "sustained"
                                              for x in why))} for a, b, why in eps],
            "timeline": tl}


def replay(path, mount=2.0, tilt=25.0, per_frame=False):
    rows, eps, meta = run(path, mount, tilt)
    print(f"\n=== REPLAY {meta['file']}  ({meta['frames']} frames = {meta['dur']:.0f}s)"
          f"  mount={mount} tilt={tilt}  [server _scene, code-of-record] ===")
    print(f"\n代码判定 FALL {len(eps)} 段 (fall_state=='fall' 连续段, 含 30s latch):")
    for i, (a, b, why) in enumerate(eps, 1):
        print(f"  #{i}: {a:6.1f}-{b:6.1f}s  持续{b-a:5.1f}s  reason={why}")

    print("\n每 3s: fall_state | pose | w_down(src) real down_dur cube_rr(str)")
    seen = set()
    for r in rows:
        s3 = int(r["t"] // 3)
        if s3 in seen and not per_frame: continue
        seen.add(s3)
        rr = "-" if r["cube_rr"] in (None, 0) else f"{r['cube_rr']}"
        flag = {"fall": "🔴FALL", "suspected": "🟡susp", "none": "  ·  "}.get(r["fall_state"], r["fall_state"])
        print(f"  {r['t']:6.1f}s {flag:7s} {str(r['pose'] or '-'):5s} "
              f"w={int(r['w_down'])}({r['w_src']}) real={int(r['real'])} "
              f"dur={r['down_dur']:4.0f} rr={rr}({r['cube_str']})")
    return eps, rows


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("npz")
    ap.add_argument("--mount", type=float, default=2.0)
    ap.add_argument("--tilt", type=float, default=25.0)
    ap.add_argument("--frames", action="store_true", help="print every frame (not per-3s)")
    ap.add_argument("--json", action="store_true", help="emit structured JSON (for the dashboard)")
    a = ap.parse_args()
    if a.json:
        print(json.dumps(to_json(a.npz, a.mount, a.tilt)))
    else:
        replay(a.npz, a.mount, a.tilt, a.frames)
