"""SOURCE layer — feeds the compute layer a trailing window, identically for a
live sensor and a recorded cube. The web server holds a Source and never cares
which one it is; swapping Replay<->Live is how you validate an algorithm change
against watch-truth first, then run it live.

Common interface:
    src.meta()            -> {"bins": [...], "dr": float, "kind": str}
    src.window(win_s)     -> (cube_win, bins, dr, fps, t_wall, watch_hr) | None
    src.start(); src.stop()

cube_win is (nbin, T, 16) complex64 = the last `win_s` seconds. watch_hr is the
aligned Apple-Watch bpm (Replay only; None live).
"""
from __future__ import annotations
import csv
import os
import sys
import threading
import time
from collections import deque
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from spatial3d.range_music import DR_M, N_VIRT_ANT, BinAccumulator
from spatial3d.cube import pack_snapshots, save_cube


def _epoch(ts):
    ts = np.asarray(ts, float)
    return ts / 1000.0 if np.nanmax(ts) > 1e12 else ts   # ms -> s if needed


# ------------------------------------------------------------------- Replay --
class ReplaySource:
    """Stream a recorded cube in real time (by its frame_ts). No hardware.
    Optional watch CSV (epoch,bpm) is aligned to the playhead for live compare."""

    kind = "replay"

    def __init__(self, npz_path, watch_csv=None, speed=1.0, loop=True):
        d = np.load(npz_path, allow_pickle=True)
        self.cube = np.asarray(d["snapshots"], np.complex64)
        self.bins = d["bins"].astype(int)
        self.dr = float(d["dr_m"]) if "dr_m" in d.files else DR_M
        self.ts = _epoch(d["frame_ts"])
        self.fps = (len(self.ts) - 1) / (self.ts[-1] - self.ts[0])
        self.speed = float(speed)
        self.loop = loop
        self.name = os.path.basename(npz_path)
        self._t0 = None
        self.watch = None
        if watch_csv and os.path.exists(watch_csv):
            w = []
            with open(watch_csv) as fp:
                for r in csv.DictReader(fp):
                    try:
                        w.append((float(r["epoch"]), float(r["bpm"])))
                    except (KeyError, ValueError):
                        pass
            if w:
                w.sort()
                self.watch = np.array(w)

    def meta(self):
        return dict(bins=[int(b) for b in self.bins], dr=self.dr, kind=self.kind,
                    name=self.name, fps=round(self.fps, 2),
                    duration_s=round(self.ts[-1] - self.ts[0], 1))

    def start(self):
        self._t0 = time.time()

    def stop(self):
        pass

    def _playhead(self):
        elapsed = (time.time() - self._t0) * self.speed
        ph_time = self.ts[0] + elapsed
        if self.loop and ph_time > self.ts[-1]:
            span = self.ts[-1] - self.ts[0]
            ph_time = self.ts[0] + (elapsed % span)
        idx = int(np.searchsorted(self.ts, ph_time))
        return min(max(idx, 0), len(self.ts) - 1), ph_time

    def _watch_hr(self, ph_time):
        if self.watch is None:
            return None
        j = int(np.argmin(np.abs(self.watch[:, 0] - ph_time)))
        return float(self.watch[j, 1]) if abs(self.watch[j, 0] - ph_time) < 8.0 else None

    def window(self, win_s):
        if self._t0 is None:
            return None
        ph, ph_time = self._playhead()
        n = int(win_s * self.fps)
        lo = max(0, ph - n)
        if ph - lo < int(self.fps * 5):        # need >=5s to say anything
            return None
        cube_win = self.cube[:, lo:ph, :]
        return cube_win, self.bins, self.dr, self.fps, ph_time, self._watch_hr(ph_time)


# --------------------------------------------------------------------- Live --
class LiveSource:
    """Attach READ-ONLY to the streaming sensor and keep a rolling per-bin buffer.
    Mirrors cap_stream's accumulation (BinAccumulator.add semantics) but evicts
    old snapshots so window() always returns the last `maxwin` seconds."""

    kind = "live"
    CLI = "/dev/cu.usbmodem0000RA441"
    DATA = "/dev/cu.usbmodem0000RA444"
    STALE_S = 12.0                     # no frame for this long => warn: sensor wedged

    def __init__(self, maxwin_s=65.0, record_prefix=None, block_s=300.0, bins=range(60, 271),
                 save_on=True):
        self.maxwin_s = maxwin_s
        self.dr = DR_M
        self.buf = {}                          # bin -> deque[(ts, vec16)]
        self._scene = {"points": None, "targets": [], "t": 0.0}  # People_Tracking live scene
        self._z_hist = deque(maxlen=7)         # primary-track Z, ~0.7s median-smoothed
        self._pc_hist = deque()                # (ts, xyz, snr) 3001 points, ~0.5s accum
        self._cube_hold_ts = 0.0               # last ts a TLV 320 fired = fall-trigger (fall cfg ARM)
        self._fall_since = 0.0                 # ts the CURRENT continuous 320 episode began (for confirm timer)
        self._cube_query = None                # server-triggered cube fetch state (see request_cube)
        self._lock = threading.Lock()
        self._run = False
        self._thr = None
        self._sess = None
        self.name = "live-sensor"
        # ---- recording (same read-only stream also persists 5-min cube files) ----
        self.record_prefix = record_prefix     # None disables recording
        self.block_s = block_s                  # 300 = 5-min wall-clock buckets
        self.rec_bins = bins
        self._rec_acc = None
        self._rec_ts = []
        self._sc_frames = []                    # People_Tracking scene/320 record buffer
        self._rec_bucket = None                 # int(epoch // block_s) of current file
        self._rec_on = bool(save_on)            # runtime SAVE switch; stream runs regardless
        self._ctl = None                        # pending on/off/status req (serviced in reader)

    def _bucket_stamp(self, bucket):
        """Local-time filename stamp = START of the 5-min wall-clock bucket."""
        return time.strftime("%Y%m%d_%H%M%S", time.localtime(bucket * self.block_s))

    def _flush_block(self):
        """Save the current 5-min block to <prefix>_<bucketstart>.npz (unchanged
        cadence). Returns the saved path (None if nothing to save)."""
        if not self.record_prefix or self._rec_acc is None or not self._rec_ts:
            return None
        out = f"{self.record_prefix}_{self._bucket_stamp(self._rec_bucket)}.npz"
        if os.path.exists(out):                 # same 5-min bucket flushed twice (e.g. WRITE
            k = 2                               # off->on->off within one window) must NOT
            while os.path.exists(f"{out[:-4]}_{k}.npz"):   # overwrite the earlier segment
                k += 1
            out = f"{out[:-4]}_{k}.npz"
        binsA, cube, counts = pack_snapshots(self._rec_acc, self.rec_bins, min_snapshots=20)
        if len(binsA) == 0:
            print(f"[rec] {out}: 0 usable bins, skipped", flush=True)
            return None
        t0 = self._rec_bucket * self.block_s
        mean = np.stack([cube[i, :int(counts[i])].mean(0) for i in range(len(binsA))]).astype(np.complex64)
        save_cube(out, self._rec_acc, self.rec_bins, min_snapshots=20, mean=mean,
                  frame_ts=np.array(self._rec_ts, dtype=np.float64), block_start_epoch=np.float64(t0))
        dur = self._rec_ts[-1] - self._rec_ts[0]
        print(f"[rec] SAVED {out}: {len(binsA)} bins, {len(self._rec_ts)} frames, {dur:.0f}s", flush=True)
        return out

    def _flush_scene(self):
        """Save the current 5-min block of People_Tracking scene/320 to
        <prefix>_scene_<bucketstart>.npz (cap_320 schema + per-frame track XYZ)."""
        if not self.record_prefix or not self._sc_frames:
            return None
        out = f"{self.record_prefix}_scene_{self._bucket_stamp(self._rec_bucket)}.npz"
        if os.path.exists(out):
            k = 2
            while os.path.exists(f"{out[:-4]}_{k}.npz"):
                k += 1
            out = f"{out[:-4]}_{k}.npz"
        ts, n_points = [], []
        e_frame, e_tid, e_bin, e_vel, e_range, e_vec = [], [], [], [], [], []
        t_frame, t_tid, t_x, t_y, t_z = [], [], [], [], []
        p_fr, p_arrs = [], []                   # per-frame 3001 minor point cloud (for offline box/LIE)
        n_ant = 0
        for fi, fr in enumerate(self._sc_frames):
            ts.append(fr["ts"]); n_points.append(fr["n_points"])
            for e in fr["tbc"]:
                e_frame.append(fi); e_tid.append(e.tid); e_bin.append(e.range_bin)
                e_vel.append(e.vel_mmps); e_range.append(e.range_m); e_vec.append(e.vec)
                n_ant = len(e.vec)
            for t in fr["tgts"]:
                t_frame.append(fi); t_tid.append(t.tid)
                t_x.append(t.x); t_y.append(t.y); t_z.append(t.z)
            pc = fr.get("pc")
            if pc is not None and len(pc):
                p_arrs.append(np.asarray(pc, np.float32))
                p_fr.append(np.full(len(pc), fi, np.int32))
        pc_xyz_all = (np.concatenate(p_arrs) if p_arrs else np.empty((0, 3), np.float32))
        p_frame = (np.concatenate(p_fr) if p_fr else np.empty(0, np.int32))
        np.savez_compressed(
            out,
            ts=np.asarray(ts, np.float64), n_points=np.asarray(n_points, np.int32),
            n_ant=np.int32(n_ant),
            e_frame=np.asarray(e_frame, np.int32), e_tid=np.asarray(e_tid, np.int32),
            e_bin=np.asarray(e_bin, np.int32), e_vel=np.asarray(e_vel, np.int16),
            e_range=np.asarray(e_range, np.float32),
            e_vec=(np.stack(e_vec).astype(np.complex64) if e_vec
                   else np.empty((0, n_ant), np.complex64)),
            t_frame=np.asarray(t_frame, np.int32), t_tid=np.asarray(t_tid, np.int32),
            t_x=np.asarray(t_x, np.float32), t_y=np.asarray(t_y, np.float32),
            t_z=np.asarray(t_z, np.float32),
            p_frame=p_frame, pc_xyz=pc_xyz_all,
            block_start_epoch=np.float64(self._rec_bucket * self.block_s))
        print(f"[rec] SAVED {out}: {len(ts)} frames, {len(e_frame)} 320-entries, "
              f"{len(t_frame)} track-pts, {len(pc_xyz_all)} cloud-pts", flush=True)
        return out

    def _service_ctl(self):
        """Serviced in the reader thread (sole owner of rec state) so the SAVE
        switch never races with accumulation. ON => resume 5-min rolling. OFF =>
        immediately flush the current partial (don't wait for the :05 boundary)
        and stop writing; the stream keeps flowing either way."""
        req = self._ctl
        if req is None:
            return
        act = req["action"]
        if act == "on":
            self._rec_on = True
            req["result"] = {"saving": True}
        elif act == "off":
            path = self._flush_block()          # 立即落盘当前这段
            sc_path = self._flush_scene()
            self._rec_on = False
            self._rec_acc = None
            self._rec_ts = []
            self._sc_frames = []
            self._rec_bucket = None
            req["result"] = {"saving": False, "saved": sc_path or path}
        else:                                   # status
            tseq = self._rec_ts if len(self._rec_ts) > 1 else [f["ts"] for f in self._sc_frames]
            frames = max(len(self._rec_ts), len(self._sc_frames))
            dur = (tseq[-1] - tseq[0]) if len(tseq) > 1 else 0.0
            req["result"] = {"saving": self._rec_on, "frames": frames,
                             "dur_s": round(dur, 1)}
        req["ev"].set()
        self._ctl = None

    def rec_set(self, on, timeout=4.0):
        """Flip the SAVE switch at runtime (stream keeps flowing either way)."""
        if not self.record_prefix:
            return {"error": "recording disabled (--no-record)"}
        req = {"action": "on" if on else "off", "ev": threading.Event(), "result": None}
        self._ctl = req
        req["ev"].wait(timeout)
        return req["result"]

    def rec_status(self, timeout=4.0):
        if not self.record_prefix:
            return {"saving": False, "disabled": True}
        req = {"action": "status", "ev": threading.Event(), "result": None}
        self._ctl = req
        req["ev"].wait(timeout)
        return req["result"]

    def _maybe_rollover(self, now):
        """Wall-clock-driven 5-min bucket rotation (0-4 -> file, 5-9 -> file, ...).
        Runs every reader tick even with NO frame, so a wedged stream still flushes
        on time instead of stalling forever. No-op while the SAVE switch is off."""
        if not self.record_prefix or not self._rec_on:
            return
        bucket = int(now // self.block_s)
        if self._rec_bucket is None:
            self._rec_bucket = bucket
            self._rec_acc = BinAccumulator(k=200000, n_ant=N_VIRT_ANT)
            self._sc_frames = []
        elif bucket != self._rec_bucket:
            self._flush_block()
            self._flush_scene()
            self._rec_bucket = bucket
            self._rec_acc = BinAccumulator(k=200000, n_ant=N_VIRT_ANT)
            self._rec_ts = []
            self._sc_frames = []

    def _record(self, ra, ts):
        """Accumulate one frame into the current block (rotation is time-driven in
        _maybe_rollover)."""
        self._rec_acc.add(ra)
        self._rec_ts.append(ts)

    def _reader(self):
        from spatial3d.uart_reader import RadarSession
        self._sess = RadarSession(self.CLI, self.DATA)
        self._sess.start_drain()
        last_frame_w = time.time()     # wall time of the last frame off the wire
        last_warn = 0.0
        stale = False
        while self._run:
            f = self._sess.get_frame(timeout=1.0)
            now = time.time()
            self._service_ctl()        # runtime SAVE on/off switch (owned by this thread)
            self._maybe_rollover(now)  # 5-min rolling; no-op when saving off / survives wedge
            if f is None:
                # WEDGE DETECTION: stream went silent. Warn loudly + repeatedly so a
                # dead sensor never masquerades as a healthy 'recording' run.
                if now - last_frame_w > self.STALE_S and now - last_warn > 20.0:
                    print(f"[live] WARN: no radar frames for {now - last_frame_w:.0f}s — "
                          f"sensor likely WEDGED. Recordings are EMPTY; power-cycle the EVM.",
                          flush=True)
                    last_warn = now
                    stale = True
                continue
            if stale:
                print(f"[live] frames resumed after {now - last_frame_w:.0f}s gap.", flush=True)
                stale = False
            last_frame_w = now
            # People_Tracking SCENE (points + tracks) — every frame, independent of
            # range_antenna (which is None on the People_Tracking firmware).
            try:
                sc_ts = getattr(f, "rx_ts", None) or now
                pts = f.detected_points(); tgts = f.targets()
                tbc = f.track_bin_cube()
                ncube = len(tbc.entries) if tbc is not None else 0
                if ncube > 0:
                    # fall-trigger: 320 fired. A NEW episode begins if 320 was quiet (>1.5s)
                    # before now; otherwise it's the same ongoing episode (confirm timer runs).
                    if not self._cube_hold_ts or (sc_ts - self._cube_hold_ts) > 1.5:
                        self._fall_since = sc_ts
                    self._cube_hold_ts = sc_ts
                # server-triggered cube fetch: collect N frames of 320 at the queried range
                q = self._cube_query
                if q is not None:
                    if tbc is not None:
                        for e in tbc.entries:
                            if abs(int(e.range_bin) - q["bin"]) <= q["hw"]:
                                q["entries"].append(e)
                    q["left"] -= 1
                    if q["left"] <= 0:
                        q["done"].set()
                if tgts:
                    self._z_hist.append(float(tgts[0].z))
                z_smooth = float(np.median(self._z_hist)) if self._z_hist else None
                # accumulate the 3001 minor point cloud over ~0.5s for the block-person
                pc = f.point_cloud()
                if pc is not None and len(pc.xyz):
                    self._pc_hist.append((sc_ts, pc.xyz, pc.snr))
                while self._pc_hist and self._pc_hist[0][0] < now - 0.5:
                    self._pc_hist.popleft()
                if self._pc_hist:
                    pc_xyz = np.concatenate([p[1] for p in self._pc_hist])
                    pc_snr = np.concatenate([p[2] for p in self._pc_hist])
                else:
                    pc_xyz = np.empty((0, 3), np.float32); pc_snr = np.empty(0, np.float32)
                self._scene = {"points": pts, "targets": tgts, "t": sc_ts, "n_cube": ncube,
                               "z_smooth": z_smooth,
                               "y0": float(tgts[0].y) if tgts else None,
                               "pc_xyz": pc_xyz, "pc_snr": pc_snr,
                               "cube_ts": self._cube_hold_ts, "fall_since": self._fall_since}
                # Feed the breathing/HR pipeline: this firmware emits the slow-time cube
                # as TLV 320 (per STILL-track per-bin 16-ant zero-Doppler vectors), NOT
                # range_antenna. Route those into the SAME per-bin buffer window() reads,
                # so RR/HR compute whenever a still track is present.
                if tbc is not None and tbc.entries:
                    cutoff = now - self.maxwin_s
                    with self._lock:
                        for e in tbc.entries:
                            dq = self.buf.setdefault(int(e.range_bin), deque())
                            dq.append((sc_ts, np.asarray(e.vec, np.complex64)))
                            while dq and dq[0][0] < cutoff:
                                dq.popleft()
                # record the scene/320 stream on the same 5-min buckets as the cube,
                # gated by the SAME write switch (single upstream control).
                if self.record_prefix and self._rec_on and self._rec_bucket is not None:
                    self._sc_frames.append({
                        "ts": sc_ts, "n_points": len(pts), "tgts": tgts,
                        "tbc": tbc.entries if tbc is not None else [],
                        "pc": (pc.xyz if (pc is not None and len(pc.xyz)) else None)})
            except Exception:
                pass
            ra = f.range_antenna()
            if ra is None:
                continue
            ts = getattr(f, "rx_ts", None)
            ts = _epoch([ts])[0] if ts else time.time()
            if self.record_prefix and self._rec_on:
                self._record(ra, ts)
            cutoff = now - self.maxwin_s
            with self._lock:
                for i in range(ra.num_bins):
                    b = int(ra.start_bin + i)
                    dq = self.buf.setdefault(b, deque())
                    dq.append((ts, np.asarray(ra.data[i], np.complex64)))
                    while dq and dq[0][0] < cutoff:
                        dq.popleft()

    def start(self):
        self._run = True
        self._thr = threading.Thread(target=self._reader, daemon=True)
        self._thr.start()

    def stop(self):
        self._run = False
        if self._thr:
            self._thr.join(timeout=2.0)
        if self._rec_on:
            self._flush_block()                          # save the partial current block
            self._flush_scene()                          # + partial People_Tracking scene/320
        if self._sess:
            try:
                self._sess.close(stop_sensor=False)      # READ-ONLY: never stop the sensor
            except Exception:
                pass

    def meta(self):
        with self._lock:
            bins = sorted(self.buf)
        return dict(bins=bins, dr=self.dr, kind=self.kind, name=self.name,
                    fps=None, duration_s=None)

    def scene(self):
        """Latest People_Tracking points + tracks (for the /api/scene panel)."""
        return self._scene

    def request_cube(self, range_bin, n_frames=30, half_win=3, timeout=6.0):
        """SERVER-TRIGGERED cube fetch (the fall second-check). On a window/MLP fall
        trigger — or when a track is LOST at a low position — the cleaner calls this with
        the fall's RANGE (from the point cloud, NOT a track pointer, so track-loss is a
        non-problem). It asks the firmware to dump the TLV-320 cube at range_bin +-
        half_win for n_frames, collects those entries off the live stream, and returns
        them for the RR / floor-band-energy check that confirms a living body is down.

        Firmware CLI contract (TO IMPLEMENT on the VM — generalises trackBinCubeCfg from
        'per still track' to 'a specified range window', track-independent):
            cubeQuery <range_bin> <half_win> <n_frames>
        -> firmware bursts 320 for n_frames at that range window, then stops.
        Until cubeQuery exists on-chip this still collects whatever 320 streams near
        range_bin (e.g. from trackcube/fall cfg), so the server plumbing is testable now.
        Returns a list of TrackBinEntry (empty if no session / nothing arrived)."""
        import threading as _th
        if not self._sess:
            return []
        q = {"bin": int(range_bin), "hw": int(half_win), "left": int(n_frames),
             "entries": [], "done": _th.Event()}
        with self._lock:
            self._cube_query = q
        try:
            self._sess.send_cli(f"cubeQuery {int(range_bin)} {int(half_win)} {int(n_frames)}")
        except Exception as e:
            print(f"[cube-query] send_cli failed (firmware may lack cubeQuery): {e}", flush=True)
        q["done"].wait(timeout)
        with self._lock:
            self._cube_query = None
        return q["entries"]

    def window(self, win_s):
        now = time.time()
        lo = now - win_s
        with self._lock:
            bins = sorted(b for b, dq in self.buf.items()
                          if dq and dq[-1][0] >= now - 3.0)   # bin seen in last 3s
            series = {}
            for b in bins:
                vecs = [v for (ts, v) in self.buf[b] if ts >= lo]
                if len(vecs) >= 16:
                    series[b] = vecs
        if len(series) < 4:
            return None
        T = min(len(v) for v in series.values())
        if T < int(win_s * 3):                 # too few frames for this window
            return None
        keep = sorted(series)
        cube_win = np.stack([np.stack(series[b][-T:], 0) for b in keep], 0)  # (nbin,T,16)
        fps = T / win_s
        return cube_win, np.array(keep, int), self.dr, fps, now, None


def make_source(spec, record_prefix=None, save_on=True):
    """spec='live' or a cube npz path (optionally 'cube.npz@watch.csv').
    record_prefix (live only) writes 5-min wall-clock cube files while serving;
    save_on=False starts with the SAVE switch off (stream only until /api/rec/on)."""
    if spec == "live":
        return LiveSource(record_prefix=record_prefix, save_on=save_on)
    if "@" in spec:
        cube, watch = spec.split("@", 1)
    else:
        cube, watch = spec, None
    return ReplaySource(cube, watch_csv=watch)
