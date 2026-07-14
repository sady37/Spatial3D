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

    def __init__(self, maxwin_s=45.0, record_prefix=None, block_s=300.0, bins=range(60, 271)):
        self.maxwin_s = maxwin_s
        self.dr = DR_M
        self.buf = {}                          # bin -> deque[(ts, vec16)]
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
        self._rec_bucket = None                 # int(epoch // block_s) of current file

    def _bucket_stamp(self, bucket):
        """Local-time filename stamp = START of the 5-min wall-clock bucket."""
        return time.strftime("%Y%m%d_%H%M%S", time.localtime(bucket * self.block_s))

    def _flush_block(self):
        """Save the current block to <prefix>_<bucketstart>.npz (cap_stream format)."""
        if not self.record_prefix or self._rec_acc is None or not self._rec_ts:
            return
        out = f"{self.record_prefix}_{self._bucket_stamp(self._rec_bucket)}.npz"
        binsA, cube, counts = pack_snapshots(self._rec_acc, self.rec_bins, min_snapshots=20)
        if len(binsA) == 0:
            print(f"[rec] {out}: 0 usable bins, skipped", flush=True)
            return
        t0 = self._rec_bucket * self.block_s
        mean = np.stack([cube[i, :int(counts[i])].mean(0) for i in range(len(binsA))]).astype(np.complex64)
        save_cube(out, self._rec_acc, self.rec_bins, min_snapshots=20, mean=mean,
                  frame_ts=np.array(self._rec_ts, dtype=np.float64), block_start_epoch=np.float64(t0))
        dur = self._rec_ts[-1] - self._rec_ts[0]
        print(f"[rec] SAVED {out}: {len(binsA)} bins, {len(self._rec_ts)} frames, {dur:.0f}s", flush=True)

    def _record(self, ra, ts):
        """Accumulate into the current 5-min block; flush + rotate on bucket change."""
        bucket = int(ts // self.block_s)
        if self._rec_bucket is None:
            self._rec_bucket = bucket
            self._rec_acc = BinAccumulator(k=200000, n_ant=N_VIRT_ANT)
        elif bucket != self._rec_bucket:
            self._flush_block()
            self._rec_bucket = bucket
            self._rec_acc = BinAccumulator(k=200000, n_ant=N_VIRT_ANT)
            self._rec_ts = []
        self._rec_acc.add(ra)
        self._rec_ts.append(ts)

    def _reader(self):
        from spatial3d.uart_reader import RadarSession
        self._sess = RadarSession(self.CLI, self.DATA)
        self._sess.start_drain()
        while self._run:
            f = self._sess.get_frame(timeout=1.0)
            if f is None:
                continue
            ra = f.range_antenna()
            if ra is None:
                continue
            ts = getattr(f, "rx_ts", None)
            ts = _epoch([ts])[0] if ts else time.time()
            if self.record_prefix:
                self._record(ra, ts)
            cutoff = time.time() - self.maxwin_s
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
        self._flush_block()                              # save the partial current block
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


def make_source(spec, record_prefix=None):
    """spec='live' or a cube npz path (optionally 'cube.npz@watch.csv').
    record_prefix (live only) writes 5-min wall-clock cube files while serving."""
    if spec == "live":
        return LiveSource(record_prefix=record_prefix)
    if "@" in spec:
        cube, watch = spec.split("@", 1)
    else:
        cube, watch = spec, None
    return ReplaySource(cube, watch_csv=watch)
