#!/usr/bin/env python3
"""Handshake cube collector: `python3 qlog.py "<label>"`
Logs the instruction (time+content), waits 3 s (prep time), reads /api/scene, picks the
in-room track matching the stated zone+posture (avoids ghosts), fires a cube burst at its
bin (retry on the cubeGuard's momentary 0), logs {instr_t, label, query_t, bin, wz, RR,
entries} to record/neg_labels_<date>.jsonl (align to the recording npz by timestamp)."""
import sys, json, math, time, urllib.request

BASE = "http://localhost:8765"; TH = math.radians(25.0); M = 2.0; STEP = 0.085
LABEL = sys.argv[1] if len(sys.argv) > 1 else "?"
DELAY = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0   # 0 = query immediately (you're posed)
FORCE_BIN = int(sys.argv[3]) if len(sys.argv) > 3 else None  # explicit bin (for lying, track drops)
LOG = f"/Users/sady3721/project/owl/Spatial3D/pc/record/neg_labels_{time.strftime('%Y%m%d')}.jsonl"

def get(u, t=12): return json.load(urllib.request.urlopen(u, timeout=t))
def scene(): return get(f"{BASE}/api/scene", 5)

# ---- parse the stated zone + posture from the label (en or 中文) ----
low = LABEL.lower()
zone = "near" if any(k in low for k in ("near", "近")) else \
       "mid" if any(k in low for k in ("mid", "中")) else \
       "far" if any(k in low for k in ("far", "远")) else None
post = "stand" if any(k in low for k in ("stand", "站")) else \
       "sit" if any(k in low for k in ("sit", "坐")) else \
       "lie" if any(k in low for k in ("lie", "lay", "躺")) else None
is_empty = any(k in low for k in ("empty", "空", "out", "走"))

instr_t = time.time()
print(f"[指令 {time.strftime('%H:%M:%S')}] \"{LABEL}\"  (zone={zone} post={post} empty={is_empty}) -> 等 {DELAY:.0f}s")
time.sleep(DELAY)

# ---- pick the matching track ----
d = scene()
tg = [t for t in d.get("targets", []) if abs(t["x"]) < 3.5 and 0.3 < t["y"] < 5.3]
def feats(t):
    wy = t["y"] * math.cos(TH) + t["z"] * math.sin(TH)
    wz = M + t["z"] * math.cos(TH) - t["y"] * math.sin(TH)
    z = "near" if wy < 1.8 else ("mid" if wy < 3.2 else "far")
    p = "stand" if wz > 0.9 else ("sit" if wz > 0.35 else "lie")
    return wy, wz, z, p, round(wy / STEP)

chosen, det = None, None
if FORCE_BIN is not None:                      # user gave the exact bin (lying track dropped)
    b = FORCE_BIN
    det = f"forced-bin{b}"
elif is_empty or not tg:
    b = 30                                    # empty -> a fixed mid bin
    det = "empty" if not tg else "empty(label)"
else:
    scored = []
    for t in tg:
        wy, wz, z, p, b = feats(t)
        s = (2 if zone and z == zone else 0) + (2 if post and p == post else 0)
        scored.append((s, t, wy, wz, z, p, b))
    scored.sort(key=lambda x: -x[0])
    _, chosen, wy, wz, z, p, b = scored[0]
    det = f"{z}-{p}"

# ---- fire the burst, retry on the cubeGuard 0 ----
res = {"entries": 0}
for att in range(2):                            # at most 2 tries (keep it FAST)
    res = get(f"{BASE}/api/cube?bin={int(b)}&n=40&hw=3")   # n=40 (~4s burst)
    if res.get("entries", 0) > 0 or res.get("error"):
        break
    time.sleep(2)
d2 = scene()
row = {"instr_t": round(instr_t, 1), "instr_hms": time.strftime('%H:%M:%S', time.localtime(instr_t)),
       "label": LABEL, "query_t": round(time.time(), 1), "query_hms": time.strftime('%H:%M:%S'),
       "zone": zone, "post": post, "detected": det, "bin": int(b),
       "wz": (None if chosen is None else round(feats(chosen)[1], 2)),
       "attempts": att + 1, "entries": res.get("entries"), "n_ant": res.get("n_ant"),
       "range_bins": res.get("range_bins"), "scene_rr": d2.get("cube_rr"),
       "scene_str": d2.get("cube_rr_str"), "err": res.get("error")}
with open(LOG, "a") as f: f.write(json.dumps(row, ensure_ascii=False) + "\n")
print(f"[查询 {row['query_hms']}] label=\"{LABEL}\" detected={det} bin={b} wz={row['wz']} "
      f"entries={row['entries']} n_ant={row['n_ant']} RR={row['scene_rr']} str={row['scene_str']} "
      f"tries={att+1}  -> {LOG}")
