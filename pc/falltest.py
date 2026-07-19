#!/usr/bin/env python3
"""Record a REAL sit->half-kneel FALL with cube coverage, for the temporal/descent-rate feature.
`python3 falltest.py "<label>" [bin]`
Timeline (from GO): T0 log + ensure recording ON -> wait 3 s -> read the SITTING person's bin
(you're still seated at T3, track alive) -> fire 3 cube bursts (n=50 ~5 s) at T~3/11/19 covering
~18 s (spaced to avoid the cubeGuard wedge; NOT one long 18 s query). You: sit by T3, then at
~T5 collapse sit->half-kneel. The 3001 cloud (continuous in the recording) captures the DESCENT;
the cube adds RR/energy of the collapse + aftermath. Logs {go_t, bursts, bin} to
pc/record/falltest_<date>.jsonl for timestamp-alignment (the fall is ~T5 after go)."""
import sys, json, math, time, urllib.request

BASE = "http://localhost:8765"; TH = math.radians(25.0); M = 2.0; STEP = 0.085
LABEL = sys.argv[1] if len(sys.argv) > 1 else "sit->halfkneel-fall"
FORCE_BIN = int(sys.argv[2]) if len(sys.argv) > 2 else None
LOG = f"/Users/sady3721/project/owl/Spatial3D/pc/record/falltest_{time.strftime('%Y%m%d')}.jsonl"

def get(u, t=12): return json.load(urllib.request.urlopen(u, timeout=t))
def scene(): return get(f"{BASE}/api/scene", 5)

# ensure recording is ON (so the 3001 descent + cube get saved)
rec = get(f"{BASE}/api/rec/on", 5)
go = time.time()
print(f"[GO {time.strftime('%H:%M:%S')}] \"{LABEL}\"  rec={rec}  -> 3s 后开查(你: sit好, ~T5 做 sit->半跪 fall)")
time.sleep(3)

# read the seated person's bin (still seated at T3, track alive) -- else forced bin
b = FORCE_BIN
if b is None:
    d = scene()
    tg = [t for t in d.get("targets", []) if abs(t["x"]) < 3.5 and 0.3 < t["y"] < 5.3]
    if tg:
        t = min(tg, key=lambda t: abs(t["x"]))
        wy = t["y"] * math.cos(TH) + t["z"] * math.sin(TH)
        b = int(round(wy / STEP))
        wz = M + t["z"] * math.cos(TH) - t["y"] * math.sin(TH)
        print(f"  T3 座位: tid{t['tid']} wz={wz:+.2f} -> bin{b}")
    else:
        b = 20; print(f"  T3 无 target -> 兜底 bin{b}")

# 3 cube bursts (n=50 ~5s) at ~T3/11/19 from go -> ~18s coverage, spaced to avoid wedge
bursts = []
for k in range(3):
    tgt = go + 3 + k * 8                       # T3, T11, T19
    while time.time() < tgt: time.sleep(0.2)
    r = get(f"{BASE}/api/cube?bin={int(b)}&n=50&hw=3")
    row = {"t": round(time.time(), 1), "rel": round(time.time() - go, 1), "bin": int(b),
           "entries": r.get("entries"), "n_ant": r.get("n_ant")}
    bursts.append(row)
    print(f"  burst{k+1} rel+{row['rel']}s bin{b} entries={row['entries']}")

with open(LOG, "a") as f:
    f.write(json.dumps({"go_t": round(go, 1), "go_hms": time.strftime('%H:%M:%S', time.localtime(go)),
                        "label": LABEL, "bin": b, "fall_rel_s": 5, "bursts": bursts},
                       ensure_ascii=False) + "\n")
print(f"[DONE] -> {LOG}  (npz 里 go+~5s = sit->半跪 fall 时刻; 3001 连续云含下坠)")
