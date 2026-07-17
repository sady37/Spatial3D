"""Host validation: firmware PoseMlp_process (MLP leg + window leg) vs Python.

MLP leg   -> pc/pose/model.py folded_forward (exported weights).
Window leg-> pc/falldet/window.py WindowDetector, fed the SAME parametric world
             height the firmware computes (h = mount + z*cos(tilt) - y*sin(tilt)).
             The firmware holds window state on a <2-point dropout (winValid=0)
             rather than treating it as "not low"; we compare only >=2-point
             frames, so the two agree exactly there.
"""
import numpy as np, subprocess, re, sys, os, math
HERE = os.path.dirname(os.path.abspath(__file__))
PC = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, PC)
from pose.model import folded_forward
from falldet.window import FloorMap, WindowDetector

src = open(os.path.join(PC, "..", "firmware", "people_tracking_6844", "src", "6844",
                        "mss", "source", "pose", "pose_model.c")).read()
def grab(name):
    m = re.search(r"const float %s((?:\[\d+\])+) = \{(.*?)\};" % name, src, re.S)
    dims = [int(d) for d in re.findall(r"\[(\d+)\]", m.group(1))]
    vals = np.array([float(x[:-1]) for x in re.findall(r"[-+][0-9.]+e[-+][0-9]+f", m.group(2))])
    return vals.reshape(dims)
layers = [(grab(f"gPoseW{i}"), grab(f"gPoseB{i}")) for i in range(4)]

GATE = 0.75; NF = 8; NP = 5
ZOFF = 0.30; MOUNT = 2.0; TILT = math.radians(25.0); MARGIN = 0.45; SUSTAIN = 5
rng = np.random.default_rng(7)

def world_h(y, z):
    return MOUNT + z * math.cos(TILT) - y * math.sin(TILT)

def build_frame(kin, pts):
    d = pts[:, :2] - np.array([kin['posX'], kin['posY']])
    g = pts[(d[:, 0]**2 + d[:, 1]**2) <= GATE * GATE]
    if len(g) < NP:
        return None
    top = g[np.argsort(g[:, 2], kind='stable')][-NP:]     # 5 highest by z, ascending
    f = [kin['posZ'] + ZOFF, kin['velY'], kin['velZ'], kin['accY'], kin['accZ']]
    for r in top:
        f += [r[1] - kin['posY'], r[2], r[3]]
    return np.array(f, dtype=np.float32)

# Scenario: one track. Frames 0-4 standing (points ~1 m up), 5-13 collapsed to
# the floor (points near z that maps to h~0.1 m). Exercises the sustain latch.
lines = [f"{ZOFF} {MOUNT} {TILT} {MARGIN} {SUSTAIN}"]
mlp_ref, win_ref = [], []      # (fi, ...) expected
hist = []
fm = FloorMap(); fm.at = lambda x, y: 0.0        # floor folded into world_h already
wd = WindowDetector(fm, margin=MARGIN, sustain=SUSTAIN, clear=SUSTAIN)

for fi in range(14):
    standing = fi < 5
    posz = 0.2 if standing else -1.6
    kin = dict(tid=1, posX=0.1, posY=2.5, posZ=posz,
               velY=0.0, velZ=0.0, accY=0.0, accZ=0.0)
    M = int(rng.integers(6, 12))
    zc = rng.uniform(-0.2, 0.9, M) if standing else rng.uniform(-1.75, -1.5, M)
    pts = np.column_stack([rng.uniform(-0.4, 0.6, M), rng.uniform(2.1, 2.9, M),
                           zc, rng.uniform(5, 40, M)]).astype(np.float32)
    lines.append(f"F 1 {M}")
    lines.append(f"1 {kin['posX']} {kin['posY']} {kin['posZ']} {kin['velY']} "
                 f"{kin['velZ']} {kin['accY']} {kin['accZ']}")
    for p in pts:
        lines.append(f"{p[0]} {p[1]} {p[2]} {p[3]}")

    # window reference (all points gate: within 0.75 of (0.1,2.5))
    d = pts[:, :2] - np.array([kin['posX'], kin['posY']])
    g = pts[(d[:, 0]**2 + d[:, 1]**2) <= GATE * GATE]
    if len(g) >= 2:
        wpts = [(x, y, world_h(y, z)) for x, y, z, _ in g]
        wo = wd.update(wpts)
        win_ref.append((fi, wo['down'], wo['h_s'], min(wo['low_run'], 255)))

    # mlp reference
    fr = build_frame(kin, pts)
    hist.append(fr)
    pushed = [h for h in hist if h is not None]
    if len(pushed) >= NF:
        flat = np.stack(pushed[-NF:]).T.reshape(-1)
        mlp_ref.append((fi, flat))

seqf = os.path.join(HERE, "_proc_seq.txt")
open(seqf, "w").write("\n".join(lines) + "\n")
PROC = os.environ.get("POSE_PROC", "pose_proc_test")
out = subprocess.run([PROC, seqf], capture_output=True, text=True)
frames = [b for b in out.stdout.split("---\n") if b.strip()]

ok = True
mlp_map = dict((f[0], f[1]) for f in mlp_ref)
win_map = dict((w[0], w[1:]) for w in win_ref)
print(f"C emitted {len(frames)} frames; MLP-ref {len(mlp_ref)}, window-ref {len(win_ref)}")
for fi in range(14):
    c = frames[fi].strip().split() if fi < len(frames) else []
    if len(c) != 8:
        ok = False; print(f"  frame {fi}: bad C output {c}"); continue
    cpose, cfp, cvalid = int(c[1]), int(c[2]), int(c[3])
    cdown, chs, clow, cwv = int(c[4]), int(c[5]), int(c[6]), int(c[7])

    if fi in win_map:
        wdown, whs, wlow = win_map[fi]
        wmatch = (cwv == 1 and cdown == int(wdown) and clow == wlow
                  and abs(chs - round(whs * 100)) <= 1)
        ok &= wmatch
        tag = "ok" if wmatch else "MISMATCH"
        print(f"  frame {fi}: WIN C(down={cdown},hs={chs}cm,low={clow}) "
              f"ref(down={int(wdown)},hs={round(whs*100)}cm,low={wlow}) {tag}")

    if fi in mlp_map:
        prob = folded_forward(layers, mlp_map[fi].astype(np.float64))
        ep, efp = int(prob.argmax()), int(prob[3] * 255 + 0.5)
        mmatch = (cvalid == 1 and cpose == ep and abs(cfp - efp) <= 1)
        ok &= mmatch
        print(f"  frame {fi}: MLP C(pose={cpose},fp={cfp}) ref(pose={ep},fp={efp}) "
              f"{'ok' if mmatch else 'MISMATCH'}")

print("\nRESULT:", "MATCH" if ok else "MISMATCH")
sys.exit(0 if ok else 1)
