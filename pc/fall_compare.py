"""Integration test / head-to-head on TI's labeled set, wiring the three falldet modules:
  falldet.mlp     (Module 2)  — TI-style learned classifier
  falldet.window  (Module 1)  — sustained max-height trigger (ports to DSP)
  falldet.clean   (Module 3)  — fuse + clean

Split by RECORDING. Reports each module's sitting-FP / lying-recall, and the cleaned
fusion, on the SAME test recordings.

    .venv/bin/python3 fall_compare.py <ti_ds_dir>
"""
import sys, os, csv, glob
import numpy as np
from falldet.mlp import MLPDetector, frame_features, WIN
from falldet.window import WindowDetector, FloorMap
from falldet.clean import Cleaner


def parse(path):
    """Format-agnostic. Per-frame dict: posz, vel(3), acc(3), points[(x,y,z,snr)]."""
    frames = []
    with open(path, encoding="utf-8-sig") as f:
        r = csv.reader(f); hdr = next(r)
        col = {h.strip().lower(): i for i, h in enumerate(hdr)}
        quads = []
        for name, zi in col.items():
            if name.startswith("pointz"):
                sfx = name[6:]
                xi, yi, si = col.get("pointx" + sfx), col.get("pointy" + sfx), col.get("snr" + sfx)
                if yi is not None and si is not None:
                    quads.append((xi, yi, zi, si))       # xi may be None (2D formats)
        for row in r:
            def g(k):
                i = col.get(k)
                if i is None or i >= len(row) or not row[i].strip(): return None
                try: return float(row[i])
                except: return None
            posz = g("posz")
            if posz is None: continue
            pts = []
            for xi, yi, zi, si in quads:
                if zi < len(row) and row[zi].strip():
                    try:
                        x = float(row[xi]) if (xi is not None and row[xi].strip()) else 0.0
                        pts.append((x, float(row[yi]), float(row[zi]), float(row[si])))
                    except: pass
            if len(pts) < 2: continue
            frames.append({"posz": posz, "posy": g("posy") or 0.0,
                           "vel": (g("velx") or 0, g("vely") or 0, g("velz") or 0),
                           "acc": (g("accx") or 0, g("accy") or 0, g("accz") or 0),
                           "points": pts})
    return frames


def main():
    ds = sys.argv[1] if len(sys.argv) > 1 else "."
    classes = ["sitting", "lying", "falling"]
    recs = {c: sorted(glob.glob(os.path.join(ds, c, "*.csv"))) for c in classes}
    train, test = [], []
    for c in classes:
        for j, p in enumerate(recs[c]):
            (test if j % 3 == 0 else train).append((p, c))
    print(f"# train={len(train)} test={len(test)}  ({', '.join(c+':'+str(len(recs[c])) for c in classes)})")

    # ---- Module 2: MLP — build 8-frame windows, train ----
    def wins(split):
        Xs, hs, ys = [], [], []
        for p, c in split:
            frs = parse(p)
            feats = [frame_features(f["posz"], f["vel"], f["acc"], f["points"], f["posy"]) for f in frs]
            for i in range(len(feats) - WIN + 1):
                Xs.append(MLPDetector.window(feats[i:i + WIN]))
                hs.append([f["points"] for f in frs[i:i + WIN]])   # raw points per frame in window
                ys.append(c)
        return Xs, hs, ys
    Xtr, _, ytr = wins(train)
    Xte, Hte, yte = wins(test)
    mlp = MLPDetector().train(np.array(Xtr), np.array(ytr))
    yte = np.array(yte)

    # Module 1 calibration: POSITION-DEPENDENT floor map from all training points (x,y,z).
    fmap = FloorMap(cell=0.5).fit(
        [(x, y, z) for p, _ in train for f in parse(p) for (x, y, z, s) in f["points"]])

    down_mlp, down_win, fall_clean = [], [], []
    for X, Hwin in zip(Xte, Hte):
        m = mlp.predict(X)
        down_mlp.append(m["falling_p"] >= 0.5)
        wd = WindowDetector(fmap, sustain=5); wout = None
        for pts in Hwin:
            wout = wd.update([(x, y, z) for (x, y, z, s) in pts])   # (x,y,z) per point
        down_win.append(bool(wout and wout["down"]))
        c = Cleaner(persist=1).decide(wout, m)
        fall_clean.append(c["trigger"])
    down_mlp = np.array(down_mlp); down_win = np.array(down_win); fall_clean = np.array(fall_clean)

    print(f"# FloorMap cells={len(fmap.hg)} default={fmap.default:+.2f}")
    print(f"\n{'class':8s} {'#win':>6s} | {'MLP':>6s} | {'WINDOW':>7s} | {'CLEANED(OR)':>11s}")
    for c in classes:
        m = yte == c
        if not m.any(): continue
        print(f"{c:8s} {int(m.sum()):6d} | {down_mlp[m].mean()*100:5.1f}% | {down_win[m].mean()*100:6.1f}% | {fall_clean[m].mean()*100:10.1f}%")
    for name, dn in [("MLP", down_mlp), ("WINDOW", down_win), ("CLEANED", fall_clean)]:
        print(f"#   {name:7s}: sitting-FP={dn[yte=='sitting'].mean()*100:4.1f}%  lying-recall={dn[yte=='lying'].mean()*100:5.1f}%")


if __name__ == "__main__":
    main()
