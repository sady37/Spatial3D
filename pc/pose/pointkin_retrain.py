"""Point-centroid retrain — prove the pose MLP works on POINT-derived kinematics
instead of the tracker's (smoothed, dead) posZ/velZ/accZ.

Root cause (onchip-pose-fall-map memory): the firmware MLP reads posZ/velZ/accZ from the
TRACKER, whose Z axis floats on a still body and is EKF-smoothed through a fall, so velZ≈0 and
the MLP never emits Falling/Lying. The raw points within the track gate carry the true vertical
signal (measured: point-centroid velZ -6.4 m/s at a fall vs tracker -2.8; posZ span ~3 m vs
~0.06 m). This script swaps ONLY the 5 kinematic features to point-centroid values and retrains,
keeping everything else identical, then compares grouped-CV recall to the tracker-feature model.

  posZ  <- mean z of the top-5 gated points        (was tracker posZ)
  velZ  <- (cz[t]-cz[t-1]) * fps                    (was tracker velZ)  [the fall signal, AUC .93]
  accZ  <- velZ[t]-velZ[t-1]                        (was tracker accZ)
  velY/accY <- same, from centroid (pointY-posY)    (was tracker velY/accY)
  the 15 point features (5x y-posy,z,snr) UNCHANGED.

Feature computation is IDENTICAL to what firmware poseBuildFrame would do (gate 0.75 m, top-5 by
z, centroid, ring-diff velZ), so train == firmware. Usage:
  python3 pose/pointkin_retrain.py --data <dir standing/sitting/lying/falling>
"""
import argparse
from pathlib import Path
import numpy as np, pandas as pd

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pose.dataset import (CLASS_DIRS, CLASS_NAMES, WINDOW_SIZE, MIN_POINTS, MAX_HEIGHT,
                          MIN_HEIGHT, MAX_DISTANCE, MIN_DISTANCE, TRACKER_COLS,
                          POSZ_DEAD_ABSMAX, _point_column_triples)
from sklearn.model_selection import GroupKFold

FPS = 10.0
NFEAT = 20
INPUT_SIZE = NFEAT * WINDOW_SIZE


def _frame_centroid_and_points(row, triples):
    """(cz, cy, point15) for one frame, or None. point15 = top-5 (y-posy,z,snr) ascending z.
    cz/cy = mean of the top-5 gated points -> the body-height / range summary the tracker floats."""
    posy = row["posy"]
    pts = []
    for py, pz, ps in triples:
        y, z, s = row[py], row[pz], row[ps]
        if np.isnan(y) or np.isnan(z) or np.isnan(s):
            continue
        pts.append((y - posy, z, s))
    if not pts:
        return None
    p = np.asarray(pts, float)
    keep = ((p[:, 0] <= MAX_DISTANCE) & (p[:, 0] >= MIN_DISTANCE)
            & (p[:, 1] <= MAX_HEIGHT) & (p[:, 1] >= MIN_HEIGHT))
    p = p[keep]
    if p.shape[0] < MIN_POINTS:
        return None
    p = p[np.argsort(p[:, 1], kind="stable")][-MIN_POINTS:]   # top-5 highest, ascending
    cz = float(p[:, 1].mean())          # POINT-centroid height (replaces tracker posZ)
    cy = float(p[:, 0].mean())          # POINT-centroid relative range
    return cz, cy, p.reshape(-1).astype(np.float32)


def load_file_pointkin(path):
    """CSV -> (features (M,20), frame idx (M,)) using point-centroid kinematics, or None."""
    df = pd.read_csv(path, encoding="utf-8", engine="python")
    missing = {c: 0.0 for c in TRACKER_COLS if c not in df.columns}
    if missing:
        df = pd.concat([df, pd.DataFrame(missing, index=df.index)], axis=1)
    pz = df["posz"].to_numpy(float, na_value=np.nan)
    pzv = pz[np.isfinite(pz) & (pz > MIN_HEIGHT) & (pz < MAX_HEIGHT)]
    if pzv.size == 0 or np.abs(pzv).max() < POSZ_DEAD_ABSMAX:   # same file-level reject as dataset.py
        return None
    triples = _point_column_triples(set(df.columns))
    if not triples:
        return None
    # pass 1: per-frame centroid + points
    cz, cy, pts, idx = [], [], [], []
    for i, (_, row) in enumerate(df.iterrows()):
        posz = row["posz"]
        if not (np.isfinite(posz) and MIN_HEIGHT <= posz <= MAX_HEIGHT):
            continue
        r = _frame_centroid_and_points(row, triples)
        if r is None:
            continue
        cz.append(r[0]); cy.append(r[1]); pts.append(r[2]); idx.append(i)
    if len(cz) < 2:
        return None
    cz = np.array(cz); cy = np.array(cy); idx = np.array(idx); pts = np.stack(pts)
    # pass 2: kinematics from the centroid sequence (contiguous frames only; else derivative=0)
    velz = np.zeros_like(cz); accz = np.zeros_like(cz)
    vely = np.zeros_like(cz); accy = np.zeros_like(cz)
    for t in range(1, len(cz)):
        if idx[t] - idx[t - 1] == 1:                 # contiguous
            velz[t] = (cz[t] - cz[t - 1]) * FPS
            vely[t] = (cy[t] - cy[t - 1]) * FPS
    for t in range(1, len(cz)):
        if idx[t] - idx[t - 1] == 1:
            accz[t] = velz[t] - velz[t - 1]
            accy[t] = vely[t] - vely[t - 1]
    feats = np.column_stack([cz, vely, velz, accy, accz, pts]).astype(np.float32)  # (M,20)
    return feats, idx


def window(feats, idx):
    out = []
    n = feats.shape[0]
    for s in range(n - WINDOW_SIZE + 1):
        if idx[s + WINDOW_SIZE - 1] - idx[s] != WINDOW_SIZE - 1:
            continue
        out.append(feats[s:s + WINDOW_SIZE].T.reshape(-1))    # feature-major
    return np.stack(out) if out else np.empty((0, INPUT_SIZE), np.float32)


def build_pointkin(root, verbose=True):
    X, y, g, files = [], [], [], []
    for label, dirname in CLASS_DIRS.items():
        kept = 0
        for path in sorted((root / dirname).glob("*.csv")):
            r = load_file_pointkin(path)
            if r is None:
                continue
            xs = window(*r)
            if xs.shape[0] == 0:
                continue
            X.append(xs); y.append(np.full(xs.shape[0], label)); g.append(np.full(xs.shape[0], len(files)))
            files.append(f"{dirname}/{path.name}"); kept += 1
        if verbose:
            print(f"  {CLASS_NAMES[label]:8s} files kept={kept}")
    return (np.concatenate(X), np.concatenate(y).astype(np.int64),
            np.concatenate(g).astype(np.int64), files)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, type=Path)
    ap.add_argument("--epochs", type=int, default=400)
    ap.add_argument("--export", type=Path, default=None)
    a = ap.parse_args()

    print("=== POINT-CENTROID kinematics retrain (track-Z-free) ===")
    X, y, g, files = build_pointkin(a.data)
    print(f"\ndataset: X={X.shape}  files={len(files)}")
    print("balance:", {CLASS_NAMES[i]: int((y == i).sum()) for i in range(4)})

    # grouped-by-file CV -- the honest number (does the MLP classify from point-centroid feats?)
    # sklearn MLP (64-32-16, same shape as the firmware net) -- avoids the torch/numpy-2 ABI break;
    # the question here is discriminability of the features, not the exact folded weights.
    from sklearn.neural_network import MLPClassifier
    from sklearn.preprocessing import StandardScaler
    print("\n" + "=" * 60)
    gkf = GroupKFold(n_splits=4)
    cms = np.zeros((4, 4), int); accs = []
    for k, (tr, te) in enumerate(gkf.split(X, y, groups=g)):
        sc = StandardScaler().fit(X[tr])
        clf = MLPClassifier(hidden_layer_sizes=(64, 32, 16), max_iter=a.epochs,
                            alpha=1e-3, random_state=42).fit(sc.transform(X[tr]), y[tr])
        pred = clf.predict(sc.transform(X[te]))
        accs.append(float((pred == y[te]).mean()))
        for aa, bb in zip(y[te], pred):
            cms[aa, bb] += 1
        print(f"  fold {k+1}/4 held-out files={len(set(g[te]))}  acc {accs[-1]*100:.1f}%")
    print(f"\n  GROUPED-BY-FILE 4-fold acc = {np.mean(accs)*100:.1f}% +/-{np.std(accs)*100:.1f}%")
    print(f"    {'true/pred':>10s} " + " ".join(f"{c:>8s}" for c in CLASS_NAMES) + "   recall")
    for i, c in enumerate(CLASS_NAMES):
        n = max(cms[i].sum(), 1)
        print(f"    {c:>10s} " + " ".join(f"{v:8d}" for v in cms[i]) + f"   {cms[i,i]/n*100:5.1f}%")
    print("\n  (tracker-feature baseline per memory: Falling-motion 96% / Lying 63%, overall ~98%)")


if __name__ == "__main__":
    main()
