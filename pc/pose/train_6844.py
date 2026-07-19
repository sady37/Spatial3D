"""Retrain the point-centroid pose MLP on OUR 6844 recordings (npz), auto-labelled by the
cloud geometry -- NO manual labels, NO TI CSV. This closes the transfer gap the point-centroid
experiment exposed: TI's model (6432 geometry, real snr) can't transfer as-is (Lying collapsed
into Sat), so we retrain on 6844 geometry (our mount/tilt, snr faked constant = the 5 snr feats
are dead here too, matching what the firmware sees from a cloud with no per-point snr).

Features are computed EXACTLY as firmware poseBuildFrame will (gate 0.75 m to the track, top-5
by z, centroid cz = mean top-5 z, velZ = ring frame-diff * fps) so train == on-chip. The
kinematics come from the CLOUD (reliable), not the tracker (posZ floats, velZ EKF-smoothed).

Auto-label per frame from the cloud WORLD height + its drop-rate (geometry-invariant velZ for
Falling, height bands for the static poses, tuned to mount=2.0 / tilt=25):
  Falling : world drop-rate < FALL_VZ           (the fall MOTION)
  Stood   : world height > STOOD_Z
  Sat     : SAT_Z < height <= STOOD_Z
  Lying   : height <= SAT_Z

Usage: python3 pose/train_6844.py case/fall_172500.npz record/live_scene_20260718_170000.npz \
                                   [--export pose/pose_model_6844.c]
"""
import argparse, os, sys, math
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pose.dataset import WINDOW_SIZE, CLASS_NAMES        # ["Stood","Sat","Lying","Falling"], 8

FPS = 10.0
GATE = 0.75                      # firmware POSE_GATE_RADIUS_M
NTOP = 5                         # firmware POSE_NUM_POINTS
NFEAT = 20
MOUNT, TILT = 2.0, math.radians(25.0)
# auto-label thresholds (world height, m) for our 2.0 m / 25 deg rig (measured on the falls)
STOOD_Z, SAT_Z, FALL_VZ = 0.60, 0.15, -1.5
FALL_NET_DROP = 0.30             # a Falling window must also have a NET world-height descent this
                                 # big across its 8 frames -- rejects noise spikes in the 5-pt
                                 # centroid (a flat lie with a 1-frame jitter isn't a fall).


def _world(py, pz):
    return MOUNT + pz * math.cos(TILT) - py * math.sin(TILT)     # world height


def extract(path):
    """Per-track windowed feature vectors (M,160) feature-major + labels (M,) from one npz.
    Features match firmware poseBuildFrame; labels from the cloud world-height/drop-rate."""
    d = np.load(path)
    tf, tid, tx, ty = d["t_frame"], d["t_tid"], d["t_x"], d["t_y"]
    pf, pc = d["p_frame"], d["pc_xyz"]
    X, Y = [], []
    for tk in sorted(set(tid.tolist())):
        # this track's frames in order
        fr = sorted(tf[tid == tk].tolist())
        pos = {int(f): (float(tx[(tf == f) & (tid == tk)][0]),
                        float(ty[(tf == f) & (tid == tk)][0])) for f in fr}
        feats, wz_seq, labs_static = {}, {}, {}
        for f in fr:
            cl = pc[pf == f]
            if not len(cl):
                continue
            gx, gy = pos[f]
            m = (cl[:, 0] - gx) ** 2 + (cl[:, 1] - gy) ** 2 <= GATE ** 2
            g = cl[m]
            if len(g) < NTOP:
                continue
            top = g[np.argsort(g[:, 2])[-NTOP:]]        # 5 highest by z (ascending)
            cz = float(top[:, 2].mean())                # firmware: mean top-5 z (radar frame)
            cy = float((top[:, 1] - gy).mean())         # centroid relative range
            p15 = []
            for r in top:                               # (y-posy, z, snr=0 -- no per-point snr)
                p15 += [float(r[1] - gy), float(r[2]), 0.0]
            feats[f] = {"cz": cz, "cy": cy, "p15": p15}
            wz_seq[f] = float(np.median(_world(g[:, 1], g[:, 2])))   # world height (for label)
        # kinematics from the centroid sequence (contiguous frames only)
        ordered = [f for f in fr if f in feats]
        vec = {}
        for i, f in enumerate(ordered):
            fp = ordered[i - 1] if i > 0 else None
            contig = fp is not None and f - fp == 1
            czf, cyf = feats[f]["cz"], feats[f]["cy"]
            if contig:
                velZ = (czf - feats[fp]["cz"]) * FPS
                velY = (cyf - feats[fp]["cy"]) * FPS
                wvz = (wz_seq[f] - wz_seq[fp]) * FPS
            else:
                velZ = velY = wvz = 0.0
            vec[f] = {"velZ": velZ, "velY": velY, "wvz": wvz}
        # accel needs prev velZ
        feat20 = {}
        for i, f in enumerate(ordered):
            fp = ordered[i - 1] if i > 0 else None
            contig = fp is not None and f - fp == 1
            accZ = (vec[f]["velZ"] - vec[fp]["velZ"]) * FPS if contig else 0.0
            accY = (vec[f]["velY"] - vec[fp]["velY"]) * FPS if contig else 0.0
            feat20[f] = [feats[f]["cz"], vec[f]["velY"], vec[f]["velZ"], accY, accZ] + feats[f]["p15"]
        # auto-label + window (feature-major, last frame's label)
        for i in range(len(ordered) - WINDOW_SIZE + 1):
            win = ordered[i:i + WINDOW_SIZE]
            if win[-1] - win[0] != WINDOW_SIZE - 1:      # need 8 contiguous frames
                continue
            fl = win[-1]
            wz = wz_seq[fl]
            # ⭐ Falling label = the fastest downward drop ANYWHERE in the 8-frame window (not just
            # the last frame). A fall's descent happens mid-window; by the last frame the body may
            # have already landed (wvz~0) -> the old last-frame-only test mislabelled real falls as
            # Lying, starving the Falling class. Peak-drop labels the whole descent window Falling.
            peak_drop = min(vec[f]["wvz"] for f in win)
            net_drop = wz_seq[win[0]] - wz_seq[win[-1]]     # +ve = descended over the window
            if peak_drop < FALL_VZ and net_drop > FALL_NET_DROP:
                lab = 3    # Falling: a rapid drop AND a real net descent (not centroid jitter)
            elif wz > STOOD_Z:      lab = 0    # Stood
            elif wz > SAT_Z:        lab = 1    # Sat
            else:                   lab = 2    # Lying
            # feature-major: gPoseIn[j*8+k] = feat[k][j]
            v = np.array([[feat20[f][j] for f in win] for j in range(NFEAT)], float).reshape(-1)
            X.append(v); Y.append(lab)
    return (np.array(X), np.array(Y)) if X else (np.zeros((0, NFEAT * WINDOW_SIZE)), np.zeros(0, int))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("npz", nargs="+")
    ap.add_argument("--epochs", type=int, default=300)
    ap.add_argument("--export", default=None)
    a = ap.parse_args()
    Xs, Ys, Gs = [], [], []
    for gi, p in enumerate(a.npz):
        X, Y = extract(p)
        print(f"{os.path.basename(p):34s} windows={len(Y):5d}  "
              + " ".join(f"{CLASS_NAMES[i]}={int((Y==i).sum())}" for i in range(4)))
        if len(Y): Xs.append(X); Ys.append(Y); Gs.append(np.full(len(Y), gi))
    X = np.vstack(Xs); Y = np.concatenate(Ys); G = np.concatenate(Gs)
    print(f"\nTOTAL windows={len(Y)}  balance=" + str({CLASS_NAMES[i]: int((Y==i).sum()) for i in range(4)}))

    # grouped-by-recording CV (honest: does point-centroid classify OUR geometry?). Uses sklearn
    # MLPClassifier (64,32,16, ReLU) == the firmware net's linear structure (torch is unusable here:
    # numpy 2.x breaks torch 2.2's from_numpy bridge). OVERSAMPLE the minority classes per train fold
    # so Falling isn't drowned by the larger Sat class. Same fit+export path -> CV == what ships. The
    # StandardScaler is FOLDED into layer 0 at export so the firmware feeds RAW point-centroid feats.
    from pathlib import Path
    from sklearn.model_selection import GroupKFold
    from sklearn.neural_network import MLPClassifier
    from sklearn.preprocessing import StandardScaler
    from pose.export_c import write_c

    def oversample(Xt, Yt):
        mx = max(int((Yt == i).sum()) for i in range(4))
        idx = []
        for i in range(4):
            ii = np.where(Yt == i)[0]
            if len(ii):
                idx.append(np.random.RandomState(42).choice(ii, mx, replace=len(ii) < mx))
        idx = np.concatenate(idx)
        return Xt[idx], Yt[idx]

    def fit(Xt, Yt):
        sc = StandardScaler().fit(Xt)
        clf = MLPClassifier((64, 32, 16), activation="relu", max_iter=a.epochs,
                            alpha=1e-3, random_state=42).fit(sc.transform(Xt), Yt)
        return sc, clf

    ngroups = len(set(G.tolist()))
    if ngroups >= 2:
        cm = np.zeros((4, 4), int)
        for tr, te in GroupKFold(n_splits=min(5, ngroups)).split(X, Y, G):
            Xo, Yo = oversample(X[tr], Y[tr])
            sc, clf = fit(Xo, Yo)
            for t, p in zip(Y[te], clf.predict(sc.transform(X[te]))):
                cm[t, p] += 1
        print(f"\ngrouped-CV confusion (rows=true) -- oversampled MLP(64,32,16):")
        print(f"    {'true/pred':>10s} " + " ".join(f"{c:>8s}" for c in CLASS_NAMES) + "   recall")
        for i, c in enumerate(CLASS_NAMES):
            tot = cm[i].sum(); rec = f"{cm[i,i]/tot*100:.0f}%" if tot else "(none)"
            print(f"    {c:>10s} " + " ".join(f"{cm[i,j]:8d}" for j in range(4)) + f"   {rec}")
        print(f"    overall acc = {np.trace(cm)/cm.sum()*100:.1f}%")

    if a.export:
        Xo, Yo = oversample(X, Y)
        sc, clf = fit(Xo, Yo)
        # sklearn coefs_[i] = (in,out); firmware wants (out,in). Layer 0 absorbs the scaler:
        # W0*(x-mean)/scale + b0 = (W0/scale)*x + (b0 - W0*(mean/scale)).
        mean, scale = sc.mean_, sc.scale_
        W0 = clf.coefs_[0].T                       # (64,160)
        b0 = clf.intercepts_[0] - W0 @ (mean / scale)
        W0 = W0 / scale
        layers = [(W0, b0)] + [(clf.coefs_[i].T, clf.intercepts_[i]) for i in range(1, 4)]
        # verify the folded (firmware) path matches sklearn on the raw features
        def fwd(layers, x):
            h = x
            for k, (W, b) in enumerate(layers):
                h = h @ W.T + b
                if k < len(layers) - 1:
                    h = np.maximum(h, 0.0)
            e = np.exp(h - h.max(1, keepdims=True)); return e / e.sum(1, keepdims=True)
        ref = clf.predict_proba(sc.transform(X))
        err = float(np.abs(ref - fwd(layers, X.astype(np.float64))).max())
        print(f"\nexport: scaler-fold check max|sklearn-folded|={err:.2e} ({'OK' if err < 1e-4 else 'MISMATCH'})")
        if err < 1e-4:
            write_c(Path(a.export), [(np.asarray(W, np.float64), np.asarray(b, np.float64)) for W, b in layers])
            print(f"  wrote {a.export}  (firmware feeds RAW point-centroid features; scaler folded into layer 0)")


if __name__ == "__main__":
    main()
