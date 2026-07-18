"""Scene-level fall feature extractor + semi-auto labeler (ENHANCED_MLP_BRIEF Task 2).

Runs the REAL server `_scene()` frame-by-frame over the recorded npz scenes (reusing the
`fall_replay` harness, so every feature is exactly what the live server computes) and dumps a
per-frame feature table. This is the training/analysis substrate for the server-side ENHANCED
pose/fall fusion: the 6 feature classes the firmware MLP can't see, plus the code-of-record
decision, the cardiac/collapse flag, and the distinct-event id.

Label (semi-auto, NON-circular): the physical CLOUD-HEIGHT ground truth — a frame is "fallen"
when the whole-cloud centroid world height (`f_height`, cloud_wz_med) sits below FALLEN_Z for a
sustained run. This is independent of the fall DECISION legs (window / cube / sustained), so a
model trained to predict it from the OTHER features is not just re-learning the code's output.

Feature classes (per the brief):
  1 TI MLP           f_mlp     falling_prob (0 in every current recording -- firmware not emitting)
  2 below-floor E    f_energy  cloud_below_frac (fraction of cloud below the floor band)
  3 cloud height     f_height  cloud_wz_med (whole-cloud median world Z)
  4 temporal window  f_win + down_dur  (sustained-down state + how long)
  5 RR + absence     f_rr + rr_absent  (breathing present; and the ⭐ no-RR-while-fallen flag)
  6 geometry         f_geom    XY/XZ/YZ flatness aspect (lying flat vs standing column)
  + context          floor_fall, real, prim_n, cube_bursts, collapse, event

Usage:
  python3 pose/scene_features.py extract case/fall_*.npz -o record/scene_feats.npz
  python3 pose/scene_features.py train   record/scene_feats.npz     # LORO-CV logistic fusion
"""
import os, sys, argparse, glob
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))            # pc/
import web.fall_replay as fr                          # the code-of-record replay harness
import web.radar_server as srv

FALLEN_Z = 0.0        # cloud centroid world height below this (m) = mass on the floor => fallen
FALLEN_SUSTAIN = 5    # frames the height must stay below FALLEN_Z to label the run "fallen"
                      # (a single dip is a fragment/flicker, not a fall)

# The 6-feature-class vector we expose per frame (order fixed; see module docstring).
FEAT_COLS = ["f_mlp", "f_energy", "f_height", "f_win", "down_dur", "f_rr",
             "rr_absent", "f_geom", "floor_fall", "real", "prim_n", "cube_bursts"]
# Features fed to the fusion model. f_energy / f_height are EXCLUDED (they derive from the same
# cloud as the height label -> leakage); the model must predict "fallen" from the independent
# legs: MLP, window+duration, RR/absence, geometry, floor-fall.
MODEL_COLS = ["f_mlp", "f_win", "down_dur", "f_rr", "rr_absent", "f_geom", "floor_fall"]


def extract_one(path, mount=2.0, tilt=25.0):
    """Drive srv._scene() over every frame of one npz; return a per-frame feature dict-of-arrays.
    Mirrors fall_replay.run()'s loop but keeps the FULL fall_ev feature vector."""
    d = np.load(path)
    srv.MOUNT = mount; srv.TILT = tilt
    clock = fr._Clock(); srv.time = clock
    srv.threading.Thread = fr._SyncThread
    srv.print = lambda *a, **k: None
    srv.open = lambda *a, **k: fr._Sink()
    fake = fr.FakeSource(d); srv._src = fake
    fr._reset_state()
    t0 = float(d["ts"][0])
    rows = []
    for fi in range(fake.nfr):
        fake.fi = fi
        clock.t = float(d["ts"][fi])
        out = srv._scene()
        ev = out["fall_ev"]
        h = ev.get("f_height")
        rows.append({
            "t": round(clock.t - t0, 2),
            "f_mlp": float(ev.get("f_mlp") or 0.0),
            "f_energy": float(ev.get("f_energy") or 0.0),
            "f_height": (np.nan if h is None else float(h)),
            "f_win": float(ev.get("f_win") or 0),
            "down_dur": float(ev.get("down_dur") or 0.0),
            "f_rr": float(ev.get("f_rr") or 0),
            "rr_absent": float(1.0 - (ev.get("f_rr") or 0)),   # ⭐ no breathing = the signal
            "f_geom": float(ev.get("f_geom") or 0.0),
            "floor_fall": float(bool(ev.get("floor_fall"))),
            "real": float(bool(ev.get("real"))),
            "prim_n": float(ev.get("prim_n") or 0),
            "cube_bursts": float(ev.get("cube_bursts") or 0),
            "fall_state": out["fall_state"],
            "collapse": int(bool(out.get("collapse_suspect"))),
            "event": int(out.get("fall_event") or 0),
        })
    return rows


def label_fallen(rows):
    """Physical cloud-height label: fallen=1 on any sustained run of f_height < FALLEN_Z.
    Independent of the decision legs -> a non-circular target for the fusion model."""
    below = np.array([(not np.isnan(r["f_height"])) and r["f_height"] < FALLEN_Z for r in rows])
    y = np.zeros(len(rows), int)
    run = 0
    for i, b in enumerate(below):
        run = run + 1 if b else 0
        if run >= FALLEN_SUSTAIN:
            y[i - run + 1:i + 1] = 1        # backfill the whole sustained run
    return y


def cmd_extract(args):
    files = []
    for pat in args.paths:
        files.extend(sorted(glob.glob(pat)))
    if not files:
        print("no files matched", args.paths); return
    X, Y, rec, tcol = [], [], [], []
    fs_meta = []
    for path in files:
        rows = extract_one(path, args.mount, args.tilt)
        y = label_fallen(rows)
        name = os.path.basename(path)
        for r, yy in zip(rows, y):
            X.append([r[c] for c in FEAT_COLS]); Y.append(int(yy))
            rec.append(name); tcol.append(r["t"])
        n_fall = int(np.sum(y)); n_col = sum(r["collapse"] for r in rows)
        n_ev = rows[-1]["event"] if rows else 0
        fs_meta.append((name, len(rows), n_fall, n_ev, n_col))
        print(f"  {name:40s} {len(rows):5d} frames  fallen={n_fall:4d}  events={n_ev}  collapse_frames={n_col}")
    X = np.array(X, float); Y = np.array(Y, int)
    out = args.out
    np.savez(out, X=X, Y=Y, rec=np.array(rec), t=np.array(tcol, float),
             feat_cols=np.array(FEAT_COLS), model_cols=np.array(MODEL_COLS))
    print(f"\nsaved {X.shape[0]} frames x {X.shape[1]} feats -> {out}")


def cmd_train(args):
    """Leave-One-Recording-Out logistic fusion. Small-data SANITY CHECK, not a production model:
    with ~7 fall recordings this validates that the 6 features linearly predict the physical
    fallen-state and that the learned weights agree in SIGN/rank with the hand-tuned _fall_fuse."""
    from sklearn.linear_model import LogisticRegression
    from sklearn.preprocessing import StandardScaler
    from sklearn.metrics import roc_auc_score
    d = np.load(args.npz, allow_pickle=True)
    X_all, Y, rec = d["X"], d["Y"], d["rec"]
    feat_cols = list(d["feat_cols"]); model_cols = list(d["model_cols"])
    ci = [feat_cols.index(c) for c in model_cols]
    X = X_all[:, ci]
    recs = sorted(set(rec.tolist()))
    print(f"{X.shape[0]} frames, {len(model_cols)} model feats, {len(recs)} recordings")
    print(f"class balance: fallen={int(Y.sum())} ({100*Y.mean():.1f}%)\n")
    # Leave-One-Recording-Out CV (the honest generalization test on tiny data)
    aucs = []
    for held in recs:
        te = rec == held; trm = ~te
        if Y[trm].sum() == 0 or Y[trm].sum() == len(Y[trm]):
            continue
        sc = StandardScaler().fit(X[trm])
        clf = LogisticRegression(max_iter=1000, class_weight="balanced")
        clf.fit(sc.transform(X[trm]), Y[trm])
        if 0 < Y[te].sum() < len(Y[te]):
            auc = roc_auc_score(Y[te], clf.decision_function(sc.transform(X[te])))
            aucs.append((held, auc))
            print(f"  LORO hold={held:34s} test-AUC={auc:.3f}")
    if aucs:
        print(f"\n  mean LORO AUC = {np.mean([a for _, a in aucs]):.3f}  (>0.9 = features carry the signal)")
    # Full-data fit -> interpretable weights vs the hand-tuned fusion
    sc = StandardScaler().fit(X)
    clf = LogisticRegression(max_iter=1000, class_weight="balanced").fit(sc.transform(X), Y)
    print("\n  learned fusion weights (standardized; sign+rank vs hand-tuned _fall_fuse):")
    order = np.argsort(-np.abs(clf.coef_[0]))
    for i in order:
        print(f"    {model_cols[i]:12s} {clf.coef_[0][i]:+.3f}")
    print("\n  _fall_fuse hand weights: mlp .22 | win .20 | height .22 | energy .18 | geom .18"
          " | floor_fall->max .6 | rr present x1.0/absent x0.7")
    print("  NOTE: ~7 recordings is a calibration/sanity check, not a deployable classifier."
          "\n  The production wins (collapse-suspect flag, event counter) need NO training.")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    pe = sub.add_parser("extract"); pe.add_argument("paths", nargs="+")
    pe.add_argument("-o", "--out", default="record/scene_feats.npz")
    pe.add_argument("--mount", type=float, default=2.0); pe.add_argument("--tilt", type=float, default=25.0)
    pe.set_defaults(func=cmd_extract)
    pt = sub.add_parser("train"); pt.add_argument("npz")
    pt.set_defaults(func=cmd_train)
    a = ap.parse_args(); a.func(a)
