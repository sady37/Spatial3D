#!/usr/bin/env python3
"""Align the handshake labels to the recording npz chunks and extract per-sample
features for the ExtraMLP ("is there a person LYING on the floor here").

Each label row (pc/record/neg_labels_<date>.jsonl) has {query_t (epoch), bin, label}.
We find the 5-min chunk whose `ts` spans query_t, take an ~8 s window around it, and pull:
  - 3001 (pc_xyz) near the bin's ground range -> height + floor + morphology + micro-motion
  - cube (e_vec at bin +-hw over the window) -> slow-time RR + energy + micro-motion
Label: lie/collapse -> 1 (person on floor), stand/sit/empty -> 0, half-kneel/side -> boundary.

Usage: python3 pose/extramlp_extract.py [pc/record/neg_labels_20260719.jsonl]
Run from pc/.  Writes pc/record/extramlp_dataset_<date>.npz + prints an alignment table."""
import os, sys, json, glob, math
import numpy as np

TH = math.radians(25.0); MOUNT = 2.0; STEP = 0.085; FLOOR_Z = 0.4
HERE = os.path.dirname(os.path.abspath(__file__)); REC = os.path.join(HERE, "..", "record")
LOGF = sys.argv[1] if len(sys.argv) > 1 else os.path.join(REC, "neg_labels_20260719.jsonl")
WIN = 8.0                      # seconds around the query to pull (burst was ~4 s @ query_t)


def label_class(lab):
    l = lab.lower()
    if "empty" in l or "无人" in l or "空" in l:            return 0, "empty"
    if "half-kneel" in l or "半跪" in l or "kneel-halfside" in l or "跪式半侧" in l:
        return -1, "boundary"                              # ambiguous collapse -> mark, don't force
    if "curl-sidefall" in l or "侧倒" in l:                 return 1, "collapse"
    if "lie" in l or "躺" in l:                             return 1, "lie"
    if "stand" in l or "站" in l or "sit" in l or "坐" in l: return 0, "upright"
    return -1, "unknown"


def load_chunks():
    idx = []
    for fn in sorted(glob.glob(os.path.join(REC, "live_scene_2026*.npz"))):
        try:
            d = np.load(fn)
            if "ts" not in d.files or len(d["ts"]) < 2:
                continue
            idx.append((float(d["ts"][0]), float(d["ts"][-1]), fn))
        except Exception:
            pass
    return idx


def chunk_for(t, idx):
    for a, b, fn in idx:
        if a - 1 <= t <= b + 1:
            return fn
    return None


def feats_3001(d, t0, t1, bin_b):
    """3001 features in [t0,t1] near the bin's ground range."""
    ts = d["ts"]; pf = d["p_frame"]; pxyz = d["pc_xyz"]
    fis = np.where((ts >= t0) & (ts <= t1))[0]
    m = np.isin(pf, fis)
    pts = pxyz[m]
    if len(pts) < 4:
        return dict(n3001=len(pts), hi2=None, medwz=None, floorfrac=None,
                    xspan=None, yspan=None, flat=None, micro=None)
    px, py, pz = pts[:, 0], pts[:, 1], pts[:, 2]
    wz = MOUNT + pz * math.cos(TH) - py * math.sin(TH)
    wy = py * math.cos(TH) + pz * math.sin(TH)
    near = np.abs(wy - bin_b * STEP) < 0.5          # points within ~0.5 m of the queried range
    if near.sum() < 4:
        near = np.ones(len(pts), bool)              # fall back to all points
    px, wz, wy = px[near], wz[near], wy[near]
    hi2 = float(np.percentile(wz, 90))               # robust "top of body" (rejects lone ceiling/noise)
    # micro-motion proxy: per-frame point-count variation near the bin
    pf_near = pf[m][near]
    cnts = [int((pf_near == fi).sum()) for fi in fis if (pf_near == fi).any()]
    micro = float(np.std(cnts)) if len(cnts) > 1 else 0.0
    return dict(n3001=int(near.sum()), hi2=round(hi2, 3), medwz=round(float(np.median(wz)), 3),
                floorfrac=round(float((wz < FLOOR_Z).mean()), 3),
                xspan=round(float(px.max() - px.min()), 3),
                yspan=round(float(wy.max() - wy.min()), 3),
                flat=round(float((wz.max() - wz.min()) / max(px.max() - px.min(), 0.1)), 3),
                micro=round(micro, 2))


def feats_cube(d, t0, t1, bin_b, hw=3):
    """Cube slow-time features at bin +-hw over [t0,t1]: per-bin ant0 FFT RR + energy."""
    if "e_frame" not in d.files:
        return dict(ncube=0, rr=None, rr_str=0.0, cube_var=None)
    ts = d["ts"]; ef = d["e_frame"]; eb = d["e_bin"]; ev = d["e_vec"]
    fis = np.where((ts >= t0) & (ts <= t1))[0]
    m = np.isin(ef, fis) & (np.abs(eb - bin_b) <= hw)
    idxs = np.where(m)[0]
    if len(idxs) < 8:
        return dict(ncube=len(idxs), rr=None, rr_str=0.0, cube_var=None)
    # group by bin, build ant0 slow-time series, FFT in 0.15-0.5 Hz
    best_rr, best_str = None, 0.0
    vars_ = []
    for b in np.unique(eb[idxs]):
        sel = idxs[eb[idxs] == b]
        vecs = np.array([ev[i] for i in sel])            # (K, n_ant) complex
        if vecs.ndim != 2 or vecs.shape[0] < 8:
            continue
        a0 = vecs[:, 0].astype(np.complex64)
        a0 = a0 - a0.mean()                              # remove DC
        vars_.append(float(np.var(np.abs(vecs))))
        K = len(a0); fps = 10.0
        sp = np.abs(np.fft.fft(a0 * np.hanning(K)))       # complex slow-time (I/Q) -> full FFT
        fr = np.fft.fftfreq(K, 1.0 / fps)
        band = (fr >= 0.15) & (fr <= 0.5)
        if band.sum() and sp.sum() > 0:
            pk = np.argmax(sp[band]); strg = float(sp[band][pk] / (sp.sum() + 1e-9))
            if strg > best_str:
                best_str, best_rr = strg, float(fr[band][pk] * 60.0)
    return dict(ncube=len(idxs), rr=(None if best_rr is None else round(best_rr, 1)),
                rr_str=round(best_str, 3), cube_var=(round(float(np.mean(vars_)), 2) if vars_ else None))


def main():
    idx = load_chunks()
    rows = [json.loads(l) for l in open(LOGF)]
    rows = [r for r in rows if (r.get("entries") or 0) > 0]
    print(f"chunks={len(idx)}  labels(valid)={len(rows)}\n")
    print(f"{'query_hms':9s} {'label':30s} {'y':1s} {'bin':4s} | {'chunk':28s} {'n3001':5s} {'hi2':6s} {'floor':5s} {'micro':5s} | {'ncube':5s} {'rr':5s} {'str':5s}")
    out = []
    for r in rows:
        t = r["query_t"]; b = int(r["bin"]); lab = r["label"]
        y, cls = label_class(lab)
        # drop a mis-captured burst: the label is a lie but the query fell on an EMPTY fallback bin
        # (the lying track dropped -> auto-detect said empty -> queried the wrong bin30).
        if y == 1 and str(r.get("detected", "")).startswith("empty"):
            print(f"{r.get('query_hms','?'):9s} {lab[:30]:30s} -- DROP (bin-mismatch, detected empty)")
            continue
        fn = chunk_for(t, idx)
        if fn is None:
            print(f"{r.get('query_hms','?'):9s} {lab[:30]:30s} ?  {b:<4d} | NO CHUNK")
            continue
        d = np.load(fn)
        f3 = feats_3001(d, t - 2, t + WIN - 2, b)
        fc = feats_cube(d, t - 2, t + WIN - 2, b)
        rec = dict(query_t=t, label=lab, y=y, cls=cls, bin=b, chunk=os.path.basename(fn), **f3, **fc)
        out.append(rec)
        print(f"{r.get('query_hms','?'):9s} {lab[:30]:30s} {y:<2d} {b:<4d} | {os.path.basename(fn):28s} "
              f"{str(f3['n3001']):5s} {str(f3['hi2']):6s} {str(f3['floorfrac']):5s} {str(f3['micro']):5s} | "
              f"{str(fc['ncube']):5s} {str(fc['rr']):5s} {str(fc['rr_str']):5s}")
    # save
    if out:
        keys = list(out[0].keys())
        outp = os.path.join(REC, f"extramlp_dataset.npz")
        np.savez(outp, **{k: np.array([o[k] for o in out], dtype=object) for k in keys})
        print(f"\nsaved {len(out)} samples -> {outp}")
    train(out)


def _fvec(o):
    """Feature vector, imputing empty (n3001=0 -> upright-like, no body)."""
    hi2 = 2.0 if o["hi2"] is None else float(o["hi2"])
    ff = 0.0 if o["floorfrac"] is None else float(o["floorfrac"])
    micro = 0.0 if o["micro"] is None else float(o["micro"])
    ysp = 0.0 if o["yspan"] is None else float(o["yspan"])
    n3 = float(o["n3001"]) if o["n3001"] else 0.0
    return [hi2, ff, micro, ysp, math.log1p(n3)]


def train(out):
    binr = [o for o in out if o["y"] in (0, 1)]
    bnd = [o for o in out if o["y"] == -1]
    print(f"\n=== TRAIN: {len(binr)} binary (lie=1 vs upright/empty=0), {len(bnd)} boundary ===")
    X = np.array([_fvec(o) for o in binr]); Y = np.array([o["y"] for o in binr])
    names = ["hi2", "floorfrac", "micro", "yspan", "log_n3001"]
    # simple rule baseline
    rule = np.array([1 if (o["hi2"] is not None and o["hi2"] < 0.5 and (o["floorfrac"] or 0) > 0.7)
                     else 0 for o in binr])
    print(f"simple rule (hi2<0.5 & floor>0.7): acc={ (rule==Y).mean():.2f}  "
          f"errors={[binr[i]['label'][:24] for i in range(len(Y)) if rule[i]!=Y[i]]}")
    try:
        from sklearn.linear_model import LogisticRegression
        from sklearn.preprocessing import StandardScaler
        from sklearn.model_selection import LeaveOneOut
        loo = LeaveOneOut(); pred = np.zeros(len(Y))
        for tr, te in loo.split(X):
            sc = StandardScaler().fit(X[tr])
            clf = LogisticRegression(max_iter=500).fit(sc.transform(X[tr]), Y[tr])
            pred[te] = clf.predict(sc.transform(X[te]))
        print(f"logistic LOO: acc={ (pred==Y).mean():.2f}  "
              f"errors={[binr[i]['label'][:24] for i in range(len(Y)) if pred[i]!=Y[i]]}")
        sc = StandardScaler().fit(X); clf = LogisticRegression(max_iter=500).fit(sc.transform(X), Y)
        print("  weights:", {n: round(float(w), 2) for n, w in zip(names, clf.coef_[0])})
        for o in bnd:
            p = clf.predict_proba(sc.transform([_fvec(o)]))[0, 1]
            print(f"  boundary {o['label'][:30]:30s} -> P(lie)={p:.2f}  (hi2={o['hi2']} floor={o['floorfrac']})")
    except Exception as e:
        print("sklearn unavailable:", e)


if __name__ == "__main__":
    main()
