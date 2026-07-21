#!/usr/bin/env python3
"""Train + EXPORT the ExtraMLP ("is a person lying on the floor here") to a numpy-loadable
weights file the radar_server can run per-frame WITHOUT sklearn.

Two feature sets are compared under leave-one-out so the cube's real contribution is measured,
not assumed:
  A  3001-only : [hi2, floorfrac, micro, yspan, log_n3001]        (height = lie vs stand)
  B  3001+cube : A + [rr_present, rr_str, log_ncube, log_cube_var] (cube = person vs furniture)

Writes record/extramlp_weights.json = {feats, mean, scale, coef, intercept, impute} for the
chosen set. Run from pc/ .  python3 pose/extramlp_train.py"""
import os, json, math
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); REC = os.path.join(HERE, "..", "record")
DS = os.path.join(REC, "extramlp_dataset.npz")
OUT = os.path.join(REC, "extramlp_weights.json")

# imputation for absent 3001 (n3001=0 -> upright-like, no body on floor) and absent cube.
IMPUTE = {"hi2": 2.0, "floorfrac": 0.0, "micro": 0.0, "yspan": 0.0, "n3001": 0.0,
          "rr": 0.0, "rr_str": 0.0, "ncube": 0.0, "cube_var": 0.0}


def _get(o, k):
    v = o[k]
    return IMPUTE[k] if v is None else float(v)


def fvec(o, use_cube):
    hi2, ff, micro = _get(o, "hi2"), _get(o, "floorfrac"), _get(o, "micro")
    ysp, n3 = _get(o, "yspan"), _get(o, "n3001")
    v = [hi2, ff, micro, ysp, math.log1p(n3)]
    if use_cube:
        rr = _get(o, "rr"); rr_str = _get(o, "rr_str")
        nc = _get(o, "ncube"); cvar = _get(o, "cube_var")
        v += [1.0 if rr > 0 else 0.0, rr_str, math.log1p(nc), math.log1p(cvar)]
    return v


NAMES_A = ["hi2", "floorfrac", "micro", "yspan", "log_n3001"]
NAMES_B = NAMES_A + ["rr_present", "rr_str", "log_ncube", "log_cube_var"]


def loo(X, Y):
    from sklearn.linear_model import LogisticRegression
    from sklearn.preprocessing import StandardScaler
    from sklearn.model_selection import LeaveOneOut
    pred = np.zeros(len(Y))
    for tr, te in LeaveOneOut().split(X):
        sc = StandardScaler().fit(X[tr])
        clf = LogisticRegression(max_iter=1000, C=1.0).fit(sc.transform(X[tr]), Y[tr])
        pred[te] = clf.predict(sc.transform(X[te]))
    return (pred == Y).mean(), pred


def main():
    d = np.load(DS, allow_pickle=True)
    rows = [{k: d[k][i] for k in d.files} for i in range(len(d["y"]))]
    binr = [o for o in rows if int(o["y"]) in (0, 1)]
    Y = np.array([int(o["y"]) for o in binr])
    print(f"binary samples: {len(binr)}  (lie={int((Y==1).sum())}, upright/empty={int((Y==0).sum())})\n")

    for use_cube, names in ((False, NAMES_A), (True, NAMES_B)):
        X = np.array([fvec(o, use_cube) for o in binr])
        acc, pred = loo(X, Y)
        errs = [str(binr[i]["label"])[:22] for i in range(len(Y)) if pred[i] != Y[i]]
        tag = "3001+cube" if use_cube else "3001-only"
        print(f"[{tag:9s}] LOO acc={acc:.3f}  errors={errs}")

    # choose 3001-only unless cube demonstrably helps (it does not on this set); export it.
    from sklearn.linear_model import LogisticRegression
    from sklearn.preprocessing import StandardScaler
    # LYING model is 3001-ONLY by design: RR detects person-vs-furniture, NOT lie-vs-stand
    # (user 2026-07-20; brief §31). Height (hi2/floorfrac/yspan) is what says "on the floor".
    # cube is applied SEPARATELY as a person-gate + far-lying rescue, not as a lying feature.
    use_cube = False
    names = NAMES_B if use_cube else NAMES_A
    X = np.array([fvec(o, use_cube) for o in binr])
    sc = StandardScaler().fit(X)
    clf = LogisticRegression(max_iter=1000).fit(sc.transform(X), Y)
    print("\nexported weights:", {n: round(float(w), 2) for n, w in zip(names, clf.coef_[0])})
    payload = {
        "feats": names, "use_cube": use_cube,
        "mean": [float(v) for v in sc.mean_],
        "scale": [float(v) for v in sc.scale_],
        "coef": [float(v) for v in clf.coef_[0]],
        "intercept": float(clf.intercept_[0]),
        "impute": IMPUTE,
        "note": "LYING model is 3001-only: RR = person-vs-furniture, NOT lie-vs-stand (user 2026-07-20, "
                "brief §31); rr is band-center-pinned (~28/min) across labels here anyway. Height "
                "(hi2/floorfrac/yspan) says on-the-floor. cube is applied by the CALLER as a separate "
                "person-gate + far-lying rescue (3001 absent + cube micro-motion/RR), not a lying feature.",
    }
    json.dump(payload, open(OUT, "w"), indent=2)
    print(f"saved -> {OUT}")


if __name__ == "__main__":
    main()
