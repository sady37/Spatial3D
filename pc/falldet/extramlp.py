"""ExtraMLP runtime — the always-on "is a person LYING on the floor here" classifier.

Loads the logistic weights exported by pose/extramlp_train.py and scores a per-cluster
feature dict with numpy only (no sklearn at runtime). This is the PRIMARY lying verdict:
3001 height (hi2/floorfrac/yspan) says lie-vs-stand; the 18 s cube (energy variance / point
count) is a weak person-vs-furniture aux. cube RR VALUE is band-pinned (~28/min across labels)
and carries ~zero weight -- see the exported note. When no cube was fetched (0-18 s of the
3001-first tier) the cube features impute to absent and the height backbone decides.

Design (per EXTRAMLP_BRIEF, user 2026-07-19/20): STATE not action -- it does not care whether a
fall descent happened, only whether a body is on the floor now.
"""
import os, json, math

_HERE = os.path.dirname(os.path.abspath(__file__))
_DEFAULT = os.path.join(_HERE, "extramlp_weights.json")   # weights live WITH the loader (falldet/)


class ExtraMLP:
    def __init__(self, weights_path=None):
        self.ok = False
        try:
            w = json.load(open(weights_path or _DEFAULT))
            self.feats = w["feats"]
            self.mean = w["mean"]; self.scale = w["scale"]
            self.coef = w["coef"]; self.intercept = float(w["intercept"])
            self.impute = w["impute"]; self.use_cube = bool(w.get("use_cube"))
            self.ok = True
        except Exception as e:
            self._err = str(e)

    def _raw(self, feat, name):
        """Map a feature name to its raw value from the feat dict, imputing when absent, and
        apply the same log1p transforms the trainer used."""
        if name == "log_n3001":
            v = feat.get("n3001"); v = self.impute["n3001"] if v is None else v
            return math.log1p(float(v))
        if name == "log_ncube":
            v = feat.get("ncube"); v = self.impute["ncube"] if v is None else v
            return math.log1p(float(v))
        if name == "log_cube_var":
            v = feat.get("cube_var"); v = self.impute["cube_var"] if v is None else v
            return math.log1p(float(v))
        if name == "rr_present":
            rr = feat.get("rr"); rr = self.impute["rr"] if rr is None else rr
            return 1.0 if float(rr) > 0 else 0.0
        v = feat.get(name)
        return float(self.impute.get(name, 0.0) if v is None else v)

    def p_lie(self, feat):
        """feat: {hi2, floorfrac, micro, yspan, n3001, rr, rr_str, ncube, cube_var} (any absent ->
        imputed). Returns P(person lying on floor) in [0,1], or None if weights failed to load."""
        if not self.ok:
            return None
        z = self.intercept
        for name, mu, sd, c in zip(self.feats, self.mean, self.scale, self.coef):
            x = (self._raw(feat, name) - mu) / (sd if sd else 1.0)
            z += c * x
        return 1.0 / (1.0 + math.exp(-max(-40.0, min(40.0, z))))


if __name__ == "__main__":
    # self-test: reproduce the sklearn LOO-refit predictions on the training set.
    import numpy as np
    ds = os.path.join(_HERE, "..", "record", "extramlp_dataset.npz")
    d = np.load(ds, allow_pickle=True)
    m = ExtraMLP()
    if not m.ok:
        raise SystemExit(f"weights load FAILED: {m._err}")
    agree = tot = 0
    for i in range(len(d["y"])):
        y = int(d["y"][i])
        if y not in (0, 1):
            continue
        feat = {k: (None if d[k][i] is None else d[k][i]) for k in
                ["hi2", "floorfrac", "micro", "yspan", "n3001", "rr", "rr_str", "ncube", "cube_var"]}
        p = m.p_lie(feat)
        pred = 1 if p >= 0.5 else 0
        agree += int(pred == y); tot += 1
    print(f"ExtraMLP numpy-runtime train-fit accuracy: {agree}/{tot} = {agree/tot:.3f} "
          f"(full-refit, expect ~1.0 -- LOO was 0.88)")
