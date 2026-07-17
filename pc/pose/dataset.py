"""Pose dataset prep for the 6844 firmware MLP (Phase 2).

Reproduces TI's Pose_And_Fall feature extraction (Pose_And_Fall_Detection/
retraining_resources/pose_and_fall_model_training.ipynb cell 6) with three
deliberate deviations, each of which fixes a defect measured in classes.zip:

1. DROP posz-dead files.  A large slice of classes.zip was recorded with the
   elevation axis unpopulated (|posz| never exceeds ~7 cm for a whole file).
   It splits cleanly by class -- 0.0% of standing/sitting/lying rows, but
   30.5% of falling and 51.2% of walking -- so "is the height axis dead" is a
   label shortcut.  A model trained on it keys on the recording config, not on
   posture, and on 6844 (posz always live) that shortcut never fires.

2. DROP insane tracker rows.  TI's FILTER gates only posz/pointy/pointz, never
   vel/acc, so three rows carrying |vel/acc| up to 2.4e35 survive into
   nn.BatchNorm1d(176).  (2.4e35)**2 = 5.8e70 overflows float32 (max 3.4e38),
   so the running variance goes inf and the input normalisation dies.

3. WINDOW WITHIN A FILE, over originally-consecutive frames only.  TI windows
   over the class-concatenated frame, so windows straddle file boundaries and
   splice non-adjacent frames across dropped rows.

Also fixes TI's leaked-loop-variable bug: cell 6's point scan iterates
`for col in df.columns` where `df` is whichever file the *previous* loop
happened to leave bound, so the point columns of every other schema are
silently never scanned.  We scan each file's own columns.

Deviations 1-3 and the class set are the reason this is a retrain and not a
reproduction; see the Phase 2 notes in firmware/people_tracking_6844/README.md.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

# ---------------------------------------------------------------- TI constants
# Kept bit-identical to the notebook (cell 1) so the features stay comparable.
WINDOW_SIZE = 8          # frames per sample; 10 fps on both 6432 and 6844 -> 0.8 s
MIN_POINTS = 5           # highest-N points used as features
MAX_HEIGHT, MIN_HEIGHT = 3.0, -4.0      # gate on pointz and posz
MAX_DISTANCE, MIN_DISTANCE = 5.0, -4.0  # gate on pointy (relative to posy)

# TI's enum, minus WALKING (dropped: after cleaning it retains ~2 sessions, and
# 51% of its rows were the posz-dead artifact).  Indices 0-3 match TI's
# class_data so the TLV contract stays forward-compatible with a 5th slot.
CLASS_NAMES = ["Stood", "Sat", "Lying", "Falling"]
CLASS_DIRS = {0: "standing", 1: "sitting", 2: "lying", 3: "falling"}

TRACKER_COLS = ["posx", "posy", "posz", "velx", "vely", "velz",
                "accx", "accy", "accz"]

# Feature order -- must stay identical to the firmware's gDataSet[] fill.
# TI uses 22: posz, velx, vely, velz, accx, accy, accz, then 5x (y-posy, z, snr).
#
# We drop velx and accx -> 20.  They are the cross-range axis, which only the
# `replay_*` schema records; the `results_*` schema omits the column entirely
# and TI's cell 6 fills it with 0.  Measured on the cleaned set, velx is
# *exactly* 0.0 in 98.1/98.0/98.5% of Stood/Sat/Lying rows but only 76.8% of
# Falling -- a real tracker never emits exact 0.0, so the feature encodes which
# file schema a row came from, and schema correlates with class (AUC 0.76).
# std(non-Falling) is 2.7e-5, so BatchNorm divides by it and amplifies that
# schema indicator ~95x into a dominant discriminator.  On 6844 the axis is
# genuinely live and O(1), so the model would read it wildly out of
# distribution.  vely/velz/accy/accz stay: their Falling separation (AUC up to
# 0.93 on velz) is real physics -- a falling body has vertical velocity.
STATIC_FEATURES = ["posz", "vely", "velz", "accy", "accz"]
FEATURE_NAMES = STATIC_FEATURES + [
    f"{v}{i}" for i in range(MIN_POINTS) for v in ("y", "z", "snr")
]
FEATURE_COUNT = len(FEATURE_NAMES)          # 20
INPUT_SIZE = FEATURE_COUNT * WINDOW_SIZE    # 160

# ------------------------------------------------------------ cleaning limits
POSZ_DEAD_ABSMAX = 0.10   # file-level: |posz| never exceeds this -> axis is dead
TRACKER_SANE_ABSMAX = 1e3  # row-level: any |vel/acc| above this is corrupt


@dataclass
class Split:
    X: np.ndarray        # (N, INPUT_SIZE) float32, feature-major window
    y: np.ndarray        # (N,) int64
    groups: np.ndarray   # (N,) int64, index into `files` -- for grouped splits
    files: list[str]


def _point_column_triples(columns) -> list[tuple[str, str, str]]:
    """(pointy, pointz, snr) column triples present in *this* file.

    classes.zip mixes two schemas: `pointy0/pointz0/snr0` (0-indexed, no
    pointx) and `pointx1/pointy1/pointz1/doppler1/snr1` (1-indexed).  TI's
    `col.replace('pointy', ...)` trick handles both; the bug was only ever
    which frame's `.columns` got scanned.
    """
    triples = []
    for col in columns:
        m = re.fullmatch(r"pointy(\d+)", str(col))
        if not m:
            continue
        z, s = f"pointz{m.group(1)}", f"snr{m.group(1)}"
        if z in columns and s in columns:
            triples.append((col, z, s))
    return triples


def _row_features(row, triples) -> np.ndarray | None:
    """TI cell-6 per-frame feature vector, or None if the row is unusable."""
    pts = []
    posy = row["posy"]
    for py, pz, ps in triples:
        y, z, s = row[py], row[pz], row[ps]
        if np.isnan(y) or np.isnan(z) or np.isnan(s):
            continue
        pts.append((y - posy, z, s))
    if not pts:
        return None

    p = np.asarray(pts, dtype=np.float64)
    keep = ((p[:, 0] <= MAX_DISTANCE) & (p[:, 0] >= MIN_DISTANCE)
            & (p[:, 1] <= MAX_HEIGHT) & (p[:, 1] >= MIN_HEIGHT))
    p = p[keep]
    if p.shape[0] < MIN_POINTS:
        return None

    # Sort by height ascending and take the MIN_POINTS highest, still ascending
    # -- matches TI's df_points.sort_values('pointz').tail(MIN_POINTS).
    p = p[np.argsort(p[:, 1], kind="stable")][-MIN_POINTS:]

    static = np.array([row[c] for c in STATIC_FEATURES], dtype=np.float64)
    return np.concatenate([static, p.reshape(-1)]).astype(np.float32)


def load_file(path: Path) -> tuple[np.ndarray, np.ndarray] | None:
    """One CSV -> (features (M, 22), original frame index (M,)).

    Returns None if the file is rejected wholesale (dead elevation axis).
    """
    df = pd.read_csv(path, encoding="utf-8", engine="python")

    # Schemas vary: some files omit posx/velx/accx entirely (a 2D tracker
    # config).  Fill so the frame is uniform -- posy/posz are the ones we use;
    # velx/accx are filled but then excluded by STATIC_FEATURES (see above).
    missing = {c: 0.0 for c in TRACKER_COLS if c not in df.columns}
    if missing:
        df = pd.concat([df, pd.DataFrame(missing, index=df.index)], axis=1)

    # Reject: elevation axis never moved -> this file has no height signal.
    pz = df["posz"].to_numpy(dtype=np.float64, na_value=np.nan)
    pz_valid = pz[np.isfinite(pz) & (pz > MIN_HEIGHT) & (pz < MAX_HEIGHT)]
    if pz_valid.size == 0 or np.abs(pz_valid).max() < POSZ_DEAD_ABSMAX:
        return None

    triples = _point_column_triples(set(df.columns))
    if not triples:
        return None

    trk = df[TRACKER_COLS].to_numpy(dtype=np.float64, na_value=np.nan)
    feats, idx = [], []
    for i, (_, row) in enumerate(df.iterrows()):
        # Row gate: TI's posz range ...
        posz = row["posz"]
        if not (np.isfinite(posz) and MIN_HEIGHT <= posz <= MAX_HEIGHT):
            continue
        # ... plus the vel/acc sanity TI never applied.
        t = trk[i]
        if not np.all(np.isfinite(t)) or np.abs(t).max() > TRACKER_SANE_ABSMAX:
            continue
        f = _row_features(row, triples)
        if f is None or not np.all(np.isfinite(f)):
            continue
        feats.append(f)
        idx.append(i)

    if not feats:
        return None
    return np.stack(feats), np.asarray(idx, dtype=np.int64)


def _window(feats: np.ndarray, idx: np.ndarray) -> np.ndarray:
    """Stack WINDOW_SIZE consecutive frames, feature-major.

    Layout must match the firmware's CreateFeatureVector() interleave:
    x[j * WINDOW_SIZE + k] = frame k, feature j, with k=0 oldest.  That is
    exactly pandas' .unstack() column-major order, which is what TI trained on.
    Only windows whose frames were consecutive in the original file are kept.
    """
    out = []
    n, w = feats.shape[0], WINDOW_SIZE
    for s in range(n - w + 1):
        if idx[s + w - 1] - idx[s] != w - 1:   # a dropped frame -> not contiguous
            continue
        out.append(feats[s:s + w].T.reshape(-1))   # (22, 8) -> feature-major 176
    return np.stack(out) if out else np.empty((0, INPUT_SIZE), dtype=np.float32)


def build(root: Path, verbose: bool = True) -> Split:
    """Load every class dir under `root` into a windowed, cleaned Split."""
    X, y, g, files = [], [], [], []
    for label, dirname in CLASS_DIRS.items():
        d = root / dirname
        kept = dropped = rows = wins = 0
        for path in sorted(d.glob("*.csv")):
            r = load_file(path)
            if r is None:
                dropped += 1
                if verbose:
                    print(f"    reject (posz axis dead): {path.name}")
                continue
            feats, idx = r
            xs = _window(feats, idx)
            if xs.shape[0] == 0:
                dropped += 1
                continue
            kept += 1
            rows += feats.shape[0]
            wins += xs.shape[0]
            X.append(xs)
            y.append(np.full(xs.shape[0], label, dtype=np.int64))
            g.append(np.full(xs.shape[0], len(files), dtype=np.int64))
            files.append(f"{dirname}/{path.name}")
        if verbose:
            print(f"  {CLASS_NAMES[label]:8s} files kept={kept:2d} dropped={dropped:2d}"
                  f"  rows={rows:5d} windows={wins:5d}")
    return Split(np.concatenate(X), np.concatenate(y), np.concatenate(g), files)
