"""Train the 4-class pose MLP and export folded weights as C arrays.

    python -m pose.train --data <dir with standing/ sitting/ lying/ falling/> \
                         --out ../firmware/people_tracking_6844/src/6844/mss/source/pose/pose_model.c

Reports two accuracies on purpose:

* "random-row split" -- what TI's notebook does (train_test_split on X).  The
  windows overlap by 7 of 8 frames, so a random split puts near-duplicates of
  every test window into train.  This number is inflated and is printed only to
  show the size of the illusion.
* "grouped by file" -- whole recordings held out.  This is the number that
  predicts field behaviour, and the one to quote in any caveat.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
from sklearn.model_selection import GroupKFold, train_test_split

from .dataset import (CLASS_NAMES, FEATURE_COUNT, FEATURE_NAMES, INPUT_SIZE,
                      WINDOW_SIZE, build)
from .model import PoseMLP, fold_model, folded_forward

SEED = 42


def _fit(Xtr, ytr, epochs=400, bs=128, lr=1e-3, verbose=False):
    torch.manual_seed(SEED)
    m = PoseMLP(INPUT_SIZE, len(CLASS_NAMES))
    opt = torch.optim.Adam(m.parameters(), lr=lr)
    # Mild class imbalance (Stood ~2x Sat); weight so recall isn't traded away.
    cnt = np.bincount(ytr, minlength=len(CLASS_NAMES)).astype(np.float64)
    w = torch.tensor((cnt.sum() / (len(cnt) * np.maximum(cnt, 1))), dtype=torch.float32)
    loss_fn = nn.CrossEntropyLoss(weight=w)
    Xt = torch.from_numpy(Xtr).float()
    yt = torch.from_numpy(ytr).long()
    n = len(Xt)
    m.train()
    for ep in range(epochs):
        perm = torch.randperm(n)
        tot = 0.0
        for i in range(0, n, bs):
            idx = perm[i:i + bs]
            if len(idx) < 2:      # BatchNorm needs >1 sample
                continue
            opt.zero_grad()
            loss = loss_fn(m(Xt[idx]), yt[idx])
            loss.backward()
            opt.step()
            tot += float(loss) * len(idx)
        if verbose and (ep + 1) % 100 == 0:
            print(f"      epoch {ep+1:4d}  loss {tot/n:.4f}")
    m.eval()
    return m


def _report(m, X, y, title):
    with torch.no_grad():
        pred = m(torch.from_numpy(X).float()).argmax(1).numpy()
    acc = float((pred == y).mean())
    print(f"\n  {title}: overall accuracy {acc*100:.1f}%")
    print(f"    {'true\\pred':>10s} " + " ".join(f"{c:>8s}" for c in CLASS_NAMES) + "   recall")
    for i, c in enumerate(CLASS_NAMES):
        row = [(int(((y == i) & (pred == j)).sum())) for j in range(len(CLASS_NAMES))]
        n = max(int((y == i).sum()), 1)
        print(f"    {c:>10s} " + " ".join(f"{v:8d}" for v in row) + f"   {row[i]/n*100:5.1f}%")
    return acc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, type=Path)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--epochs", type=int, default=400)
    args = ap.parse_args()

    print(f"features: {FEATURE_COUNT} x {WINDOW_SIZE} frames = {INPUT_SIZE} inputs")
    print(f"classes : {CLASS_NAMES}\n")
    s = build(args.data)
    print(f"\ndataset: X={s.X.shape}  files={len(s.files)}")
    print("balance:", {CLASS_NAMES[i]: int((s.y == i).sum()) for i in range(len(CLASS_NAMES))})

    # ---- 1. TI's split, reproduced only to show how much it flatters ----
    Xtr, Xte, ytr, yte = train_test_split(s.X, s.y, test_size=0.2, random_state=SEED)
    m = _fit(Xtr, ytr, epochs=args.epochs)
    acc_random = _report(m, Xte, yte, "RANDOM-ROW split (TI's method -- INFLATED, overlapping windows)")

    # ---- 2. Honest: hold out whole recordings ----
    print("\n" + "=" * 74)
    gkf = GroupKFold(n_splits=4)
    accs, cms = [], np.zeros((len(CLASS_NAMES), len(CLASS_NAMES)), dtype=int)
    for k, (tr, te) in enumerate(gkf.split(s.X, s.y, groups=s.groups)):
        mk = _fit(s.X[tr], s.y[tr], epochs=args.epochs)
        with torch.no_grad():
            pred = mk(torch.from_numpy(s.X[te]).float()).argmax(1).numpy()
        accs.append(float((pred == s.y[te]).mean()))
        for a, b in zip(s.y[te], pred):
            cms[a, b] += 1
        print(f"  grouped fold {k+1}/4: held-out files={len(set(s.groups[te]))}  acc {accs[-1]*100:.1f}%")
    print(f"\n  GROUPED-BY-FILE 4-fold: accuracy {np.mean(accs)*100:.1f}% +/- {np.std(accs)*100:.1f}%"
          f"   <-- the honest number")
    print(f"    {'true\\pred':>10s} " + " ".join(f"{c:>8s}" for c in CLASS_NAMES) + "   recall")
    for i, c in enumerate(CLASS_NAMES):
        n = max(cms[i].sum(), 1)
        print(f"    {c:>10s} " + " ".join(f"{v:8d}" for v in cms[i]) + f"   {cms[i, i]/n*100:5.1f}%")
    print(f"\n  inflation from TI's random-row split: "
          f"{acc_random*100:.1f}% -> {np.mean(accs)*100:.1f}%  ({(acc_random-np.mean(accs))*100:+.1f} pts)")

    # ---- 3. Final model on all data, folded + exported ----
    print("\n" + "=" * 74)
    print("\ntraining final model on all data...")
    final = _fit(s.X, s.y, epochs=args.epochs)
    layers = fold_model(final)

    # The folded path is what the firmware runs -- prove it matches torch.
    with torch.no_grad():
        ref = final.predict_proba(torch.from_numpy(s.X).float()).numpy()
    got = folded_forward(layers, s.X.astype(np.float64))
    err = float(np.abs(ref - got).max())
    print(f"  BN-fold check: max |torch - folded| = {err:.3e}  "
          f"({'OK' if err < 1e-4 else 'MISMATCH'})")
    if err >= 1e-4:
        raise SystemExit("BN folding does not reproduce the torch model -- aborting export")

    nparam = sum(W.size + b.size for W, b in layers)
    print(f"  folded params: {nparam} floats = {nparam*4/1024:.1f} KB rodata")
    for i, (W, b) in enumerate(layers):
        print(f"    layer{i}: W{W.shape} b{b.shape}")

    if args.out:
        from .export_c import write_c
        write_c(args.out, layers, acc=float(np.mean(accs)))
        print(f"\n  wrote {args.out}")


if __name__ == "__main__":
    main()
