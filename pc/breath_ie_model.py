"""Test the PREMISE of predict-and-subtract HR: is expiration (e) a stable, predictable
shape (passive exponential recoil) that can be predicted from the inspiration peak?

Measures, on a regular breathing segment:
  - I:E ratio stability (Ti vs Te per breath, CV)
  - shape reproducibility: normalized inspiration & expiration overlaid -> fixed shape?
  - expiration exponential fit V=A*exp(-(t-tp)/tau): tau stability, fit R^2
Verdict: if e collapses onto one curve with stable tau, predict-subtract is viable.

    python3 breath_ie_model.py [block.npz]
"""
import sys
import numpy as np
from scipy.signal import find_peaks
from bcg_vitals import demod_channels, bandpass, sqi, RR_LO, RR_HI


def load(path):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    C = cube[:, :int(counts.min()), :]
    try:
        ts = np.asarray(d["frame_ts"], float)[:int(counts.min())]
        span = ts[-1] - ts[0]
        fps = (len(ts) - 1) / (span / 1000 if span > 1e4 else span)
    except KeyError:
        fps = 18.78
    return C, bins, fps


def main():
    block = sys.argv[1] if len(sys.argv) > 1 else "chairL_20260713_183514.npz"
    C, bins, fps = load(block)
    chans = demod_channels(C, bins)
    resp_sqi = np.array([sqi(bandpass(c, fps, RR_LO, RR_HI), fps, RR_LO, RR_HI) for c in chans])
    rb = int(np.argmax(resp_sqi))
    b = bandpass(chans[rb], fps, 0.08, 0.8)
    T = len(b)

    tr, _ = find_peaks(-b, distance=int(fps / RR_HI), prominence=b.std() * 0.3)
    pk, _ = find_peaks(b,  distance=int(fps / RR_HI), prominence=b.std() * 0.3)
    print(f"{block}: bin {bins[rb]} @ {fps:.2f}fps, {len(tr)} troughs {len(pk)} peaks")

    # build clean breaths: trough -> peak -> trough (need one peak strictly between)
    breaths = []
    for a, c in zip(tr[:-1], tr[1:]):
        mids = pk[(pk > a) & (pk < c)]
        if len(mids) == 1:
            breaths.append((a, int(mids[0]), c))
    print(f"clean single-peak breaths: {len(breaths)}")
    if len(breaths) < 5:
        print("too few clean breaths (irregular breathing) — predict-subtract premise weak here")

    Ti = np.array([(p - a) / fps for a, p, c in breaths])   # rise (side L)
    Te = np.array([(c - p) / fps for a, p, c in breaths])    # fall (side R)
    # orient: expiration = the LONGER passive side
    rise_is_insp = np.median(Ti) < np.median(Te)
    insp = Ti if rise_is_insp else Te
    expi = Te if rise_is_insp else Ti
    print(f"\n durations: rise {np.median(Ti):.2f}s (CV {np.std(Ti)/np.mean(Ti):.2f}), "
          f"fall {np.median(Te):.2f}s (CV {np.std(Te)/np.mean(Te):.2f})")
    print(f" -> inspiration ~{np.median(insp):.2f}s, expiration ~{np.median(expi):.2f}s, "
          f"I:E = 1:{np.median(expi)/np.median(insp):.2f}  (CV of I:E {np.std(insp/expi)/np.mean(insp/expi):.2f})")

    # exponential fit of each expiration (peak -> next trough), passive-recoil test
    taus, r2s, exps_norm = [], [], []
    for a, p, c in breaths:
        seg = b[p:c].copy()
        if len(seg) < 4:
            continue
        seg = seg - b[c]                       # baseline at end-expiration
        A = seg[0]
        if A <= 1e-9:
            continue
        tt = np.arange(len(seg)) / fps
        y = seg / A
        yy = np.clip(y, 1e-3, None)
        # log-linear fit -> tau
        good = y > 0.05
        if good.sum() < 3:
            continue
        k = np.polyfit(tt[good], np.log(yy[good]), 1)[0]
        tau = -1 / k if k < 0 else np.nan
        pred = np.exp(k * tt)
        ss = 1 - np.sum((y - pred) ** 2) / (np.sum((y - y.mean()) ** 2) + 1e-9)
        if np.isfinite(tau) and 0.05 < tau < 20:
            taus.append(tau); r2s.append(ss)
            exps_norm.append(np.interp(np.linspace(0, 1, 40), tt / tt[-1], y))
    taus = np.array(taus); r2s = np.array(r2s)
    print(f"\n expiration exponential fit ({len(taus)} breaths):")
    print(f"   tau = {np.median(taus):.2f}s  (CV {np.std(taus)/np.mean(taus):.2f})   "
          f"exp-fit R^2 median = {np.median(r2s):.2f}")

    # shape reproducibility: variance of normalized expiration across breaths
    if exps_norm:
        E = np.array(exps_norm)
        spread = np.mean(np.std(E, 0))         # avg pointwise std of normalized shape
        print(f"   normalized-expiration shape spread (0=identical) = {spread:.3f}")
        verdict = ("STRONG: e is a stable predictable shape -> predict-subtract viable"
                   if np.std(taus)/np.mean(taus) < 0.35 and np.median(r2s) > 0.8 and spread < 0.12
                   else "WEAK: e shape/tau too variable -> per-breath prediction unreliable here")
        print(f"\n VERDICT: {verdict}")

    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 3, figsize=(15, 4.3))
        # overlay all breaths aligned at peak
        for a, p, c in breaths:
            tt = (np.arange(a, c) - p) / fps
            ax[0].plot(tt, b[a:c] - b[c], lw=.6, alpha=.5)
        ax[0].axvline(0, color="k", ls=":", label="peak")
        ax[0].set_title(f"{len(breaths)} breaths aligned at peak (t=0)"); ax[0].set_xlabel("s from peak")
        ax[0].legend(fontsize=8)
        # normalized expirations overlaid
        if exps_norm:
            xn = np.linspace(0, 1, 40)
            for e in exps_norm:
                ax[1].plot(xn, e, "C1", lw=.5, alpha=.4)
            ax[1].plot(xn, E.mean(0), "k", lw=2.5, label="mean")
            ax[1].plot(xn, np.exp(-xn * (1/np.median(taus)) * (np.median(expi))), "C0--",
                       lw=2, label=f"exp(tau={np.median(taus):.2f}s)")
            ax[1].set_title(f"normalized expirations (spread={spread:.3f})")
            ax[1].set_xlabel("normalized expiration time"); ax[1].legend(fontsize=8)
        ax[2].hist(taus, bins=12, color="C1", alpha=.8)
        ax[2].axvline(np.median(taus), color="k", ls="--")
        ax[2].set_title(f"expiration tau: med {np.median(taus):.2f}s CV {np.std(taus)/np.mean(taus):.2f}")
        ax[2].set_xlabel("tau (s)")
        fig.tight_layout(); fig.savefig("breath_ie_model.png", dpi=115)
        print("saved breath_ie_model.png")
    except Exception as e:
        print("plot skipped:", e)


if __name__ == "__main__":
    main()
