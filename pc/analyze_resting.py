"""Resting deep-dive: sliding-window HR/RR over the chairL/chairR blocks vs the
fingertip pulse-oximeter reference (74-81 bpm, avg 77). Establishes the resting
per-window noise floor + how close HR sits to a breathing harmonic (STEP-2 preview).

    .venv/bin/python3 analyze_resting.py
"""
import glob, datetime as dt
import numpy as np
from bcg_vitals import demod_channels, estimate_rr, estimate_hr

OXI_LO, OXI_HI, OXI_AVG = 74, 81, 77
WIN_S, STEP_S = 30.0, 10.0

blocks = []
for f in sorted(glob.glob("chairL_*.npz")):
    d = np.load(f, allow_pickle=True)
    cube = np.asarray(d["snapshots"], np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    ts = d["frame_ts"]; fps = len(ts) / (ts[-1] - ts[0])
    C = cube[:, :int(counts.min()), :]
    chans = demod_channels(C, bins)
    pos = "chairR" if ts[0] >= 1783989615 - 1 else "chairL"
    blocks.append((f, pos, ts, fps, chans))

allt, allhr, allrr, allstr, allcoll = [], [], [], [], []
for f, pos, ts, fps, chans in blocks:
    win, step = int(WIN_S * fps), int(STEP_S * fps)
    n = chans[0].shape[0]
    for s in range(0, n - win + 1, step):
        seg = [c[s:s + win] for c in chans]
        rr, f0, _, _ = estimate_rr(seg, fps)
        hr = estimate_hr(seg, fps, f0)
        if hr["hr"] is None:
            continue
        tc = ts[0] + (s + win / 2) / fps
        fH = hr["hr"] / 60.0
        dmin = min(abs(fH - k * f0) for k in range(1, 9)) if f0 else 9
        allt.append(tc); allhr.append(hr["hr"]); allrr.append(rr)
        allstr.append(hr["strength"]); allcoll.append(dmin)

allt = np.array(allt); allhr = np.array(allhr); allrr = np.array(allrr)
allstr = np.array(allstr); allcoll = np.array(allcoll)

inrange = np.mean((allhr >= OXI_LO) & (allhr <= OXI_HI)) * 100
print("=== 静息滑窗 HR (30s/10s) vs 血氧 74-81 (avg77) ===")
print(f"windows={len(allhr)}  HR: med={np.median(allhr):.1f} mean={allhr.mean():.1f} "
      f"std={allhr.std():.1f}  min={allhr.min():.0f} max={allhr.max():.0f}")
print(f"MAE vs oxi-avg 77 = {np.mean(np.abs(allhr - OXI_AVG)):.1f} bpm")
print(f"% windows inside 74-81 = {inrange:.0f}%")
print(f"RR: med={np.median(allrr):.0f} rpm   strength med={np.median(allstr):.2f}")
print(f"HR-to-nearest-harmonic |fH - k*f0|: med={np.median(allcoll):.3f}Hz "
      f"min={allcoll.min():.3f}  (<0.10Hz = resonant/entangled)")
res = np.mean(allcoll < 0.10) * 100
print(f"% windows resonant (HR within 0.10Hz of k*RR) = {res:.0f}%")

try:
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    t0 = allt[0]; tt = (allt - t0) / 60.0
    fig, ax = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    ax[0].axhspan(OXI_LO, OXI_HI, color="green", alpha=0.15, label="oximeter 74-81")
    ax[0].axhline(OXI_AVG, color="green", ls="--", lw=1, label="oxi avg 77")
    sc = ax[0].scatter(tt, allhr, c=allstr, cmap="viridis", s=18, label="radar HR (color=strength)")
    ax[0].plot(tt, allhr, color="0.6", lw=0.5)
    for f, pos, ts, fps, chans in blocks:
        ax[0].axvline((ts[0] - t0) / 60, color="k", ls=":", lw=0.6)
    ax[0].set_ylabel("HR bpm"); ax[0].set_ylim(60, 95); ax[0].legend(loc="upper right", fontsize=8)
    ax[0].set_title("Resting HR (chairL 0-10min, chairR 10-15min) vs oximeter — sliding 30s")
    plt.colorbar(sc, ax=ax[0], label="strength")
    ax[1].plot(tt, allcoll, "C3.-", lw=0.6, ms=3)
    ax[1].axhline(0.10, color="r", ls="--", lw=0.8, label="0.10Hz resonance threshold")
    ax[1].set_ylabel("|fH - k·f0| Hz"); ax[1].set_xlabel("t (min from %s)"
                     % dt.datetime.fromtimestamp(t0).strftime("%H:%M:%S"))
    ax[1].legend(fontsize=8); ax[1].set_title("HR distance to nearest breathing harmonic (resonance/entanglement)")
    fig.tight_layout(); fig.savefig("resting_hr_track.png", dpi=110)
    print("\nsaved resting_hr_track.png")
except Exception as e:
    print("plot skipped:", e)
