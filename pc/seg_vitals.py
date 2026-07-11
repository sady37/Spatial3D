"""Non-stationary vital/tremor analysis: motion envelope + spectrogram + per-segment.

For aperiodic motion (a pull/jerk, not a sustained tremor) a single 120s FFT
smears the short burst into the noise. Instead:
  1. motion envelope = per-frame fluctuation energy over the target bins -> when
     is there motion (active) vs still.
  2. spectrogram (STFT) of the peak bin's coherent projection -> time-frequency
     of the pull (broadband/transient) vs the still band (RR/HR lines).
  3. per-segment: still -> RR(0.15-0.5)/HR(0.8-2.5) peaks; active -> span + band.

    python seg_vitals.py fall20_cube.npz --fps 18.78 --png seg_fall20.png
"""
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DR = 0.0234375


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path")
    ap.add_argument("--fps", type=float, default=None)
    ap.add_argument("--png", default="seg_vitals.png")
    ap.add_argument("--win", type=float, default=3.0, help="STFT window (s)")
    a = ap.parse_args()

    d = np.load(a.path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], dtype=np.complex64)   # (M, Kmax, 16)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    fps = a.fps or (float(d["fps"]) if "fps" in d else 5.0)
    kmin = int(counts.min())
    C = cube[:, :kmin, :]                                   # aligned
    t = np.arange(kmin) / fps

    Cdc = C - C.mean(axis=1, keepdims=True)                 # remove static
    # motion envelope: total fluctuation energy per frame (all bins, all ant)
    motion = (np.abs(Cdc) ** 2).sum(axis=(0, 2))
    # peak bin = most micro-motion overall
    pk = int(np.argmax((np.abs(Cdc) ** 2).sum(axis=(1, 2))))
    m = C[pk].mean(0); m = m / (np.linalg.norm(m) + 1e-9)
    sig = (Cdc[pk] @ m.conj()).real                        # slow-time projection

    # smooth envelope (~1s) and segment: active = above median + 4*MAD
    w = max(1, int(fps))
    env = np.convolve(motion, np.ones(w) / w, mode="same")
    med = np.median(env); mad = np.median(np.abs(env - med)) + 1e-9
    active = env > med + 4 * mad
    # contiguous segments
    segs = []
    i = 0
    while i < len(active):
        j = i
        while j < len(active) and active[j] == active[i]:
            j += 1
        if (j - i) / fps >= 1.0:                            # >=1s runs only
            segs.append((i, j, bool(active[i])))
        i = j

    def band_pk(x, lo, hi):
        x = x - x.mean()
        f = np.fft.rfftfreq(len(x), 1 / fps); S = np.abs(np.fft.rfft(x)) ** 2
        mk = (f >= lo) & (f <= hi)
        if not mk.any() or len(x) < fps * 2:
            return None
        j = np.argmax(S[mk]); return f[mk][j], S[mk][j]

    print(f"cube {a.path}: {len(bins)} bins {bins[0]}-{bins[-1]} "
          f"({bins[0]*DR:.2f}-{bins[-1]*DR:.2f}m), {kmin} frames @{fps:.1f}fps, "
          f"peak bin{bins[pk]}={bins[pk]*DR:.2f}m")
    print(f"{'segment':>16} {'kind':>6} | RR / HR / tremor peak")
    for i, j, act in segs:
        x = sig[i:j]; dur = (j - i) / fps
        lbl = f"{i/fps:5.1f}-{j/fps:5.1f}s"
        if act:
            tr = band_pk(x, 3.0, hi := min(9.0, fps / 2 - 0.5))
            rms = np.sqrt(np.mean((x - x.mean()) ** 2))
            trs = f"tremor/pull peak {tr[0]:.2f}Hz" if tr else "n/a"
            print(f"{lbl:>16} {'ACTIVE':>6} | {trs}  (rms={rms:.0f}, {dur:.0f}s aperiodic)")
        else:
            rr = band_pk(x, 0.15, 0.5); hr = band_pk(x, 0.8, 2.5)
            rrs = f"RR {rr[0]*60:.0f}/min" if rr else "RR n/a"
            hrs = f"HR {hr[0]*60:.0f}/min" if hr else "HR n/a"
            print(f"{lbl:>16} {'still':>6} | {rrs} | {hrs}  ({dur:.0f}s)")

    # --- figure: envelope (segments shaded) + spectrogram ---
    win = max(8, int(a.win * fps)); hop = max(1, win // 4)
    cols, tc = [], []
    hann = np.hanning(win)
    for s0 in range(0, len(sig) - win, hop):
        S = np.abs(np.fft.rfft((sig[s0:s0 + win] - sig[s0:s0 + win].mean()) * hann)) ** 2
        cols.append(S); tc.append((s0 + win / 2) / fps)
    Sxx = np.array(cols).T
    fspec = np.fft.rfftfreq(win, 1 / fps)

    fig, ax = plt.subplots(2, 1, figsize=(13, 7), sharex=True,
                           gridspec_kw={"height_ratios": [1, 2]})
    ax[0].plot(t, env / env.max(), color="k", lw=0.8)
    for i, j, act in segs:
        ax[0].axvspan(i / fps, j / fps, color="red" if act else "green", alpha=0.15)
    ax[0].set_ylabel("motion envelope"); ax[0].set_title(
        f"{a.path} @{fps:.1f}fps — motion (green=still, red=active/pull)")
    im = ax[1].pcolormesh(tc, fspec, 10 * np.log10(Sxx + 1e-6), shading="auto",
                          cmap="magma")
    for lo, hi, c in [(0.15, 0.5, "lime"), (0.8, 2.5, "cyan"), (3, 9, "white")]:
        ax[1].axhline(lo, color=c, lw=0.6, ls=":"); ax[1].axhline(hi, color=c, lw=0.6, ls=":")
    ax[1].set_ylim(0, fps / 2); ax[1].set_ylabel("Hz"); ax[1].set_xlabel("time (s)")
    ax[1].set_title("spectrogram of peak-bin projection  "
                    "(green=RR band, cyan=HR, white=tremor 3-9Hz)")
    fig.colorbar(im, ax=ax[1], shrink=0.7, label="dB")
    plt.tight_layout(); plt.savefig(a.png, dpi=120); plt.close()
    print(f"saved {a.png}")


if __name__ == "__main__":
    main()
