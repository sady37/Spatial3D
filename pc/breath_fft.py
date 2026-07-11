"""Slow-time breathing FFT = base-free liveness + localization (§6.4, §7).

Per range bin, the slow-time complex series (K frames x 16 antennas) -> remove
DC (static) -> FFT -> breathing-band (0.15-0.5Hz) power = liveness; peak = rate.
Distinguishes person/empty, localizes range, outputs breathing rate. Immune to
the elevation artifact (reads frequency, not angle). Localization (fall = floor
prior): Y=sqrt(R^2-H^2) from the range bin; X=Y*tan(az) from a beamformer on the
breathing-band-filtered signal (accuracy from the narrowband SNR gain, NOT MUSIC
-- MUSIC == Bartlett here for a single target, §7.4).

    python breath_fft.py cal_I.npz                 # micro-motion vs range + rate
    python breath_fft.py cal_I.npz --localize      # + X/Y of the breathing person
    python breath_fft.py sit_detect.npz --cube cube_sit   # a named cube inside npz
"""
import argparse

import numpy as np

H_MOUNT = 2.0
FPS = 5.0
DR = 0.0234375


def load_cube(path, key):
    """Return (cube[M,K,16] complex, counts[M], bins[M], dr)."""
    d = np.load(path, allow_pickle=True)
    bins = d["bins"].astype(int)
    dr = float(d["dr_m"]) if "dr_m" in d else DR
    if key:                                    # e.g. cube_sit / cube_base
        cube = np.asarray(d[key], dtype=np.complex64)
        cnt = d[key.replace("cube", "counts")] if key.replace("cube", "counts") in d \
            else np.full(len(bins), cube.shape[1], np.int32)
    elif "snapshots" in d:
        cube = np.asarray(d["snapshots"], dtype=np.complex64)
        cnt = d["counts"] if "counts" in d else np.full(len(bins), cube.shape[1], np.int32)
    else:
        raise ValueError(f"{path}: no cube; pass --cube <key>")
    return cube, cnt.astype(int), bins, dr


def bin_spectrum(x):
    """Non-DC power spectrum summed over antennas for one bin's (K,16) series."""
    x = x - x.mean(0, keepdims=True)
    return (np.abs(np.fft.fft(x, axis=0)) ** 2).sum(1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path")
    ap.add_argument("--cube", default=None, help="cube key inside npz (e.g. cube_sit)")
    ap.add_argument("--fps", type=float, default=FPS)
    ap.add_argument("--localize", action="store_true")
    ap.add_argument("--png", default=None)
    a = ap.parse_args()

    cube, counts, bins, dr = load_cube(a.path, a.cube)
    K = cube.shape[1]
    freqs = np.fft.fftfreq(K, 1 / a.fps)
    fb = (np.abs(freqs) >= 0.1) & (np.abs(freqs) <= 2.5)      # micro-motion
    breath = (np.abs(freqs) >= 0.15) & (np.abs(freqs) <= 0.5)
    micro = np.zeros(len(bins)); bre = np.zeros(len(bins))
    for i in range(len(bins)):
        k = int(counts[i])
        if k < 20:
            continue
        sp = bin_spectrum(cube[i, :k])
        micro[i] = sp[fb[:k]].sum(); bre[i] = sp[breath[:k]].sum()
    rng = bins * dr
    pk = int(np.argmax(micro)); k = int(counts[pk])
    sp = bin_spectrum(cube[pk, :k])
    # Rate = peak WITHIN the breathing band (0.15-0.5Hz). Taking the global argmax
    # lets DC-leakage at ~0.02Hz win and reports a bogus ~1/min; restrict the search.
    bb = (freqs[:k] >= 0.15) & (freqs[:k] <= 0.5)
    fpk = freqs[:k][bb][np.argmax(sp[bb])]
    print(f"micro peak: bin{bins[pk]} R={rng[pk]:.2f}m  breath-rate={fpk:.3f}Hz={fpk*60:.0f}/min  "
          f"peak/median={micro.max()/np.median(micro[micro>0]):.1f}x")

    if a.localize:
        from spatial3d.music import awrl6844_array, bartlett_doa
        array = awrl6844_array()
        band = (np.abs(freqs) >= 0.15) & (np.abs(freqs) <= 0.5)
        top = np.argsort(bre)[::-1][:6]
        xs, ys = [], []
        for i in top:
            k = int(counts[i]); x = cube[i, :k] - cube[i, :k].mean(0)
            X = np.fft.fft(x, axis=0); Xb = np.zeros_like(X); Xb[band[:k]] = X[band[:k]]
            xb = np.fft.ifft(Xb, axis=0); R = (xb.conj().T @ xb) / k
            dets = bartlett_doa(R, array, az_range=(-45, 45), el_range=(-20, 20),
                                resolution_deg=1.5)
            if not dets:
                continue
            Y = np.sqrt(max((bins[i] * dr) ** 2 - H_MOUNT ** 2, 0))
            xs.append(Y * np.tan(np.deg2rad(dets[0][0]))); ys.append(Y)
        print(f"breathing person: X={np.mean(xs):+.2f}±{np.std(xs):.2f}m  "
              f"Y={np.mean(ys):.2f}±{np.std(ys):.2f}m  "
              f"(Y=sqrt(R^2-H^2) floor prior; X=Y*tan(az) breath-band beamform)")

    if a.png:
        import matplotlib
        matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(13, 4))
        ax[0].bar(rng, micro, width=dr * 2, color="teal")
        ax[0].set_xlabel("range (m)"); ax[0].set_ylabel("micro-motion (0.1-2.5Hz)")
        ax[0].set_title("base-free micro-motion vs range")
        pos = freqs[:k] > 0
        ax[1].plot(freqs[:k][pos], sp[pos]); ax[1].axvspan(0.15, 0.5, color="orange", alpha=0.2)
        ax[1].set_xlim(0, 2.5); ax[1].set_xlabel("Hz"); ax[1].set_ylabel("|FFT|^2")
        ax[1].set_title(f"spectrum @ bin{bins[pk]} peak {fpk*60:.0f}/min")
        plt.tight_layout(); plt.savefig(a.png, dpi=120); plt.close()
        print(f"saved {a.png}")


if __name__ == "__main__":
    main()
