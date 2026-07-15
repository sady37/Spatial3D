"""I/E respiratory-phase-FOLDED spectrogram — the time-frequency form of the phase axis.

User lead (2026-07-14): I:E breathing asymmetry makes each breathing HARMONIC's
instantaneous frequency SWEEP within a breath (k*f_RR(t) -> a wavy/diagonal line in the
time-freq plane), while the HEARTBEAT is a CONSTANT-frequency HORIZONTAL line. So fold
the CWT energy over the respiratory phase: harmonics draw DIAGONAL curves (move with
breath phase), the heartbeat draws a FLAT HORIZONTAL band you can read HR off directly.

WHAT IT IS GOOD FOR (validated): a clean, interpretable HR readout + heart/harmonic
separation on RADIAL / range-separated geometry. lie_long (radial, truth 71): chest
bin171 shows a bright flat horizontal band at ~70bpm across all resp phases. chairL
(range-separated) likewise. See pc/case/_folded_figs/folded_lie_long.png.

WHAT IT CANNOT DO (negative, do not re-chase): break the COLOCATED / clipped-chest cell.
sitR (colocated + elevated 96-110): no heartbeat line even to the eye, only the 4th
harmonic ~80 (folded_sitR.png). chairR (body compact at the near window edge bins149-151,
chest clipped below the window): same (folded_chairR.png). No SCALAR metric (flatness x
persistence x ridge) beats an empty-room null — narrowband autocorr q~0.95 and whitened
flatness are both high for pure noise. Confirms the whole-session boundary: colocated+4m
is SNR-dead for the heartbeat on every axis (freq/space/phase/time-freq); only radial
geometry lets HR through. See memory next-crack-rr-harmonic (0714 I/E block).

    python ie_folded_spec.py pc/case/lie_long_20260714.npz --truth 69-73
"""
import argparse, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bcg_vitals as bv
from scipy.signal import hilbert

FPS = 18.78


def cwt_morlet(x, fps, freqs, w0=8):
    """Complex Morlet CWT via a freq-domain Gaussian. w0 = #cycles (time-freq tradeoff);
    w0~8 gives ~0.3Hz / ~0.6s resolution at 1.6Hz, enough to resolve I/E within a breath."""
    x = x - x.mean(); N = len(x)
    Xf = np.fft.fft(x); f = np.fft.fftfreq(N, 1 / fps)
    W = np.empty((len(freqs), N), complex)
    for i, fc in enumerate(freqs):
        sig_t = w0 / (2 * np.pi * fc); sig_f = 1 / (2 * np.pi * sig_t)
        W[i] = np.fft.ifft(Xf * np.exp(-0.5 * ((f - fc) / sig_f) ** 2))
    return W


def resp_phase(chans, fps):
    """Respiratory phase clock theta(t) = Hilbert phase of the strongest breathing bin."""
    Pb = np.array([(np.abs(np.fft.rfft(bv.bandpass(c, fps, bv.RR_LO, bv.RR_HI))) ** 2).sum()
                   for c in chans])
    ab = int(np.argmax(Pb))
    theta = np.unwrap(np.angle(hilbert(bv.bandpass(chans[ab], fps, bv.RR_LO, bv.RR_HI))))
    return theta, ab, Pb


def folded_map(x, fps, phi, freqs, nbin=24, w0=8):
    """Energy folded over respiratory phase -> (nfreq, nbin). Per-time whitened so ridges
    pop regardless of absolute level."""
    ph = np.mod(phi, 2 * np.pi)
    S = np.abs(cwt_morlet(x, fps, freqs, w0)) ** 2
    S = S / (S.mean(0, keepdims=True) + 1e-12)
    edges = np.linspace(0, 2 * np.pi, nbin + 1)
    M = np.zeros((len(freqs), nbin))
    for b in range(nbin):
        m = (ph >= edges[b]) & (ph < edges[b + 1])
        if m.any():
            M[:, b] = S[:, m].mean(1)
    return M


def plot(path, truth="", out_dir=None, nbin=24):
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d['snapshots'], np.complex64); bins = d['bins'].astype(int)
    K = int(d['counts'].astype(int).min())
    chans = bv.demod_channels(cube[:, :K, :], bins)
    phi, ab, Pb = resp_phase(chans, FPS)
    frr = np.median(np.gradient(phi)) * FPS / (2 * np.pi)
    cand = [i for i in range(len(chans))
            if 0.02 * Pb[ab] < Pb[i] < 0.6 * Pb[ab] and abs(bins[i] - bins[ab]) >= 3]
    sel = sorted(cand, key=lambda i: -Pb[i])[:4] + [ab]
    freqs = np.arange(0.7, 2.4, 0.01)
    fig, axes = plt.subplots(1, len(sel), figsize=(4 * len(sel), 5), squeeze=False)
    for c, i in enumerate(sel):
        M = folded_map(bv.bandpass(chans[i], FPS, 0.5, 2.6), FPS, phi, freqs, nbin)
        ax = axes[0][c]
        ax.pcolormesh(np.arange(nbin), freqs * 60, M, shading='auto', cmap='magma')
        for h in range(2, 11):
            ax.axhline(h * frr * 60, color='c', ls=':', lw=0.4, alpha=0.5)
        ax.set_title(f'bin{int(bins[i])}{" (abd)" if i == ab else ""}')
        ax.set_ylim(55, 150); ax.set_xlabel('resp phase bin')
    axes[0][0].set_ylabel('bpm')
    fig.suptitle(f'{os.path.basename(path)} folded  TRUTH {truth}  f_rr={frr*60:.0f}rpm '
                 f'(dotted=k*RR; flat horizontal band = HR)')
    out_dir = out_dir or os.path.join(os.path.dirname(path), '_folded_figs')
    os.makedirs(out_dir, exist_ok=True)
    fn = os.path.join(out_dir, 'folded_' + os.path.splitext(os.path.basename(path))[0] + '.png')
    fig.tight_layout(); fig.savefig(fn, dpi=90); plt.close()
    print(f"saved {fn}  (f_rr={frr*60:.0f}rpm, abdomen bin{int(bins[ab])})")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("path"); ap.add_argument("--truth", default="")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    plot(a.path, a.truth, a.out)
