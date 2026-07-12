"""Test the user's doubt: do the 7th/8th breathing harmonics (119/136) really carry
breathing energy, or is it an UNWRAP artifact (imperfect unwrap of the huge breathing
swing reintroduces the PM Bessel comb J_n(beta), which stays strong up to n~beta~12)?

If unwrap were perfect, phi(t) is linear in displacement -> only the breathing SHAPE
harmonics survive (fast decay). Strong n=6,7 would then be UNWRAP artifacts. Compare:
  (a) unwrap(angle(z))                       -- current demod
  (b) robust: cumsum of consecutive Delta-phi = angle(z[t]*conj(z[t-1]))
  (c) lowpass z before phase (denoise the carrier)
Measure harmonic amplitudes n=1..8; fit the breathing decay from n=1..4 (pure breathing,
below the cardiac band) and flag EXCESS at n=6,7 (where the 91-131 cardiac lives).
"""
import numpy as np
from bcg_vitals import bandpass, sqi, demod_channels, estimate_rr, LAMBDA_MM, RR_LO, RR_HI

FPS = 18.78


def phase_unwrap(z):
    return np.unwrap(np.angle(z))


def phase_robust(z):
    dphi = np.angle(z[1:] * np.conj(z[:-1]))        # per-sample increment in [-pi,pi]
    return np.concatenate([[0], np.cumsum(dphi)])


def phase_lowpass(z):
    # smooth the complex carrier a touch before angle (kills spike-induced wrap noise)
    k = 3
    zr = np.convolve(z.real, np.ones(k) / k, "same") + 1j * np.convolve(z.imag, np.ones(k) / k, "same")
    return np.unwrap(np.angle(zr))


def harm_amps(disp, f0, nh=8):
    disp = disp - disp.mean()
    f = np.fft.rfftfreq(len(disp), 1 / FPS)
    S = np.abs(np.fft.rfft(disp))
    return np.array([float(S[np.abs(f - n * f0) <= 0.03].max()) if (np.abs(f - n * f0) <= 0.03).any()
                     else 0.0 for n in range(1, nh + 1)])


d = np.load("tachy2_cube.npz", allow_pickle=True)
cube = np.asarray(d["snapshots"], np.complex64)
counts = d["counts"].astype(int); bins = d["bins"].astype(int)
C = cube[:, :int(counts.min()), :]
chans = demod_channels(C, bins)
_, f0, _, _ = estimate_rr(chans, FPS)
rr = np.array([sqi(bandpass(c, FPS, RR_LO, RR_HI), FPS, RR_LO, RR_HI) for c in chans])
ci = int(np.argsort(rr)[::-1][0])
X = C[ci]                                            # (T,16)
m = X.mean(0); m /= (np.linalg.norm(m) + 1e-9)
z = X @ m.conj()

print(f"tachy2 chest bin {bins[ci]}, f0={f0:.3f}Hz ({f0*60:.0f}rpm)")
print(f"harmonic bpm:  {[round(n*f0*60) for n in range(1,9)]}")
print(f"cardiac band (91-131) ~ harmonics n=5..7 ({[round(n*f0*60) for n in (5,6,7)]})\n")
for name, fn in [("unwrap (current)", phase_unwrap), ("robust dphi", phase_robust),
                 ("lowpass+unwrap", phase_lowpass)]:
    disp = -LAMBDA_MM / (4 * np.pi) * fn(z)
    A = harm_amps(disp, f0)
    # fit log-decay from n=1..4 (breathing only), extrapolate, excess at n=5..8
    n14 = np.arange(1, 5)
    good = A[:4] > 0
    if good.sum() >= 2:
        c = np.polyfit(n14[good], np.log(A[:4][good] + 1e-9), 1)
        base = np.exp(np.polyval(c, np.arange(1, 9)))
        exc = A / (base + 1e-9)
    else:
        exc = np.ones_like(A)
    print(f"{name:18s} amps={[round(a,1) for a in A]}")
    print(f"{'':18s} excess(vs n1-4 decay)={[round(e,2) for e in exc]}  "
          f"-> n5-7 excess {[round(exc[k],1) for k in (4,5,6)]}")
