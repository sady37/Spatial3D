"""Voxel-level vitals: DON'T collapse the whole range bin. Beamform to each (az,el)
direction inside the chest range bins and read HR from that VOXEL — the idea being the
precordium (cardiac point source) sits at a different (az,el) than the diaphragm
(breathing), so some voxel isolates the cardiac that the whole-bin mean-steering buries.
4x4 UPA lambda/2 array (music.py approximation). Test on tachy2 (near, true 131->91):
is there a voxel reading ~120-131 while the breathing-centroid reads ~66-81?

    python voxel_vitals.py
"""
import numpy as np
from bcg_vitals import bandpass, sqi, autocorr_peak, demod_channels, estimate_rr, LAMBDA_MM, RR_LO, RR_HI

FPS = 18.78
# 4x4 UPA az/el indices (units of lambda/2), matches music.py make-array
AZ_IDX = np.array([a for e in range(4) for a in range(4)], float)
EL_IDX = np.array([e for e in range(4) for a in range(4)], float)


def steer(az, el):
    return np.exp(1j * np.pi * (AZ_IDX * np.sin(az) * np.cos(el) + EL_IDX * np.sin(el)))


def voxel_disp(X, az, el):
    """X:(T,16) -> phase-demod displacement of the (az,el) beam."""
    z = X @ steer(az, el).conj()
    phi = np.unwrap(np.angle(z))
    return -LAMBDA_MM / (4 * np.pi) * (phi - phi.mean())


d = np.load("tachy2_cube.npz", allow_pickle=True)
cube = np.asarray(d["snapshots"], np.complex64)
counts = d["counts"].astype(int); bins = d["bins"].astype(int)
C = cube[:, :int(counts.min()), :]
chans = demod_channels(C, bins)
_, f0, _, _ = estimate_rr(chans, FPS)
rr = np.array([sqi(bandpass(c, FPS, RR_LO, RR_HI), FPS, RR_LO, RR_HI) for c in chans])
chest = int(np.argsort(rr)[::-1][0])
X = C[chest]
print(f"tachy2 chest bin {bins[chest]} ({bins[chest]*0.0234:.2f}m); true HR 131->91")

# baseline: whole-bin mean-steering (current method)
base = chans[chest]
b_hr, b_h = autocorr_peak(bandpass(base, FPS, 1.0, 2.5), FPS, 60, 150)
print(f"whole-bin mean-steering: HR={b_hr:.0f} (autocorr h={b_h:.2f})")

# scan az x el
azs = np.deg2rad(np.arange(-40, 41, 5))
els = np.deg2rad(np.arange(-40, 41, 5))
best = []
for az in azs:
    for el in els:
        disp = voxel_disp(X, az, el)
        sig = bandpass(disp, FPS, 1.0, 2.5)
        # tachy-band clarity: autocorr peak height restricted to 1.7-2.3Hz (102-138bpm)
        hr, h = autocorr_peak(sig, FPS, 102, 138)
        # breathing suppression: RR-band RMS (lower = better isolated cardiac)
        br = np.sqrt(np.mean(bandpass(disp, FPS, RR_LO, RR_HI) ** 2))
        if hr:
            best.append((h / (br + 1e-6), hr, h, np.rad2deg(az), np.rad2deg(el), br))
best.sort(reverse=True)
print("\ntop voxels by tachy-clarity / breathing-RMS:")
print(f"  {'az':>5} {'el':>5} {'HR':>5} {'clarity':>7} {'breathRMS':>9}")
for score, hr, h, az, el, br in best[:8]:
    print(f"  {az:>5.0f} {el:>5.0f} {hr:>5.0f} {h:>7.2f} {br*1000:>8.1f}um")
# also: does ANY voxel read the true tachy 120-131 with decent clarity?
tachy_voxels = [(hr, h, az, el) for score, hr, h, az, el, br in best if 115 <= hr <= 135 and h > 0.3]
print(f"\nvoxels reading 115-135bpm with h>0.3: {len(tachy_voxels)}")
for hr, h, az, el in tachy_voxels[:6]:
    print(f"  az={az:.0f} el={el:.0f} HR={hr:.0f} h={h:.2f}")
