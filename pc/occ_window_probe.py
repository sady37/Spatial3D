"""Per-window occupancy feature distribution: the full-cube threshold does not
transfer to short windows (breathing needs several cycles). Measure disp_rms &
rr_spread over sliding windows of several lengths on empty vs a real seated
person, to pick an occupancy window length + threshold that separates at the
window scale used by the RT tracker."""
import numpy as np
from bcg_vitals import demod_channels, bandpass, sqi, fft_peak, RR_LO, RR_HI

FPS = 18.8
CASES = [("emptyT", "emptyT_cube.npz"), ("sit39", "sit39_cube.npz"),
         ("sidesit", "sidesit_cube.npz"), ("lie41", "lie41_cube.npz")]


def feats(chans, fps, topk=8):
    resp_sqi = np.array([sqi(bandpass(c, fps, RR_LO, RR_HI), fps, RR_LO, RR_HI)
                         for c in chans])
    top = np.argsort(resp_sqi)[::-1][:topk]
    rr_f, rms = [], []
    for i in top:
        b = bandpass(chans[i], fps, RR_LO, RR_HI)
        ff = fft_peak(chans[i], fps, RR_LO, RR_HI)
        if ff:
            rr_f.append(ff * 60)
        rms.append(float(np.sqrt(np.mean(b ** 2))))
    return (float(np.median(rms)) * 1000,                       # um
            float(np.std(rr_f)) if len(rr_f) > 1 else 99.0)


for win_s in (15, 30, 45):
    print(f"\n=== occupancy window {win_s}s  (disp_rms um / rr_spread rpm, "
          f"p10..p50..p90 over sliding windows) ===")
    for name, path in CASES:
        d = np.load(path, allow_pickle=True)
        cube = np.asarray(d["snapshots"], dtype=np.complex64)
        K = int(d["counts"].astype(int).min()); bins = d["bins"].astype(int)
        cube = cube[:, :K, :]
        w = int(win_s * FPS); hop = int(1.5 * FPS)
        ds, rs = [], []
        for i in range(0, max(1, K - w + 1), hop):
            dr, sp = feats(demod_channels(cube[:, i:i + w, :], bins), FPS)
            ds.append(dr); rs.append(sp)
        ds, rs = np.array(ds), np.array(rs)
        print(f"  {name:8} disp[{np.percentile(ds,10):5.1f} {np.percentile(ds,50):5.1f} "
              f"{np.percentile(ds,90):6.1f}]  spread[{np.percentile(rs,10):4.1f} "
              f"{np.percentile(rs,50):4.1f} {np.percentile(rs,90):4.1f}]  n={len(ds)}")
