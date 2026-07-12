"""Probe candidate OCCUPANCY features on empty vs occupied cubes, to pick a
clean presence discriminator before wiring a gate. A stationary person always
has COHERENT breathing (0.12-0.6Hz): strong band energy, high periodicity, and
the SAME rate across chest bins. Empty-room noise has none of these.

Features per cube (median over resp-SQI top-8 bins unless noted):
  resp_sqi     : E_resp / (E_total - E_resp)  -- breathing band concentration
  resp_str     : autocorr peak height in resp band -- breathing periodicity
  rr_spread    : std of per-bin RR fft peaks (rpm) -- inter-bin agreement (LOW=coherent)
  disp_rms     : RMS of resp-bandpassed displacement (mm) -- coherent chest motion
"""
import sys
import numpy as np
from bcg_vitals import demod_channels, bandpass, sqi, fft_peak, autocorr_peak, RR_LO, RR_HI

CASES = [("emptyT", "emptyT_cube.npz", "EMPTY"),
         ("sit39", "sit39_cube.npz", "occ"),
         ("sidesit", "sidesit_cube.npz", "occ"),
         ("lie41", "lie41_cube.npz", "occ"),
         ("fall20", "fall20_cube.npz", "occ")]
FPS = 18.8


def probe(path, fps, topk=8):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], dtype=np.complex64)
    K = int(d["counts"].astype(int).min()); bins = d["bins"].astype(int)
    cube = cube[:, :K, :]
    chans = demod_channels(cube, bins)
    resp_sqi = np.array([sqi(bandpass(c, fps, RR_LO, RR_HI), fps, RR_LO, RR_HI)
                         for c in chans])
    top = np.argsort(resp_sqi)[::-1][:topk]
    rr_f, strs, rms = [], [], []
    for i in top:
        b = bandpass(chans[i], fps, RR_LO, RR_HI)
        ff = fft_peak(chans[i], fps, RR_LO, RR_HI)
        if ff:
            rr_f.append(ff * 60)
        _, st = autocorr_peak(b, fps, int(RR_LO * 60), int(RR_HI * 60))
        strs.append(st)
        rms.append(float(np.sqrt(np.mean(b ** 2))))
    return dict(
        resp_sqi=float(np.median(resp_sqi[top])),
        resp_str=float(np.median(strs)),
        rr_spread=float(np.std(rr_f)) if len(rr_f) > 1 else 99.0,
        disp_rms=float(np.median(rms)),
        rr_med=float(np.median(rr_f)) if rr_f else 0.0)


if __name__ == "__main__":
    print(f"{'case':8} {'label':6} {'resp_sqi':>9} {'resp_str':>9} "
          f"{'rr_spread':>9} {'disp_rms':>9} {'rr_med':>7}")
    for name, path, lab in CASES:
        try:
            m = probe(path, FPS)
        except FileNotFoundError:
            print(f"{name:8} (missing)"); continue
        print(f"{name:8} {lab:6} {m['resp_sqi']:9.3f} {m['resp_str']:9.3f} "
              f"{m['rr_spread']:9.2f} {m['disp_rms']:9.4f} {m['rr_med']:7.1f}")
