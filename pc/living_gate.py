"""Living-person occupancy gate — RR-band, spatial, scene-invariant.

Rationale (validated 2026-07-13, empty-chair vs person):
  Amplitude (disp_rms) is scene-dependent (empty room 0.0017 vs empty chair 0.0057)
  -> any fixed amplitude threshold leaks. A MOVED chair also makes motion. The ONLY
  living-person signature is a SUSTAINED, LOCALIZED breathing rhythm:
    - spatial CONCENTRATION: RR-band energy peaks at ONE range (chest), scene-invariant
      as a peak/mean RATIO. empty ~1.8 vs person 2.7-5.3.
    - tight CLUSTER: the top RR bins span a chest-sized region (~0.05-0.08 m) vs
      empty scattered (~0.23 m).
    - inter-bin RR AGREEMENT: bins agree on one rate. empty ~2.9 vs person 1.0-1.5 rpm.
  A single window gives present/absent; a moved chair (transient) is rejected by
  requiring PERSISTENCE across consecutive windows.

    .venv/bin/python3 living_gate.py        # validate on empty/person cubes
"""
import numpy as np
from bcg_vitals import bandpass, fft_peak, RR_LO, RR_HI

# thresholds from 2026-07-13 empty-chair/person data (refine as more scenes accrue).
# NOTE inter-bin RR spread was DROPPED — per-window it does NOT separate (empty 3.3 <
# person 3.7-3.8, even inverted). The discriminators are SPATIAL: concentration + cluster.
CONC_MIN = 2.6       # RR-band SQI peak/mean (spatial concentration); empty~2.1, person 3.1-5.9
SPAN_MAX = 0.15      # m; top-K RR bin cluster span (chest-sized); empty~0.20, person 0.08-0.10
TOPK = 6


def living_window(chans, bins, dr, fps):
    """One window -> living-person present? + diagnostics. chans=(nbin,T) mm displacement.
    Two SPATIAL discriminators (scene-invariant): RR-band energy is (a) concentrated
    (peak/mean) and (b) tightly clustered (chest-sized) at one range."""
    rr_sqi = np.array([np.std(bandpass(c, fps, RR_LO, RR_HI)) for c in chans])
    conc = float(rr_sqi.max() / (rr_sqi.mean() + 1e-12))
    top = np.argsort(rr_sqi)[::-1][:TOPK]
    span = float(np.std(bins[top].astype(float))) * dr
    rrf = [fft_peak(chans[i], fps, RR_LO, RR_HI) * 60 for i in top
           if fft_peak(chans[i], fps, RR_LO, RR_HI)]
    rr = float(np.median(rrf)) if rrf else None
    rng = float(np.median(bins[top].astype(float))) * dr
    present = (conc >= CONC_MIN) and (span <= SPAN_MAX)
    return dict(present=present, rr=rr, conc=round(conc, 2), span=round(span, 3),
                range_m=round(rng, 2))


def living_present(chans, bins, dr, fps, win_s=30.0, step_s=10.0, persist=0.4):
    """Persistence-gated occupancy: present iff >= `persist` fraction of windows pass.
    Rejects a moved-chair transient (one bad window) and empty-scene flukes."""
    n = chans[0].shape[0]; win = min(int(win_s * fps), n); step = max(int(step_s * fps), 1)
    flags, diags = [], []
    for s in range(0, n - win + 1, step):
        w = chans[:, s:s + win] if isinstance(chans, np.ndarray) else np.array([c[s:s + win] for c in chans])
        r = living_window(w, bins, dr, fps)
        flags.append(r["present"]); diags.append(r)
    frac = float(np.mean(flags)) if flags else 0.0
    return dict(present=frac >= persist, present_frac=round(frac, 2), n_win=len(flags),
                rr=np.median([d["rr"] for d in diags if d["rr"]]) if diags else None)


def _main():
    from bcg_vitals import demod_channels
    cases = [("emptychair_20260713_192151.npz", "空椅(无人)"),
             ("chairL_20260713_183013.npz", "chairL(有人)"),
             ("chairL_20260713_184015.npz", "chairR(有人)"),
             ("emptyL_cube.npz", "空房")]
    print("living-person gate — per-window present fraction (want empty~0, person~1):\n")
    for fn, lab in cases:
        try:
            d = np.load(fn, allow_pickle=True)
            cube = np.asarray(d["snapshots"], np.complex64)
            counts = d["counts"].astype(int); bins = d["bins"].astype(int); dr = float(d["dr_m"])
            try:
                ts = d["frame_ts"]; fps = len(ts) / (ts[-1] - ts[0])
            except KeyError:
                fps = 18.8
            chans = demod_channels(cube[:, :int(counts.min()), :], bins)
            r = living_present(chans, bins, dr, fps)
            v = "PERSON" if r["present"] else "empty"
            print(f"  {lab:14} -> {v:6}  present_frac={r['present_frac']:.0%}  "
                  f"({r['n_win']} win)  RR={r['rr'] if r['rr'] else '--'}")
        except FileNotFoundError:
            print(f"  {lab:14} (missing)")


if __name__ == "__main__":
    _main()
