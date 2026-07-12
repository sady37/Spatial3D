"""Scheme C on-device projection (validation-stage codec).

Key property: the server pipeline's FIRST step (demod_channels) already collapses the
16 antennas to ONE complex per bin per frame via the mean steering vector:
    m = C[i].mean(0)/|.|;  z = C[i] @ m.conj();  disp = -lam/4pi*unwrap(angle(z))
So doing that projection ON-DEVICE and transmitting only z (1 complex/bin/frame) is
LOSSLESS for everything the pipeline computes (HR/RR/occupancy/AF/tachy-slope) — the
16-antenna spatial detail was shown to add nothing for vitals. Bandwidth for the
1.8-4.2m gate (103 bins) @18.78fps IQ16 = ~62 kbps (fits an 87.5 kbps link); raw
16-antenna would be ~1 Mbps.

This module: encode(cube)->IQ16 z-stream, decode, demod_from_z, and a validation that
scheme-C outputs match the full-cube pipeline.

    python scheme_c.py            # validate losslessness + report kbps
"""
import numpy as np
from bcg_vitals import (demod_channels, estimate_rr, estimate_hr, occupancy,
                        LAMBDA_MM)
from bcg_vitals_rt import elevated_hr_trend


def project(cube):
    """On-device 16->1: per bin, mean-steering beamform -> z (nbin, T) complex64.
    Identical to demod_channels' internal projection (so demod_from_z == demod)."""
    z = np.empty((cube.shape[0], cube.shape[1]), np.complex64)
    for i in range(cube.shape[0]):
        m = cube[i].mean(0); m = m / (np.linalg.norm(m) + 1e-9)
        z[i] = cube[i] @ m.conj()
    return z


def encode_phase16(z):
    """Transmit WRAPPED PHASE only, int16 over [-pi,pi] (2 bytes/sample). The whole
    pipeline uses ONLY angle(z) (mm displacement lives in phase; amplitude is never
    consumed downstream), so this is lossless to 0.038um resolution — and half the
    bytes of IQ. IQ16 fails because a single amplitude scale wastes bits on the large
    static reflector, coarsening the tiny cardiac phase modulation below the signal."""
    q = np.round(np.angle(z) / np.pi * 32767.0)
    return np.clip(q, -32767, 32767).astype(np.int16)


def decode_phase16(q):
    return q.astype(np.float64) / 32767.0 * np.pi          # wrapped phase (nbin, T)


def demod_from_phase(phase_wrapped):
    """Server-side: unwrap the transmitted wrapped phase -> mm displacement (nbin,T)."""
    out = np.empty(phase_wrapped.shape, np.float64)
    for i in range(phase_wrapped.shape[0]):
        phi = np.unwrap(phase_wrapped[i])
        out[i] = -LAMBDA_MM / (4 * np.pi) * (phi - phi.mean())
    return out


def kbps(nbin, fps, bytes_per_sample=2):                    # phase16 = 2 bytes
    return nbin * bytes_per_sample * fps * 8 / 1e3


def validate(path, fps=18.78):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], dtype=np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    cube = cube[:, :int(counts.min()), :]

    # --- reference: full-cube pipeline ---
    ch_full = demod_channels(cube, bins)
    occ_f = occupancy(ch_full, fps)
    _, f0_f, _, _ = estimate_rr(ch_full, fps)
    hr_f = estimate_hr(ch_full, fps, f0_f)["hr"]

    # --- scheme C: project 16->1 on device -> transmit int16 WRAPPED PHASE -> server ---
    q = encode_phase16(project(cube))                       # the bytes on the wire
    phase = decode_phase16(q)
    ch_c = demod_from_phase(phase)
    occ_c = occupancy(ch_c, fps)
    _, f0_c, _, _ = estimate_rr(ch_c, fps)
    hr_c = estimate_hr(ch_c, fps, f0_c)["hr"]

    # elevated-trend re-demods per segment; feed a unit-amplitude pseudo-cube e^{j*phase}
    pseudo = np.exp(1j * phase)[:, :, None].astype(np.complex64)
    ev_f = elevated_hr_trend(cube, bins, fps)
    ev_c = elevated_hr_trend(pseudo, bins, fps)

    max_disp_err = float(np.max(np.abs(ch_full - ch_c)))
    print(f"{path:18s} | HR full={hr_f and round(hr_f)} C={hr_c and round(hr_c)} "
          f"| occ {occ_f['present']}/{occ_c['present']} "
          f"| elev {ev_f['elevated']}({ev_f['slope_rec']:+.0f})/"
          f"{ev_c['elevated']}({ev_c['slope_rec']:+.0f}) "
          f"| max|disp Δ|={max_disp_err*1000:.2f}um")


if __name__ == "__main__":
    print("=== Scheme-C losslessness (full-cube vs projected+IQ16) ===")
    for p in ("tachy2_cube.npz", "tachy3_cube.npz", "sit39_cube.npz",
              "lie41_cube.npz", "fall20_cube.npz"):
        validate(p)
    print("\n=== bandwidth (1.8-4.2m = 103 bins, scheme C) ===")
    for fps in (18.78, 20, 15):
        print(f"  {fps:>5}fps IQ16: {kbps(103, fps):.1f} kbps  (+30% ovh {kbps(103,fps)*1.3:.1f})")
