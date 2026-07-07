"""Raw slow-time cube — the canonical captured artifact.

The firmware's type-8 TLV streams, per frame, the zero-Doppler 16-antenna
complex vector for each range bin. Accumulating K frames per bin gives a
``[bin, K, 16]`` complex cube — the *full* information the sensor delivers.
Everything downstream is a server-side VIEW of this cube:

    covariance R = (1/K) Σ xₖxₖᴴ         -> MUSIC angle (static structure)
    mean m       = (1/K) Σ xₖ            -> coherent static component
    variance     = trace(R) - |m|²        -> liveness (breathing) per bin
    R_fluc       = R - m·mᴴ               -> MUSIC on the MOVING part (breathing 3D)
    slow-time    = xₖ over k              -> Doppler / breathing-rate FFT

Collapsing the K axis into R (the old save format) throws away the mean and the
slow-time — exactly the information that separates a static object from a
breathing person (both give the SAME rank-1 R). So we archive the cube itself.
Cost is MB-scale (K=100, 184 bins, 16 ant, complex64 ≈ 2.3 MB) and needs no
firmware change — the type-8 stream already carries every frame.

    save_cube("cap.npz", acc, bins, dr=DR_M)   # after collection
    c = Cube.load("cap.npz")
    covs = c.covariances()      # {bin: R}  (drop-in for the old format)
    var  = c.variance()         # {bin: trace(R)-|m|²}  liveness
    flc  = c.fluctuation()      # {bin: R - m mᴴ}  -> MUSIC = moving scatterers
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from .range_music import DR_M, N_VIRT_ANT


def pack_snapshots(acc, bins, min_snapshots: int = 10):
    """Pack an accumulator's per-bin snapshots into an aligned cube.

    Returns ``(bins_arr, cube, counts)`` where *cube* is ``(M, Kmax, n_ant)``
    complex64 zero-padded to the largest K, and *counts* holds each bin's true
    snapshot count (so the padding is never mistaken for data).
    """
    keep = [int(b) for b in bins if len(acc.snaps.get(int(b), [])) >= min_snapshots]
    keep.sort()
    if not keep:
        return (np.empty(0, np.int32),
                np.empty((0, 0, N_VIRT_ANT), np.complex64),
                np.empty(0, np.int32))
    counts = np.array([len(acc.snaps[b]) for b in keep], dtype=np.int32)
    kmax = int(counts.max())
    n_ant = len(acc.snaps[keep[0]][0])
    cube = np.zeros((len(keep), kmax, n_ant), dtype=np.complex64)
    for i, b in enumerate(keep):
        s = np.stack(acc.snaps[b], axis=0)          # (K, n_ant)
        cube[i, : s.shape[0]] = s
    return np.array(keep, dtype=np.int32), cube, counts


def save_cube(path, acc, bins, dr: float = DR_M, min_snapshots: int = 10,
              **extra):
    """Save the raw slow-time cube (plus a derived covariance for back-compat).

    The npz carries ``snapshots`` (the cube), ``counts``, ``bins``, ``dr_m``,
    and — so existing MUSIC consumers keep working unchanged — the derived
    ``covariances``. Everything new (variance, fluctuation MUSIC, breathing FFT)
    reads ``snapshots``.
    """
    binsA, cube, counts = pack_snapshots(acc, bins, min_snapshots)
    covs = _covs(cube, counts)
    np.savez(path, bins=binsA, snapshots=cube, counts=counts,
             covariances=covs, dr_m=np.float32(dr), **extra)
    return binsA, cube, counts


def _covs(cube, counts):
    """Per-bin covariance R=(1/K)ΣxxᴴH from a padded cube, honouring counts."""
    n = cube.shape[0]
    out = np.zeros((n, cube.shape[2], cube.shape[2]), dtype=np.complex64)
    for i in range(n):
        k = int(counts[i])
        x = cube[i, :k]                              # (K, n_ant)
        out[i] = (x.conj().T @ x) / max(k, 1)
    return out


@dataclass
class Cube:
    """Loaded raw cube with lazy server-side views."""

    bins: NDArray[np.int32]
    cube: NDArray[np.complex64]        # (M, Kmax, n_ant)
    counts: NDArray[np.int32]
    dr: float = DR_M

    @classmethod
    def load(cls, path) -> "Cube":
        d = np.load(path, allow_pickle=True)
        if "snapshots" not in d:
            raise ValueError(
                f"{path} has no 'snapshots' cube (old covariance-only capture); "
                "re-capture with the cube-saving collector to get variance/breathing")
        return cls(bins=d["bins"].astype(np.int32),
                   cube=np.asarray(d["snapshots"], dtype=np.complex64),
                   counts=d["counts"].astype(np.int32),
                   dr=float(d["dr_m"]) if "dr_m" in d else DR_M)

    def _valid(self, i):
        return self.cube[i, : int(self.counts[i])]        # (K, n_ant)

    def means(self) -> dict[int, NDArray]:
        """Coherent mean vector m=(1/K)Σxₖ per bin (the static component)."""
        return {int(self.bins[i]): self._valid(i).mean(axis=0)
                for i in range(len(self.bins))}

    def covariances(self) -> dict[int, NDArray]:
        """Per-bin R=(1/K)Σxₖxₖᴴ (drop-in for the old covariance format)."""
        out = {}
        for i in range(len(self.bins)):
            x = self._valid(i)
            out[int(self.bins[i])] = (x.conj().T @ x) / max(len(x), 1)
        return out

    def fluctuation(self) -> dict[int, NDArray]:
        """Fluctuation covariance R_fluc = R - m·mᴴ per bin.

        The static component m·mᴴ is removed, leaving only what VARIES frame to
        frame — noise plus any moving (breathing) scatterer. MUSIC on R_fluc
        therefore localises the *moving* content in angle, i.e. the breathing
        person's 3D position, where MUSIC on R would be dominated by static
        clutter at the same range.
        """
        out = {}
        for i in range(len(self.bins)):
            x = self._valid(i)
            m = x.mean(axis=0)
            R = (x.conj().T @ x) / max(len(x), 1)
            out[int(self.bins[i])] = R - np.outer(m, m.conj())
        return out

    def variance(self) -> dict[int, float]:
        """Per-bin liveness scalar trace(R) - |m|² = Σ_ant Var over frames.

        ≈0 for a static object (constant snapshot), large for a breathing
        person (phase swings over the breath make m→0 while trace(R) stays).
        """
        out = {}
        for i in range(len(self.bins)):
            x = self._valid(i)
            total = float(np.mean(np.sum(np.abs(x) ** 2, axis=1)))   # trace(R)
            static = float(np.sum(np.abs(x.mean(axis=0)) ** 2))       # |m|²
            out[int(self.bins[i])] = total - static
        return out

    def slowtime(self, bin_idx: int) -> NDArray:
        """Slow-time complex series (K, n_ant) for one bin — for a breathing FFT."""
        i = int(np.where(self.bins == bin_idx)[0][0])
        return self._valid(i)
