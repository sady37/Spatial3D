"""MUSIC (MUltiple SIgnal Classification) super-resolution DOA estimation.

Server-side module for the AWRL6844 radar's 4T4R TDM-MIMO virtual antenna
array.  Runs on the server to refine FFT-based angle estimates using per-
antenna complex data, achieving angular resolution far below the FFT
beamwidth (~29 deg for a 4-element ULA).

Typical pipeline:
    1. Radar streams per-antenna complex snapshots for each range-Doppler cell
    2. estimate_covariance() builds the spatial covariance matrix
    3. music_doa() or refine_angles() produces high-resolution angle estimates

References:
    R.O. Schmidt, "Multiple emitter location and signal parameter estimation,"
    IEEE Trans. Antennas Propagation, vol. AP-34, pp. 276-280, Mar. 1986.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

# ---------------------------------------------------------------------------
# Physical constants for AWRL6844 at 60 GHz band
# ---------------------------------------------------------------------------
FREQ_CENTER = 60.5e9          # center frequency (Hz)
C = 3e8                        # speed of light (m/s)
LAMBDA = C / FREQ_CENTER       # wavelength ~4.96 mm


# ---------------------------------------------------------------------------
# AntennaArray
# ---------------------------------------------------------------------------
class AntennaArray:
    """Virtual antenna array geometry for 2D MUSIC.

    Parameters
    ----------
    positions_az : (N,) array
        Azimuth positions of each virtual antenna, in metres.
    positions_el : (N,) array
        Elevation positions of each virtual antenna, in metres.
    wavelength : float
        Operating wavelength in metres.
    """

    def __init__(
        self,
        positions_az: NDArray[np.floating],
        positions_el: NDArray[np.floating],
        wavelength: float,
    ) -> None:
        self.positions_az = np.asarray(positions_az, dtype=np.float64)
        self.positions_el = np.asarray(positions_el, dtype=np.float64)
        self.wavelength = wavelength
        self.n_antennas = len(self.positions_az)
        assert len(self.positions_el) == self.n_antennas

    def steering_vector(self, az_rad: float, el_rad: float) -> NDArray[np.complexfloating]:
        """Steering vector for a single look-direction.

        a(az, el) = exp(j * 2*pi/lambda *
                        (d_az * sin(az)*cos(el) + d_el * sin(el)))

        Parameters
        ----------
        az_rad : float   Azimuth angle in radians.
        el_rad : float   Elevation angle in radians.

        Returns
        -------
        (N,) complex128 array
        """
        k = 2.0 * np.pi / self.wavelength
        phase = k * (
            self.positions_az * np.sin(az_rad) * np.cos(el_rad)
            + self.positions_el * np.sin(el_rad)
        )
        return np.exp(1j * phase)

    def steering_matrix(
        self,
        az_rads: NDArray[np.floating],
        el_rads: NDArray[np.floating],
    ) -> NDArray[np.complexfloating]:
        """Steering vectors for a grid of (az, el) pairs.

        Parameters
        ----------
        az_rads : (M,) array of azimuth angles (rad)
        el_rads : (P,) array of elevation angles (rad)

        Returns
        -------
        (M, P, N) complex128 array — one steering vector per grid point.
        """
        az_grid, el_grid = np.meshgrid(az_rads, el_rads, indexing="ij")
        k = 2.0 * np.pi / self.wavelength
        # (M, P, 1) broadcast with (N,) positions
        phase = k * (
            self.positions_az[np.newaxis, np.newaxis, :]
            * np.sin(az_grid)[..., np.newaxis]
            * np.cos(el_grid)[..., np.newaxis]
            + self.positions_el[np.newaxis, np.newaxis, :]
            * np.sin(el_grid)[..., np.newaxis]
        )
        return np.exp(1j * phase)


# ---------------------------------------------------------------------------
# Covariance estimation
# ---------------------------------------------------------------------------
def estimate_covariance(
    snapshots: NDArray[np.complexfloating],
    spatial_smoothing: int = 0,
) -> NDArray[np.complexfloating]:
    """Sample covariance matrix from K time snapshots.

    Parameters
    ----------
    snapshots : (K, N_antennas) complex array
        K independent snapshots of the antenna outputs for one
        range-Doppler cell.
    spatial_smoothing : int
        If > 0, apply forward-backward spatial smoothing with the given
        sub-array size.  Useful for coherent (correlated) sources.
        0 means no smoothing (plain sample covariance).

    Returns
    -------
    R : (N, N) or (L, L) complex Hermitian covariance matrix
        where L = spatial_smoothing when smoothing is applied.
    """
    snapshots = np.asarray(snapshots, dtype=np.complex128)
    K, N = snapshots.shape

    if spatial_smoothing <= 0:
        # Standard sample covariance: R = (1/K) sum_k x_k x_k^H
        # With rows-as-snapshots layout: R = X.T @ X.conj() / K
        R = (snapshots.T @ snapshots.conj()) / K
        return R

    # --- Forward-backward spatial smoothing ---
    L = spatial_smoothing
    if L > N:
        raise ValueError(
            f"spatial_smoothing ({L}) must be <= n_antennas ({N})"
        )
    n_subarrays = N - L + 1

    R = np.zeros((L, L), dtype=np.complex128)
    # Exchange matrix for backward averaging
    J = np.fliplr(np.eye(L))

    for i in range(n_subarrays):
        sub = snapshots[:, i : i + L]          # (K, L)
        R_fwd = (sub.T @ sub.conj()) / K
        R_bwd = J @ R_fwd.conj() @ J
        R += (R_fwd + R_bwd)

    R /= 2 * n_subarrays
    return R


# ---------------------------------------------------------------------------
# 2-D forward-backward spatial smoothing (for a UPA)
# ---------------------------------------------------------------------------
def spatial_smoothing_2d(
    snapshots: NDArray[np.complexfloating],
    grid: tuple[int, int] = (4, 4),
    sub: tuple[int, int] = (3, 3),
    forward_backward: bool = True,
) -> NDArray[np.complexfloating]:
    """Covariance with 2-D spatial smoothing over a planar array.

    The 1-D smoothing in :func:`estimate_covariance` treats the 16 virtual
    antennas as a line, which is wrong for the AWRL6844's 4x4 grid. This
    decorrelates *coherent* sources (e.g. several static scatterers in one
    range bin, whose antenna response is identical every frame) by averaging
    the covariance over all overlapping (sub_el x sub_az) sub-arrays.

    Antenna ordering must match :func:`awrl6844_array`: index = el*grid_az + az
    (elevation-major). Pair the returned covariance with
    :func:`subarray_array` so the steering geometry matches.

    Parameters
    ----------
    snapshots        : (K, grid_el*grid_az) complex array.
    grid             : (n_el, n_az) full array shape.
    sub              : (sub_el, sub_az) sub-array shape (<= grid per axis).
    forward_backward : also average the backward (conjugate-reversed) covariance.

    Returns
    -------
    R : (sub_el*sub_az, sub_el*sub_az) complex Hermitian covariance.
    """
    snapshots = np.asarray(snapshots, dtype=np.complex128)
    K, N = snapshots.shape
    n_el, n_az = grid
    s_el, s_az = sub
    if n_el * n_az != N:
        raise ValueError(f"grid {grid} has {n_el*n_az} elements != {N} antennas")
    if s_el > n_el or s_az > n_az:
        raise ValueError(f"sub {sub} exceeds grid {grid}")

    L = s_el * s_az
    X = snapshots.reshape(K, n_el, n_az)
    R = np.zeros((L, L), dtype=np.complex128)
    J = np.fliplr(np.eye(L))
    n_sub = 0
    for p in range(n_el - s_el + 1):
        for q in range(n_az - s_az + 1):
            sub_x = X[:, p:p + s_el, q:q + s_az].reshape(K, L)  # (K, L)
            R_fwd = (sub_x.T @ sub_x.conj()) / K
            if forward_backward:
                R += 0.5 * (R_fwd + J @ R_fwd.conj() @ J)
            else:
                R += R_fwd
            n_sub += 1
    R /= max(n_sub, 1)
    return R


def subarray_array(
    sub: tuple[int, int] = (3, 3),
    wavelength: float = LAMBDA,
) -> "AntennaArray":
    """AntennaArray matching a (sub_el, sub_az) sub-array from spatial_smoothing_2d.

    Elements are on a lambda/2 grid, ordered elevation-major to match the
    smoothed covariance produced by :func:`spatial_smoothing_2d`.
    """
    s_el, s_az = sub
    half_lambda = wavelength / 2.0
    az_idx, el_idx = [], []
    for e in range(s_el):
        for a in range(s_az):
            az_idx.append(a)
            el_idx.append(e)
    return AntennaArray(
        np.asarray(az_idx, dtype=np.float64) * half_lambda,
        np.asarray(el_idx, dtype=np.float64) * half_lambda,
        wavelength,
    )


# ---------------------------------------------------------------------------
# MDL signal-number estimation
# ---------------------------------------------------------------------------
def mdl_estimate_signals(
    eigenvalues: NDArray[np.floating],
    n_snapshots: int,
) -> int:
    """Estimate the number of signals using the MDL criterion.

    Parameters
    ----------
    eigenvalues : (N,) real array, sorted in *ascending* order.
    n_snapshots : int  (K — number of time snapshots used).

    Returns
    -------
    n_signals : int
    """
    eigenvalues = np.asarray(eigenvalues, dtype=np.float64)
    N = len(eigenvalues)

    # Ensure ascending order
    eigenvalues = np.sort(eigenvalues)

    mdl_vals = np.zeros(N)

    for k in range(N):
        # k = hypothesised number of signals (0 .. N-1)
        noise_eigs = eigenvalues[: N - k]
        m = len(noise_eigs)
        if m == 0:
            mdl_vals[k] = 0
            continue

        # Geometric and arithmetic means of the smallest (N-k) eigenvalues
        log_eigs = np.log(np.maximum(noise_eigs, 1e-30))
        geo_mean_log = np.mean(log_eigs)            # log of geometric mean
        arith_mean = np.mean(noise_eigs)
        arith_mean_log = np.log(max(arith_mean, 1e-30))

        # Log-likelihood component
        log_likelihood = -n_snapshots * m * (geo_mean_log - arith_mean_log)

        # Penalty (number of free parameters)
        penalty = 0.5 * k * (2 * N - k) * np.log(n_snapshots)

        mdl_vals[k] = log_likelihood + penalty

    return int(np.argmin(mdl_vals))


# ---------------------------------------------------------------------------
# 1-D MUSIC spectrum
# ---------------------------------------------------------------------------
def music_spectrum_1d(
    R: NDArray[np.complexfloating],
    array: AntennaArray,
    scan_angles: NDArray[np.floating],
    n_signals: int | None = None,
    n_snapshots: int = 100,
    dimension: str = "az",
    fixed_angle: float = 0.0,
) -> NDArray[np.floating]:
    """1-D MUSIC pseudo-spectrum, scanning one angular dimension.

    Parameters
    ----------
    R             : (N, N) covariance matrix.
    array         : AntennaArray
    scan_angles   : (M,) angles to scan, in radians.
    n_signals     : int or None (auto-detect via MDL).
    n_snapshots   : int, used only if n_signals is None (MDL).
    dimension     : 'az' | 'el' — which axis to scan.
    fixed_angle   : float (rad) — the value of the *other* dimension.

    Returns
    -------
    spectrum : (M,) real array — pseudo-spectrum values (linear scale).
    """
    R = np.asarray(R)
    N = R.shape[0]

    # Eigen-decomposition (ascending eigenvalue order)
    eigvals, eigvecs = np.linalg.eigh(R)

    if n_signals is None:
        n_signals = mdl_estimate_signals(eigvals, n_snapshots)
    n_signals = max(1, min(n_signals, N - 1))

    # Noise subspace: eigenvectors for the (N - n_signals) smallest eigenvalues
    En = eigvecs[:, : N - n_signals]  # (N, N-d)

    spectrum = np.empty(len(scan_angles), dtype=np.float64)
    for i, angle in enumerate(scan_angles):
        if dimension == "az":
            a = array.steering_vector(angle, fixed_angle)
        else:
            a = array.steering_vector(fixed_angle, angle)
        # P(theta) = 1 / (a^H En En^H a)
        proj = En.conj().T @ a          # (N-d,)
        denom = np.real(np.dot(proj.conj(), proj))
        spectrum[i] = 1.0 / max(denom, 1e-30)

    return spectrum


# ---------------------------------------------------------------------------
# 2-D MUSIC spectrum
# ---------------------------------------------------------------------------
def music_spectrum_2d(
    R: NDArray[np.complexfloating],
    array: AntennaArray,
    az_angles: NDArray[np.floating],
    el_angles: NDArray[np.floating],
    n_signals: int | None = None,
    n_snapshots: int = 100,
) -> NDArray[np.floating]:
    """2-D MUSIC pseudo-spectrum over an azimuth x elevation grid.

    Parameters
    ----------
    R          : (N, N) covariance matrix.
    array      : AntennaArray
    az_angles  : (M,) azimuth scan angles (rad).
    el_angles  : (P,) elevation scan angles (rad).
    n_signals  : int or None (auto via MDL).
    n_snapshots: int (for MDL).

    Returns
    -------
    spectrum : (M, P) real array — pseudo-spectrum in linear scale.
    """
    R = np.asarray(R)
    N = R.shape[0]

    eigvals, eigvecs = np.linalg.eigh(R)

    if n_signals is None:
        n_signals = mdl_estimate_signals(eigvals, n_snapshots)
    n_signals = max(1, min(n_signals, N - 1))

    En = eigvecs[:, : N - n_signals]  # (N, N-d)

    # Vectorised: build all steering vectors at once
    A = array.steering_matrix(az_angles, el_angles)  # (M, P, N)

    # Project each steering vector onto noise subspace
    # proj[m, p, :] = En^H @ a[m, p, :]  -> (M, P, N-d)
    proj = np.einsum("kd,mpk->mpd", En.conj(), A)
    denom = np.real(np.sum(proj * proj.conj(), axis=-1))  # (M, P)
    spectrum = 1.0 / np.maximum(denom, 1e-30)

    return spectrum


# ---------------------------------------------------------------------------
# High-level DOA estimation
# ---------------------------------------------------------------------------
def music_doa(
    R: NDArray[np.complexfloating],
    array: AntennaArray,
    n_signals: int | None = None,
    n_snapshots: int = 100,
    az_range: tuple[float, float] = (-60.0, 60.0),
    el_range: tuple[float, float] = (-60.0, 60.0),
    resolution_deg: float = 1.0,
) -> list[tuple[float, float, float]]:
    """Estimate directions-of-arrival via 2D MUSIC + peak finding.

    Parameters
    ----------
    R              : (N, N) covariance matrix.
    array          : AntennaArray
    n_signals      : int or None.
    n_snapshots    : int (for MDL).
    az_range       : (min_deg, max_deg) azimuth scan range.
    el_range       : (min_deg, max_deg) elevation scan range.
    resolution_deg : grid spacing in degrees.

    Returns
    -------
    detections : list of (az_deg, el_deg, power) tuples, sorted by power
                 descending.
    """
    az_angles = np.deg2rad(
        np.arange(az_range[0], az_range[1] + resolution_deg / 2, resolution_deg)
    )
    el_angles = np.deg2rad(
        np.arange(el_range[0], el_range[1] + resolution_deg / 2, resolution_deg)
    )

    spectrum = music_spectrum_2d(
        R, array, az_angles, el_angles,
        n_signals=n_signals, n_snapshots=n_snapshots,
    )

    # Convert to dB for peak finding
    spectrum_db = 10.0 * np.log10(spectrum / spectrum.max() + 1e-30)

    # Find peaks: threshold at -20 dB below maximum, require some prominence
    detections: list[tuple[float, float, float]] = []

    # Flatten, find peaks, unflatten
    flat = spectrum_db.ravel()
    # Use 1-D peak finding on flattened array; also check neighbours in 2D
    M, P = spectrum_db.shape

    for i in range(1, M - 1):
        for j in range(1, P - 1):
            val = spectrum_db[i, j]
            if val < -20.0:
                continue
            # Check 8-connected neighbourhood
            neighbourhood = spectrum_db[i - 1 : i + 2, j - 1 : j + 2]
            if val >= neighbourhood.max():
                az_deg = float(np.rad2deg(az_angles[i]))
                el_deg = float(np.rad2deg(el_angles[j]))
                power = float(spectrum[i, j])
                detections.append((az_deg, el_deg, power))

    # Sort by power descending
    detections.sort(key=lambda x: x[2], reverse=True)
    return detections


# ---------------------------------------------------------------------------
# Focused refinement around FFT-based estimates
# ---------------------------------------------------------------------------
def refine_angles(
    detections: list[dict],
    complex_data: NDArray[np.complexfloating],
    array: AntennaArray,
    fft_az: NDArray[np.floating] | None = None,
    fft_el: NDArray[np.floating] | None = None,
    search_halfwidth_deg: float = 15.0,
    resolution_deg: float = 0.5,
    n_snapshots: int = 1,
) -> list[dict]:
    """Refine FFT-estimated angles via focused MUSIC around each detection.

    This is the key production function.  It takes coarse FFT angle
    estimates and per-detection complex antenna data, then runs a
    focused MUSIC search in a narrow window around each estimate.

    Parameters
    ----------
    detections : list of dicts with keys 'az_deg', 'el_deg', and any
                 other fields to carry through.  Each dict corresponds
                 to one point-cloud detection.
    complex_data : (N_detections, N_antennas) complex array —
                   per-antenna complex amplitude for each detection.
                   If only one snapshot per cell, shape is (N_det, N_ant).
    array : AntennaArray
    fft_az, fft_el : ignored (present for API compatibility); angles
                     come from *detections*.
    search_halfwidth_deg : half-width of the focused search window.
    resolution_deg : scan grid spacing for the focused search.
    n_snapshots : number of snapshots that went into complex_data
                  (for MDL, if applicable).

    Returns
    -------
    refined : list of dicts, same as *detections* but with 'az_deg'
              and 'el_deg' updated to MUSIC-refined values.  An
              additional key 'music_power' is added.
    """
    complex_data = np.asarray(complex_data)
    refined = []

    for idx, det in enumerate(detections):
        az0 = det["az_deg"]
        el0 = det["el_deg"]

        # Build single-snapshot covariance (outer product)
        x = complex_data[idx]  # (N_antennas,)
        R = np.outer(x, x.conj())

        # Focused scan window
        az_lo = az0 - search_halfwidth_deg
        az_hi = az0 + search_halfwidth_deg
        el_lo = el0 - search_halfwidth_deg
        el_hi = el0 + search_halfwidth_deg

        az_scan = np.deg2rad(
            np.arange(az_lo, az_hi + resolution_deg / 2, resolution_deg)
        )
        el_scan = np.deg2rad(
            np.arange(el_lo, el_hi + resolution_deg / 2, resolution_deg)
        )

        spec = music_spectrum_2d(R, array, az_scan, el_scan, n_signals=1)

        # Find peak
        peak_idx = np.unravel_index(np.argmax(spec), spec.shape)
        best_az = float(np.rad2deg(az_scan[peak_idx[0]]))
        best_el = float(np.rad2deg(el_scan[peak_idx[1]]))
        best_power = float(spec[peak_idx])

        out = dict(det)
        out["az_deg"] = best_az
        out["el_deg"] = best_el
        out["music_power"] = best_power
        refined.append(out)

    return refined


# ---------------------------------------------------------------------------
# Default AWRL6844 array
# ---------------------------------------------------------------------------
def awrl6844_array() -> AntennaArray:
    """Return the default AntennaArray for xWRL6844EVM (4T x 4R TDM-MIMO).

    TODO: verify exact antenna positions from the TI SDK's
          ``antGeometryBoard xWRL6844EVM`` definition.  The positions
          below assume a 4x4 Uniform Planar Array (UPA) with lambda/2
          spacing, which is a reasonable approximation for the standard
          EVM layout.

    The 16 virtual antennas are arranged on a 4x4 grid:
        Azimuth indices  : 0, 1, 2, 3  (in units of lambda/2)
        Elevation indices: 0, 1, 2, 3  (in units of lambda/2)

    Returns
    -------
    AntennaArray configured for the AWRL6844 at 60.5 GHz.
    """
    half_lambda = LAMBDA / 2.0

    # 4x4 UPA: enumerate all (az_idx, el_idx) pairs
    az_indices = []
    el_indices = []
    for el_idx in range(4):
        for az_idx in range(4):
            az_indices.append(az_idx)
            el_indices.append(el_idx)

    positions_az = np.array(az_indices, dtype=np.float64) * half_lambda
    positions_el = np.array(el_indices, dtype=np.float64) * half_lambda

    return AntennaArray(positions_az, positions_el, LAMBDA)


# ---------------------------------------------------------------------------
# Self-test / demo
# ---------------------------------------------------------------------------
def _generate_synthetic_data(
    array: AntennaArray,
    sources: list[tuple[float, float]],
    snr_db: float = 20.0,
    n_snapshots: int = 200,
    rng: np.random.Generator | None = None,
) -> NDArray[np.complexfloating]:
    """Generate synthetic array snapshots for testing.

    Parameters
    ----------
    array     : AntennaArray
    sources   : list of (az_deg, el_deg) for each source.
    snr_db    : per-element SNR in dB.
    n_snapshots : number of time snapshots.
    rng       : random generator.

    Returns
    -------
    snapshots : (n_snapshots, N_antennas) complex array.
    """
    if rng is None:
        rng = np.random.default_rng(42)

    N = array.n_antennas
    D = len(sources)

    # Build steering matrix A: (N, D)
    A = np.zeros((N, D), dtype=np.complex128)
    for d, (az_deg, el_deg) in enumerate(sources):
        A[:, d] = array.steering_vector(np.deg2rad(az_deg), np.deg2rad(el_deg))

    # Source signals: uncorrelated complex Gaussian
    S = (rng.standard_normal((D, n_snapshots))
         + 1j * rng.standard_normal((D, n_snapshots))) / np.sqrt(2)

    # Noise
    noise_power = 10 ** (-snr_db / 10)
    noise = np.sqrt(noise_power / 2) * (
        rng.standard_normal((N, n_snapshots))
        + 1j * rng.standard_normal((N, n_snapshots))
    )

    # Received data: X = A @ S + noise
    X = A @ S + noise  # (N, n_snapshots)
    return X.T  # (n_snapshots, N)


def _run_demo() -> None:
    """Self-test: resolve two sources 5 deg apart (below FFT beamwidth)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    print("=" * 60)
    print("MUSIC super-resolution demo  --  AWRL6844 (4T4R, 16 virt)")
    print("=" * 60)

    array = awrl6844_array()
    print(f"Array: {array.n_antennas} virtual antennas, lambda = {LAMBDA*1e3:.2f} mm")

    # Two sources close together: 5 deg apart in both az and el
    src1 = (20.0, 10.0)
    src2 = (25.0, 15.0)
    sources = [src1, src2]
    print(f"True sources: {src1}, {src2}  (separation: 5 deg az, 5 deg el)")

    # FFT beamwidth for a 4-element ULA at lambda/2: ~0.886*lambda/(4*lambda/2) rad
    fft_bw = np.rad2deg(0.886 * LAMBDA / (4 * LAMBDA / 2))
    print(f"FFT beamwidth (4-element ULA): ~{fft_bw:.1f} deg  --> CANNOT resolve")

    # Generate data
    n_snapshots = 200
    snapshots = _generate_synthetic_data(array, sources, snr_db=20.0,
                                         n_snapshots=n_snapshots)
    print(f"Generated {n_snapshots} snapshots, SNR = 20 dB")

    # Covariance
    R = estimate_covariance(snapshots)

    # MDL signal count
    eigvals = np.sort(np.linalg.eigvalsh(R))
    n_est = mdl_estimate_signals(eigvals, n_snapshots)
    print(f"MDL estimated number of signals: {n_est}")

    # Full 2D MUSIC DOA
    print("\nRunning 2D MUSIC DOA estimation ...")
    dets = music_doa(R, array, n_signals=2, n_snapshots=n_snapshots,
                     az_range=(0, 45), el_range=(-10, 35), resolution_deg=0.5)

    print(f"Detected {len(dets)} source(s):")
    for i, (az, el, pwr) in enumerate(dets):
        print(f"  Source {i+1}: az = {az:+.1f} deg, el = {el:+.1f} deg, "
              f"power = {10*np.log10(pwr/max(d[2] for d in dets)):.1f} dB rel")

    # Evaluate accuracy
    if len(dets) >= 2:
        err1_az = abs(dets[0][0] - src1[0])
        err1_el = abs(dets[0][1] - src1[1])
        err2_az = abs(dets[1][0] - src2[0])
        err2_el = abs(dets[1][1] - src2[1])

        # Match detections to sources (find best assignment)
        err_a = max(err1_az, err1_el, err2_az, err2_el)
        # Try swapped assignment
        err1s_az = abs(dets[0][0] - src2[0])
        err1s_el = abs(dets[0][1] - src2[1])
        err2s_az = abs(dets[1][0] - src1[0])
        err2s_el = abs(dets[1][1] - src1[1])
        err_b = max(err1s_az, err1s_el, err2s_az, err2s_el)

        if err_b < err_a:
            err1_az, err1_el = err1s_az, err1s_el
            err2_az, err2_el = err2s_az, err2s_el
            assignment = "swapped"
        else:
            assignment = "direct"

        max_err = min(err_a, err_b)
        print(f"\n  Max angular error: {max_err:.1f} deg ({assignment} assignment)")
        print(f"  Resolution achieved: sources 5 deg apart {'RESOLVED' if len(dets) >= 2 else 'NOT resolved'}")
    else:
        print("\n  WARNING: fewer than 2 sources detected")

    # --- Also test refine_angles ---
    print("\nTesting refine_angles() (focused MUSIC) ...")
    coarse_dets = [
        {"az_deg": 18.0, "el_deg": 8.0, "range_m": 2.5},   # rough estimate near src1
        {"az_deg": 27.0, "el_deg": 17.0, "range_m": 3.0},   # rough estimate near src2
    ]
    # Simulate per-detection complex data: each detection sees one source
    # (in production, each range-Doppler cell contains a single target)
    rng_ref = np.random.default_rng(99)
    a1_sv = array.steering_vector(np.deg2rad(src1[0]), np.deg2rad(src1[1]))
    a2_sv = array.steering_vector(np.deg2rad(src2[0]), np.deg2rad(src2[1]))
    sig1 = (rng_ref.standard_normal() + 1j * rng_ref.standard_normal()) / np.sqrt(2)
    sig2 = (rng_ref.standard_normal() + 1j * rng_ref.standard_normal()) / np.sqrt(2)
    noise_ref = np.sqrt(0.01 / 2) * (
        rng_ref.standard_normal((2, array.n_antennas))
        + 1j * rng_ref.standard_normal((2, array.n_antennas))
    )
    complex_data = np.vstack([
        (sig1 * a1_sv + noise_ref[0]).reshape(1, -1),
        (sig2 * a2_sv + noise_ref[1]).reshape(1, -1),
    ])
    refined = refine_angles(coarse_dets, complex_data, array,
                            search_halfwidth_deg=15.0, resolution_deg=0.5)
    for i, r in enumerate(refined):
        true_src = sources[i]
        err_az = abs(r["az_deg"] - true_src[0])
        err_el = abs(r["el_deg"] - true_src[1])
        print(f"  Detection {i+1}: coarse ({coarse_dets[i]['az_deg']:.0f}, "
              f"{coarse_dets[i]['el_deg']:.0f}) -> refined ({r['az_deg']:.1f}, "
              f"{r['el_deg']:.1f})  error: ({err_az:.1f}, {err_el:.1f}) deg")

    # --- Plot ---
    print("\nGenerating 2D spectrum plot ...")
    az_scan = np.deg2rad(np.arange(0, 45.5, 0.5))
    el_scan = np.deg2rad(np.arange(-10, 35.5, 0.5))
    spec_2d = music_spectrum_2d(R, array, az_scan, el_scan,
                                n_signals=2, n_snapshots=n_snapshots)
    spec_db = 10.0 * np.log10(spec_2d / spec_2d.max() + 1e-30)

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    # 2D heatmap
    ax = axes[0]
    az_deg = np.rad2deg(az_scan)
    el_deg = np.rad2deg(el_scan)
    im = ax.pcolormesh(az_deg, el_deg, spec_db.T, shading="auto",
                       cmap="hot", vmin=-30, vmax=0)
    ax.set_xlabel("Azimuth (deg)")
    ax.set_ylabel("Elevation (deg)")
    ax.set_title("2D MUSIC Spectrum")
    ax.plot(src1[0], src1[1], "c+", markersize=14, markeredgewidth=2, label="True")
    ax.plot(src2[0], src2[1], "c+", markersize=14, markeredgewidth=2)
    if len(dets) >= 2:
        ax.plot(dets[0][0], dets[0][1], "wo", markersize=8, markeredgewidth=2,
                fillstyle="none", label="MUSIC")
        ax.plot(dets[1][0], dets[1][1], "wo", markersize=8, markeredgewidth=2,
                fillstyle="none")
    ax.legend(loc="upper left")
    fig.colorbar(im, ax=ax, label="dB (rel)")

    # 1D azimuth slice at true elevation of source 1
    ax = axes[1]
    az_1d = np.deg2rad(np.arange(0, 45.1, 0.25))
    spec_1d = music_spectrum_1d(R, array, az_1d, n_signals=2,
                                dimension="az",
                                fixed_angle=np.deg2rad(src1[1]))
    spec_1d_db = 10.0 * np.log10(spec_1d / spec_1d.max() + 1e-30)
    ax.plot(np.rad2deg(az_1d), spec_1d_db, "b-", linewidth=1.5)
    ax.axvline(src1[0], color="r", linestyle="--", alpha=0.7, label=f"True {src1[0]} deg")
    ax.axvline(src2[0], color="r", linestyle="--", alpha=0.7, label=f"True {src2[0]} deg")
    ax.set_xlabel("Azimuth (deg)")
    ax.set_ylabel("Pseudo-spectrum (dB)")
    ax.set_title(f"1D MUSIC (el fixed at {src1[1]} deg)")
    ax.set_ylim(-40, 2)
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out_path = "music_test_spectrum.png"
    plt.savefig(out_path, dpi=150)
    print(f"Saved: {out_path}")
    print("\nDone.")


if __name__ == "__main__":
    _run_demo()
