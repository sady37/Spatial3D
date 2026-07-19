"""Is the e_vec beamforming even pointing the right way?

At dropout-EDGE frames the track-bin-cube events and the CFAR point cloud
coexist for the SAME person. Ground truth for that frame:
  - point cloud centroid (radar frame x,y,z)
  - track (t_x,t_y,t_z)
  - => truth azimuth = atan2(x, y), truth elevation = atan2(z, hypot(x,y))

Beamform each e_vec and compare. The AWRL6844 array model + antenna ORDERING
is only usable if beamformed (az,el) lands near truth. TLV 320 (track-bin-cube)
may pack the 16 virtual antennas differently than the type-8 slow-time cube the
array model was tuned on, so we try candidate reorderings and report which one
minimises the DOA error against ground truth.
"""
import numpy as np
from spatial3d.music import awrl6844_array
from spatial3d.range_music import spherical_to_cart

ARR = awrl6844_array()
AZ = np.deg2rad(np.arange(-60, 61, 1.0))
EL = np.deg2rad(np.arange(-70, 41, 1.0))
STEER = ARR.steering_matrix(AZ, EL)          # (naz,nel,16)


def perm_candidates():
    """16-index reorderings to try for the virtual-array mapping."""
    idx = np.arange(16).reshape(4, 4)          # (el,az) row-major default
    cands = {
        "identity": np.arange(16),
        "transpose(az<->el)": idx.T.reshape(-1),
        "flip_el": idx[::-1, :].reshape(-1),
        "flip_az": idx[:, ::-1].reshape(-1),
        "flip_both": idx[::-1, ::-1].reshape(-1),
        "transpose+flipel": idx.T[::-1, :].reshape(-1),
    }
    return cands


def beam_dom(evec):
    """Dominant (az,el) of a single 16-vector by Bartlett peak."""
    resp = np.abs(np.einsum("aek,k->ae", STEER.conj(), evec)) ** 2
    ia, ie = np.unravel_index(resp.argmax(), resp.shape)
    return AZ[ia], EL[ie]


def truth_angles(xyz):
    x, y, z = xyz
    return np.arctan2(x, y), np.arctan2(z, np.hypot(x, y))


def validate(path):
    d = np.load(path, allow_pickle=True)
    ef, er, ev = d["e_frame"], d["e_range"], d["e_vec"]
    pf, pxyz = d["p_frame"], d["pc_xyz"]
    tf, tx, ty, tz = d["t_frame"], d["t_x"], d["t_y"], d["t_z"]
    # overlap frames: events + >=5 pc points
    evf = np.unique(ef)
    ovl = [f for f in evf if (pf == f).sum() >= 5]
    print(f"\n===== {path.split('/')[-1]}  overlap frames: {ovl[:8]}")
    if not ovl:
        print("  (no overlap - cannot validate here)"); return
    cands = perm_candidates()
    # accumulate DOA error per candidate ordering across all overlap events
    err = {k: [] for k in cands}
    for f in ovl:
        pm = pf == f
        pcxyz = pxyz[pm]
        pc_c = pcxyz.mean(0)
        taz, tel = truth_angles(pc_c)
        tm = tf == f
        trk = (tx[tm][0], ty[tm][0], tz[tm][0]) if tm.any() else None
        print(f"  frame {f}: pc centroid xyz=({pc_c[0]:+.2f},{pc_c[1]:+.2f},"
              f"{pc_c[2]:+.2f})  truth az={np.degrees(taz):+.0f} el={np.degrees(tel):+.0f}"
              + (f"  track xyz=({trk[0]:+.2f},{trk[1]:+.2f},{trk[2]:+.2f})" if trk else ""))
        em = ef == f
        for evec in ev[em]:
            for k, p in cands.items():
                az, el = beam_dom(evec[p])
                err[k].append(np.hypot(np.degrees(az - taz),
                                       np.degrees(el - tel)))
    print("  --- mean DOA error vs pc-centroid truth (deg), by ordering ---")
    ranked = sorted(cands, key=lambda k: np.median(err[k]))
    for k in ranked:
        e = np.array(err[k])
        print(f"    {k:22s} median {np.median(e):5.1f}  mean {e.mean():5.1f}")
    best = ranked[0]
    print(f"  >> best ordering: {best}  (median {np.median(err[best]):.1f} deg)")
    return best


if __name__ == "__main__":
    import sys
    fs = sys.argv[1:] or ["case/fall_hk_chairR.npz", "case/fall_hk_chairR_2.npz",
                          "case/fall_hk_chairL_2.npz"]
    for f in fs:
        validate(f)
