"""A-E experiment: decouple soft-reset from elapsed time.

Cross-session covariance mismatch is ~44% while intra-stream is ~0.7%. Two
candidate causes were RULED OUT (diagonal phase step: cross-bin std 25-49deg,
correction ineffective; sub-bin sample-time shift: best delta=0.05, no
improvement). The remaining hypothesis is that the ROOM itself drifts between
hourly sessions (lambda=5mm -> any multipath contributor moving ~1mm rotates its
phase 144deg), and "soft reset" merely coincides with "a long time passed" and
took the blame. This experiment separates the two.

Five segments in ONE sitting, room undisturbed throughout:
  A            steady-state, no reset (after >=PREHEAT warmup)
  A->B  RESET  soft reset (sensorStop -> resend same cfg -> sensorStart), wait GAP
  B->C  ----   NO reset, stream kept alive, wait GAP
  C->D  RESET  soft reset, wait GAP
  D->E  ----   NO reset, wait GAP

Reset pairs: (A,B),(C,D).  No-reset pairs: (B,C),(D,E). Same GAP each, so the
only difference within a pair type is whether a reset happened.

Metrics per pair (median over shared bins):
  rel(R)  = ||Ri-Rj||_F / ||Ri||_F            covariance mismatch
  dirSim  = |mi^H mj| / (|mi||mj|)             mean-vector direction cosine
                                                (~10x more sensitive; 0.998 vs
                                                 0.968 is an obvious split)

Verdict:
  rel(A,B)~=rel(C,D)~=rel(B,C)~=rel(D,E)~=0.1  -> RESET INNOCENT; 44% is hourly
                                                  scene aging -> fix = baseline
                                                  refresh (rolling/nightly), not
                                                  calibration.
  reset pairs >> no-reset pairs                 -> reset has a real structural
                                                  effect -> dig deeper.

    python reset_ab_test.py --out reset_ab.npz --k 300 --gap 120 --preheat 600
"""
import argparse
import time

import numpy as np

from spatial3d.range_music import DR_M, N_VIRT_ANT, BinAccumulator
from spatial3d.uart_reader import RadarSession

CLI = "/dev/cu.usbmodem0000RA441"
DATA = "/dev/cu.usbmodem0000RA444"
CFG = "/Users/sady3721/project/TI/Tiinstall/profile_music_5fps_fullroom.cfg"


def collect_segment(session, k, start, num, timeout):
    """Collect K snapshots; return the accumulator (keeps the full cube)."""
    acc = BinAccumulator(k=k, n_ant=N_VIRT_ANT)
    bins = range(start, start + num)
    t0 = time.time()
    while time.time() - t0 < timeout and acc.min_count(bins) < k:
        f = session.get_frame(timeout=1.0)
        if f is None:
            continue
        ra = f.range_antenna()
        if ra is not None:
            acc.add(ra)
    return acc


def cov_mean(acc):
    """Per-bin covariance R and mean vector m from an accumulator."""
    covs, means = {}, {}
    for b, lst in acc.snaps.items():
        if len(lst) < 10:
            continue
        x = np.stack(lst, axis=0)                          # (K,16)
        covs[b] = ((x.conj().T @ x) / len(x)).astype(np.complex64)
        means[b] = x.mean(axis=0).astype(np.complex64)
    return covs, means


def pack_cube(acc, bins, k):
    """Aligned (M, k, 16) cube + counts for the given bins (zero-padded)."""
    cube = np.zeros((len(bins), k, N_VIRT_ANT), dtype=np.complex64)
    cnt = np.zeros(len(bins), dtype=np.int32)
    for i, b in enumerate(bins):
        x = np.stack(acc.snaps[b], axis=0)
        cube[i, : len(x)] = x
        cnt[i] = len(x)
    return cube, cnt


def drain(session, seconds):
    """Keep the stream alive (read frames) for *seconds* without sending CLI."""
    t0 = time.time()
    while time.time() - t0 < seconds:
        session.get_frame(timeout=0.5)


def soft_reset(session):
    """Reproduce the production event reset: sensorStop -> resend same cfg.

    send_cfg replays flushCfg + all params (factoryCalibCfg 0 1 = restore from
    flash) + sensorStart — i.e. exactly what a profile switch does, minus the
    profile change (same cfg here so ONLY the stop/reconfig/start cycle varies).
    """
    session.send_cli("sensorStop 0", wait=2.0, echo=False)
    session.send_cfg(CFG, echo=False)
    session.flush_frames()


def rel_cov(Ci, Cj):
    bs = sorted(set(Ci) & set(Cj))
    v = [np.linalg.norm(Ci[b] - Cj[b]) / (np.linalg.norm(Ci[b]) + 1e-9) for b in bs]
    return float(np.median(v)), len(bs)


def dir_sim(Mi, Mj):
    bs = sorted(set(Mi) & set(Mj))
    v = [abs(np.vdot(Mi[b], Mj[b])) / (np.linalg.norm(Mi[b]) * np.linalg.norm(Mj[b]) + 1e-12)
         for b in bs]
    return float(np.median(v))


def main():
    import os
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="reset_ab.npz")
    ap.add_argument("--k", type=int, default=300)
    ap.add_argument("--gap", type=float, default=120.0, help="wait between samples (s)")
    ap.add_argument("--preheat", type=float, default=180.0,
                    help="warmup if we had to send cfg (skip round1 transient)")
    ap.add_argument("--settle", type=float, default=30.0,
                    help="short settle if attaching to an already-running stream")
    ap.add_argument("--start", type=int, default=87)
    ap.add_argument("--num", type=int, default=184)
    ap.add_argument("--seg-timeout", type=float, default=90.0)
    ap.add_argument("--sit", action="store_true",
                    help="after A-E (empty), capture a SIT segment F in the SAME "
                         "stream (no reset) gated by --trigger -> clean detection")
    ap.add_argument("--trigger", default=None)
    ap.add_argument("--status", default=None)
    a = ap.parse_args()

    s = RadarSession(CLI, DATA)
    s.start_drain()
    # Attach to an already-running (warm, steady) stream if present, so segment A
    # has NO cfg/reset event before it. Only send cfg if nothing is streaming.
    live, t0 = 0, time.time()
    while time.time() - t0 < 6 and live < 5:
        f = s.get_frame(timeout=1.0)
        if f is not None and f.range_antenna() is not None:
            live += 1
    if live >= 5:
        print(f"attached to EXISTING stream (warm/steady) -> settle {a.settle:.0f}s; "
              "A = true steady-state, no reset before it", flush=True)
        drain(s, a.settle)
    else:
        print(f"no stream -> send cfg + preheat {a.preheat:.0f}s", flush=True)
        s.send_cfg(CFG, echo=False)
        drain(s, a.preheat)

    plan = [("A", None), ("B", "reset"), ("C", "wait"),
            ("D", "reset"), ("E", "wait")]
    accs, stamps = {}, {}
    for name, action in plan:
        if action == "reset":
            print(f"  [RESET] soft reset then wait {a.gap:.0f}s -> {name}", flush=True)
            soft_reset(s)
            drain(s, a.gap)
        elif action == "wait":
            print(f"  [wait ] no reset, wait {a.gap:.0f}s -> {name}", flush=True)
            drain(s, a.gap)
        stamps[name] = time.time()
        accs[name] = collect_segment(s, a.k, a.start, a.num, a.seg_timeout)
        print(f"  sampled {name}: {len(accs[name].snaps)} bins", flush=True)

    # optional SIT segment F in the SAME stream (no reset) -> coherent detection
    if a.sit:
        if a.status:
            with open(a.status, "w") as fh:
                fh.write("SIT_NOW\n")
        print("  >>> come in, SIT on the chair, hold still, then touch the "
              f"trigger: {a.trigger}", flush=True)
        while a.trigger and not os.path.exists(a.trigger):
            s.get_frame(timeout=0.3)
        stamps["F"] = time.time()
        accs["F"] = collect_segment(s, a.k, a.start, a.num, a.seg_timeout)
        print(f"  sampled F (SIT): {len(accs['F'].snaps)} bins", flush=True)
    s.close()

    # derive cov/mean, persist FULL cube per segment (canonical) + cov/mean
    covs, means = {}, {}
    for n in accs:
        covs[n], means[n] = cov_mean(accs[n])
    common = sorted(set.intersection(*[set(c) for c in covs.values()]))
    bins = np.array(common, dtype=np.int32)
    save = {"bins": bins, "dr_m": np.float32(DR_M),
            "names": np.array(list(accs), dtype=object)}
    for n in accs:
        cube, cnt = pack_cube(accs[n], common, a.k)
        save[f"cube_{n}"] = cube
        save[f"counts_{n}"] = cnt
        save[f"cov_{n}"] = np.stack([covs[n][b] for b in common]).astype(np.complex64)
        save[f"mean_{n}"] = np.stack([means[n][b] for b in common]).astype(np.complex64)
    np.savez(a.out, **save)

    # report: reset vs no-reset
    print("\n" + "=" * 60, flush=True)
    print(f"{'pair':>7} {'kind':>9} {'dt(s)':>7} {'rel(R)':>8} {'dirSim':>8}", flush=True)
    agg = {"RESET": [], "NO-RESET": []}
    for kind, pairs in [("RESET", [("A", "B"), ("C", "D")]),
                        ("NO-RESET", [("B", "C"), ("D", "E")])]:
        for i, j in pairs:
            r, _ = rel_cov(covs[i], covs[j]); d = dir_sim(means[i], means[j])
            agg[kind].append((r, d))
            print(f"  {i}-{j} {kind:>9} {stamps[j]-stamps[i]:>7.0f} {r:>8.3f} {d:>8.4f}",
                  flush=True)
    rr = np.mean([x[0] for x in agg["RESET"]]); dr_ = np.mean([x[1] for x in agg["RESET"]])
    rn = np.mean([x[0] for x in agg["NO-RESET"]]); dn = np.mean([x[1] for x in agg["NO-RESET"]])
    print("-" * 60, flush=True)
    print(f"  RESET    mean: rel(R)={rr:.3f}  dirSim={dr_:.4f}", flush=True)
    print(f"  NO-RESET mean: rel(R)={rn:.3f}  dirSim={dn:.4f}", flush=True)
    ratio = rr / rn if rn else float("inf")
    verdict = ("RESET INNOCENT -> 44% is hourly scene aging; fix = rolling baseline refresh"
               if ratio < 1.5 else
               "RESET has a real structural effect (reset >> no-reset) -> dig deeper")
    print(f"  reset/no-reset ratio = {ratio:.2f}  ->  {verdict}", flush=True)
    if "F" in accs:
        r, _ = rel_cov(covs["E"], covs["F"]); d = dir_sim(means["E"], means["F"])
        print(f"\n  DETECTION (same-stream, coherent):  E(empty) vs F(sit)  "
              f"rel(R)={r:.3f}  dirSim={d:.4f}", flush=True)
        print(f"    (compare rel to the ~0.1 no-reset floor: sit should stand out)",
              flush=True)
    print(f"  SAVED {a.out}", flush=True)


if __name__ == "__main__":
    main()
