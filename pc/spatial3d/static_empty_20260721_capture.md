# Static empty-room cube capture — 2026-07-21

Material for the static-scene occlusion background (LEGO 面/柱 model). See the build spec in the
`static-scene-lego-model` design notes.

## Data
- **File:** `pc/case/static_empty_20260721.npz` (1.7 MB)
- **Keys:** `bins, snapshots, counts, covariances, mean, dr_m` (same format as `spatial3d.cube.save_cube`)
- **Coverage:** 63 range bins, bin 1–63 → **0.11–6.68 m** (reaches the 6.2 m front wall)
- **Snapshots/bin:** min 75 / med 125 / max 200 (per-bin 16-antenna zero-Doppler vectors → 16×16 covariance)
- **Range resolution (dr_m):** 0.106 m/bin (pose65s 6.5 m cfg)

## Environment
- **Room:** EMPTY (person outside during the whole sweep — a moving person would pollute the static cloud)
- **Radar:** AWRL6844EVM, mount height 2.0 m, down-tilt 25° (radar-frame data; apply mount/tilt to get world z/range)
- **Firmware:** people_tracking_6844 (pose/FALLSM image), cfg **pose65s** = `sbr_3dpt_6p5m_pose_128.cfg`
  (128 ADC samples, 64 range bins, R_max 6.8 m, dr 0.106 m). NOT the mmw_demo static-demo firmware.

## Method — cubeQuery layered sweep (people_tracking, NOT mmw_demo)
Uses the firmware's `cubeQuery <range_bin> <half_win> <n_frames>` CLI (mmw_cli.c:MmwDemo_CLICubeQuery),
which forces a TLV-320 burst at a bin window **track-independent** — so it works in an empty room with
no person/track. `TBC_MAX_ENTRIES=40` caps half_win at 19 (≤39 bins/shot), hence TWO shots:
- **shot 1:** `cubeQuery 20 19 25` → bins 1–39
- **shot 2:** `cubeQuery 48 16 25` → bins 32–64  (overlap 32–39)
- **5 rounds**, 60 s between rounds (anti-wedge; the firmware `cubeGuardCfg 300 300 3000` also caps
  cube to ≤300 frames per 300 s, so a flood can't wedge the sensor). Total ~50 cube-frames/round × 5.

Per-bin 16-antenna vectors accumulate across rounds → covariances → `save_cube`.

## Tool
`pc/spatial3d/cube_sweep.py` — attaches to the live pose65s stream (sends the cfg first to sensorStart
if the sensor is idle), runs the 2-shot × 5-round sweep, saves the npz. Re-runnable.

## Caveat
Rounds 3–4 shot-1 returned 0 entries (likely a per-window budget/timing hiccup), so the near bins 1–31
(shot-1 only) carry ~75 snapshots vs 125–200 elsewhere — still ample for a 16×16 covariance, no re-capture
needed. If a future capture wants it even, shorten the round gap or interleave the two shots per round.

## Next
Build the MUSIC static voxel map from `covariances` (相位定角 MUSIC + 功率定质量) → the LEGO background
screen. Compare against `case/emptyL_cube.npz` (older fullroom empty, 2.04–6.33 m) as a sanity cross-check.
