# case/ — fall + still-person analysis recordings

Live People_Tracking scene recordings (npz, cap_320 schema: `ts`, `t_*` per-track,
`e_*` TLV-320 cube, `pc_xyz` 3001 cloud). Server-side fall pipeline + FloorTracker +
cube-RR analysis.

## fall_215500.npz — the complete scenario (2026-07-17 21:55, 227 s)
Walk to far chair (ChairR ~3 m) → sit 50 s → **fall** → lie ~30 s → get up → walk
back to NEAR chair (~0.9 m) → sit → disappear. Timeline (geometry pose from cloud):

| t(s) | wy | geom pose | down% | cloud wz | what |
|---|---|---|---|---|---|
| 0–45 | 2.9 m | SIT | 0 | +0.49 | sitting far chair |
| 60 | 3.5 m | STAND | 52 | −0.15 | **fall onset** |
| 75–105 | 3.6 m | **LIE** | 96–100 | **−0.22** | **lying on floor ~30 s** (cube RR≈10.8) |
| 120 | 2.3 m | STAND | 31 | +0.54 | getting up |
| 135–180 | 0.9 m | STAND→SIT | 0 | +0.7 | walked to NEAR chair, sitting |
| 195–225 | — (no track) | — | 0 | +0.7 | GTRACK dropped near sitter; lost-probe 150-frame burst → **firmware WEDGED** |

Findings:
- Fall correctly signalled: sustained down 96–100 % + geometry LIE + cloud below the
  floor line (−0.22 m) + cube RR 10.8. Geometry pose (SIT/LIE/STAND) is reliable here
  (the firmware MLP `t_pose` is mostly "Stood" — 6432-geometry, ignore it).
- The near-chair "disappearance" == the 150-frame (15 s) lost-probe cubeQuery WEDGING
  the firmware ([NO-Done] + no frames). Fixed: lost-probe reverted to 60 frames;
  request_cube hard-caps any single cubeQuery at 300 frames (30 s).

## fall_213500.npz — earlier fall/RR session (21:35, 197 s)
Track present 57 % (flickery). 4 cube bursts, interpolated RR 11.6 / 20.7 / 10.3 /
11.1 rpm (was quantized to 10/20 before parabolic interpolation). Down episodes short.

## Known walls (measured on these)
- Weak-target breathing SNR ≈ 1 (micro-motion ~0.5–1 µm ≈ noise floor); only close
  chest bins reach SNR ~2–3. Long coherent integration is the only lift — but via
  STACKED short cube bursts (sliding buffer), never one long burst (wedges the fw).

## fall_222500.npz — 3-fall session (22:25, 278 s), REPLAY-VALIDATED
Ground truth (user): **3 falls**; the MIDDLE one was at ~4.5 m, behind ChairR.
Real-code verdict via `web/fall_replay.py` (drives the live `radar_server._scene`):
**2 fall episodes** — #1 158.5–220.2 s, #2 236.7–277.5 s. The middle 4.5 m fall
(≈198–222 s) is NOT independently detected — it only reads red because it lands inside
fall #1's 30 s latch. At 198–219 s the replay shows `real=0, w=0`: the `real_person`
gate (n≥12 cloud points, radar_server.py:475) fails because a lying body at 4.5 m
collapses the 3001 cloud below 12 points (0 points for 204–216 s). => **far falls (>~3.5 m)
are a real gap**; only the cube energy at that range still sees the body.

## Replay harness — web/fall_replay.py
Feeds an npz back through the REAL `srv._scene()` (fake source + recorded ts as the clock
+ synchronous cube fetch from the recorded 320 vectors), so the fall count is the LIVE
CODE's, reproducibly — not an ad-hoc script that drifts. `python3 web/fall_replay.py
case/<f>.npz`. Validated: 215500→1, 222000→1 (match manual), 222500→2 (reveals the gap).
