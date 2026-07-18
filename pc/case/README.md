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
