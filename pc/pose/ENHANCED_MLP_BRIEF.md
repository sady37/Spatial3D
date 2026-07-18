# Server-side ENHANCED pose/fall MLP — design brief

## Objective
Build a **server-side** classifier that fuses TI's on-chip per-track pose MLP with the
scene-level features the firmware can't see, to output **pose + fall-probability + a
cardiac/collapse flag**, and to close the hard cases the current hand-tuned pipeline still
struggles with (far falls, GTRACK-drop, chest-blockage half-kneel, furniture rejection,
fast recovery). Runs in `pc/web/radar_server.py` (Python), validated by replay — NOT firmware.

## Why server-side (context)
- The firmware MLP (TLV 321) reads `falling_prob = 0` in every recording we have — it only
  sees per-track points and isn't emitting useful fall motion. The server has the WHOLE scene.
- Current server logic is a **hand-tuned weighted fusion** `_fall_fuse()` (radar_server.py) +
  a track-independent `floor_fall` leg + sustained-down + a 30 s recovery cancel. It works but
  the weights are guessed. This brief upgrades it to a **trained** model on the same features.

## Feature vector (per track / per fallen-region, per frame or short window)
1. **TI MLP** — pose one-hot (Stood/Sat/Lying/Falling) + falling_prob (TLV 321).
2. **Below-floor energy density** — fraction of the region cloud below the floor band
   (`cloud_below_frac`; per-box `floor_frac`).
3. **Cloud height** — whole-cloud + per-box median world-Z (`cloud_wz_med`), and the min/2nd-
   highest (window-leg `h_s`).
4. **Temporal window** — last N frames (5–8) of the height / down-state (sustained-down slope,
   how long below floor). This is what separates a transient crouch from a real fall.
5. **RR + strength** (cube) — AND an explicit **`rr_absent_while_fallen`** flag (see insight ⭐1).
6. **Geometry XY/XZ/YZ** — box extents + flatness aspect (`_flatness`): lying is flat/spread,
   standing is a tall column, **half-kneel is folded (mid aspect, mid height)**.
7. **Context** — track present/lost, GTRACK-death-nearby, ground range (near vs >3.5 m far).

## Outputs
- Base pose: **Stood / Sat / Lying / Falling**.
- **Fall probability P** ∈ [0,1] (replaces the guessed `_fall_fuse` weights).
- ⭐ A distinct **Collapse/Cardiac-suspect** flag: fallen-geometry (Lying OR half-kneel) +
  immobile (sustained) + **no RR** — the chest-blockage case, the most critical.

## ⭐ Insights that MUST be encoded (learned from the case recordings)
1. **No-RR is the SIGNAL for a cardiac/chest collapse, not a downgrade.** The cube-RR gate
   ("no living body on floor → not red") is BACKWARDS for a heart/chest emergency (breathing
   is weak/absent — that IS the emergency). A fallen-GEOMETRY body that is immobile with NO RR
   must ESCALATE (Cardiac-suspect), never clear. `fall_013500` end fall proves it (caught only
   by sustained-down, cube rr=None throughout).
2. **Half-kneel is the hardest pose.** A collapse frozen half-kneeling (slumped on a toilet,
   head down, knees propped, z_med ~ +0.3–0.5) reads geometry SIT/STAND and never trips the
   `floor_fall` z<0.15 gate. Needs the geometry+temporal legs, not the flat-floor test.
3. **Far falls (>3.5 m) collapse the cloud** (<12 pts, GTRACK drops) → the n≥12 real-person
   gate and cube both fail; hold via sustained + latch. `fall_222500` (middle 4.5 m),
   `fall_231500` (two 4.5 m), `fall_000000` (fall B).
4. **GTRACK-drop + fragmentation** → prim=None; the track-independent floor leg (below-floor
   aggregate, death-armed) is required. `fall_231000` fall A (500 pts, 100% below floor, 60 s,
   was TOTALLY missed before the floor leg).
5. **Furniture rejection** — a persistent below-floor blob with NO death nearby + never a
   tracked walk-in = furniture; must never latch (the 22-min false-fall wedge).
6. **Recovery** — cloud centroid up for RECOVER_S cancels the fall (a self-recovered stumble
   isn't an emergency). Do NOT use the GTRACK track-Z (it floats ~1 m on a still body).

## Training approach (no big labelled scene dataset)
- **Base pose leg**: retrain from TI `classes.zip` (already done, `pc/pose/`) — per-track points.
- **Scene fusion**: LABEL the case recordings (semi-auto: `fall_replay.py` timeline + the
  cloud-height ground truth + the per-recording scenario notes below → per-window labels), then
  extract the feature vectors and train a SMALL model (logistic / gradient-boosted trees / tiny
  MLP — small data, keep it interpretable). Seed weights from the current `_fall_fuse`.
- Validate by `fall_replay.py` on held-out recordings (it drives the REAL `_scene`).

## Existing code to build on
- `pc/pose/` — dataset.py / model.py / train.py (base MLP from classes.zip).
- `radar_server.py` — `_fall_fuse()` (6-feature hand fusion), `_flatness()`, `floor_fall` leg,
  recovery, cube-RR (`_rr_from_cube`), all feature computations already exist in `_scene()`.
- `pc/web/fall_replay.py` — the code-of-record validation harness (npz → real `_scene`).
- `pc/falldet/` — window / mlp / clean / floor_track modules.

## Test-case matrix (case/*.npz — ground truth from live testing)
| recording | scenario | truth | what it tests |
|---|---|---|---|
| fall_215500 | ChairR sit 50s→fall→lie 30s→up→near chair | 1 | clean near fall + recovery |
| fall_222000 | sit ChairR, fall, back to near chair | 1 | clean baseline |
| fall_222500 | 3 falls, MIDDLE at 4.5 m behind ChairR | 3 | far-fall (4.5 m) detection |
| fall_231000 | fall A @2.5 m GTRACK-drop + mid + fall B | ~3 | GTRACK-drop floor-fall leg |
| fall_231500 | two far falls @4.5 m | 2 | far-fall sustain through 0-cloud collapse |
| fall_000000 | fall A @2.4 m + fall B @4.2 m + recovery | 2 | recovery overhang / cloud-centroid clear |
| fall_013500 | 5 falls incl **chest-blockage half-kneel** at end | 5 | ⭐ cardiac/half-kneel (no RR) |
| chairL_sit / chairR / lie_* / sit_* / stand_* | posture references | pose | base pose classification |
| empty_* / emptychair_* | no person / furniture | 0 falls | furniture/empty rejection (no false red) |

Current code-of-record replay counts (baseline to beat): 215500→1, 222000→1, 222500→2 (middle
4.5 m latch-covered not independent), 231000→2, 231500→2, 000000→2, 013500→5.
