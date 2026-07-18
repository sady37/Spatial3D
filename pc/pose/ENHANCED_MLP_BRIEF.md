# Server-side TRACK-INDEPENDENT fall detector — design brief (NO MLP)

## ⭐ DECISION 2026-07-18: DROP the MLP entirely (firmware AND server-side "MLP")
Data proved TI's on-chip pose MLP is a STRUCTURAL dead-end for falls, so fall detection uses
NO per-track classifier — only track-INDEPENDENT scene features off the 3001 cloud + cube:
1. **Track-gated dead-end (primary):** the MLP runs once per LIVE GTRACK track. GTRACK
   allocates tracks from MOTION, so a person lying still is DROPPED — measured in fall_013500:
   30 s of a 36 s lie had ZERO tracks, so the MLP never ran during the exact pose it should
   detect. It "never sends fall" because it never gets to look.
2. **Feature mismatch (secondary):** even the few tracked frames output Stood despite posZ=-0.13
   (correctly low) — the on-chip raw radar-frame posZ + uncalibrated `zOffset_cm=0` are out of
   the training distribution, so the net falls back to the majority class Stood. Calibrating
   zOffset would fix (2) but NOT (1), so the MLP can never be the fall detector.
Empirical: across 8 recordings / ~11,400 track-frames the firmware MLP emitted Stood=9220,
Sat=30, unknown=2146, **Lying=0, Falling=0**; falling_prob max ever 0.247. => feature #1 below
is a dead constant — REMOVE it (its 0.22 weight in `_fall_fuse` is dead).

## Objective (revised)
Formalize/tune the **track-INDEPENDENT** scene-feature fall pipeline (already ~built as
`_fall_fuse` + `floor_fall` + sustained + geometry + recovery + collapse-suspect) into a clean
transparent scorer + pose read-out, and close the hard cases (far falls, GTRACK-drop, chest-
blockage half-kneel, furniture rejection, fast recovery). All legs read the 3001 cloud / cube,
NONE need a per-track box or the firmware MLP. Runs in `radar_server.py`, validated by replay.

## Why (context)
- The firmware MLP (TLV 321) is dead (see DECISION above) — do not use it, do not train a
  replacement per-track MLP; the fall pose (lying) is exactly when there is no track.
- Current server logic is a **hand-tuned weighted fusion** `_fall_fuse()` + the track-free
  `floor_fall` leg + sustained-down + a 30 s recovery cancel. It works but the weights are
  guessed; this brief tunes them (optionally with a SMALL model on the scene features only,
  never per-track). Keep everything track-independent.

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

---

## IMPLEMENTATION & RESULTS (2026-07-18)

Built server-side in `radar_server._scene()` (no firmware), validated by `fall_replay.py`
(code-of-record). Three deliverables:

### 1. ⭐ Cardiac / collapse-suspect flag  (`collapse_suspect`, `collapse_conf`)
Encodes brief insight #1 (no-RR = SIGNAL, not downgrade). A sustained red Fall (`down_dur >=
COLLAPSE_SUSTAIN_S=12`) with **fallen geometry** (floor_fall OR pose LIE OR flat/folded OR a
below-floor mass) and **breathing never confirmed** on the episode (`_fall_had_rr` false)
ESCALATES — it can never clear on "no RR". `collapse_conf`: **strong** = cube bursts were spent
at the fall spot and still returned no RR (measured apnea); **weak** = we never got to measure
(far fall / cloud collapse — a fall we can't rule out breathing on). Episode-scoped (resets on
`CUBE_RESET_S` of quiet). Exposed top-level + in `fall_ev`.
- **Fires**: `fall_013500` (the ⭐ chest-blockage half-kneel, rr=None throughout) and
  `fall_231000` (no-RR 2.5 m GTRACK-drop far fall). **Silent** on 215500/222000/222500/
  231500/000000 (breathing confirmed → not cardiac).

#### ⭐ REVISION 2026-07-18b — micro-motion living-confirm + honest two-tier (from the record/ audit)
A furniture/empty audit of yesterday's `record/live_scene_*` (via a sub-agent) found NO
no-person false positives (the only clean empty, `214000`, is correctly rejected; gates hold).
But it surfaced a REAL problem: **`💔` over-fired on 4 real BREATHING people** (fall rehearsals
`133500/143000/152000/191500`) purely because the cube never LOCKED RR — and *"no RR
confirmed" ≠ apnea*. A decisive cube-forensics experiment settled the design:
- The 4 rehearsals' recorded cube HAS breathing-band micro-motion (band-frac 0.11–0.28, a
  plausible per-bin rhythm) — the estimator just couldn't lock a confident RR.
- The true cardiac `013500 #5` and the far `231000` have **NO recorded 320 at all** in the fall
  (genuinely UNASSESSED), not "measured silence".
So (per the user's steer *"RR 或微动 → 是人；能测 RR 更佳，没 RR 也可接受"*):
1. **Living-body confirm = RR-lock OR cube micro-motion** (`_fall_living`; `_rr_from_cube` now
   returns `(rr, strength, micro, measured)`). A breathing person the RR-lock misses is still
   confirmed living → NOT a cardiac collapse.
2. **Honest two-tier** (`collapse_conf`): **strong 💔** = cube MEASURED the chest and found it
   silent (no RR + no micro = genuine apnea) → the real cardiac alarm, highest priority.
   **weak ⚠️** = cube never returned data (unassessed) → labelled "跌倒-生命体征未确认" (a red
   FALL with breathing we couldn't check), explicitly **NOT** a cardiac claim (no-RR is accepted,
   the fall is not downgraded).
- Result: **zero false 💔 cardiac alarms** in all data — the 4 rehearsals + 231000 + 013500 are
  now ⚠️ weak (vitals-unconfirmed), and 💔 strong is reserved for measured apnea (needs a
  recording that actually captures 320 during a silent chest — a data gap to fill).
- LIMIT (replay ceiling): the recorded 320 is SPARSE (only where the original live run queried),
  so replay can seldom feed the micro-motion path — `191500` was the one case a burst landed on
  recorded 320 and confirmed living. A "settled-body cube gate" was tried to steer the tiny
  3-burst budget onto the lie but it fragmented the red on clean falls (215500 1→6) and was
  reverted; live deployment (where `request_cube` always returns data) will exercise micro-motion
  far more than replay can. To truly validate 💔 strong vs weak, capture a scene with continuous
  320 through a real breath-hold/apnea lie.

### 2. Fall-ONSET event counter  (`fall_event`)  — latch-blind re-segmentation
The 30 s display latch MERGES falls <30 s apart into one `fall_state` episode. `fall_event`
counts distinct onsets from the PRE-latch red trigger, re-arming only after a genuine RECOVERY
(whole-cloud centroid risen `cloud_up`, held `FALL_EVENT_GAP_S=4`). Gating on the stand-up (not
raw `down` clearing) is what makes it robust — a far-fall cloud collapse drops `down` while the
body is still on the floor but the mass never rises, so it does NOT falsely re-arm.
- Events vs truth: 215500 **1/1**, 222000 **1/1**, 231500 **2/2**, 000000 **2/2** (exact);
  222500 2/3, 231000 2/3, 013500 4/5 (off-by-one). The 3 misses are FUNDAMENTAL, not tuning:
  222500's middle 4.5 m fall's cloud fully collapses (no fresh `down` — survives only as the
  latch), 231000's "mid" fall never sustains past the 10 s gate (stays 🟡suspected), 013500's
  #3/#4 are a 1 s flicker-split of one physical fall. Closing 222500 needs a far-range
  cube-energy-at-bin leg (per [[fall-replay-harness]]); not chased here to avoid overfitting 7 files.
- Purpose is RE-ALERT (fresh alarm when a recovered person falls again), for which
  recovery-gated re-arm is the correct semantics regardless of exact count.

### 3. Feature extractor + labeler + interpretable fusion  (`pose/scene_features.py`)
`extract` drives the REAL `_scene()` over the npz set (reusing `fall_replay`) and dumps the
per-frame 6-class feature vector + code decision + collapse + event → `record/scene_feats.npz`.
Label = physical cloud-height (`f_height < 0` sustained), INDEPENDENT of the decision legs
(non-circular). `train` fits a Leave-One-Recording-Out logistic fusion.
- **Mean LORO-CV AUC = 0.924** (000000 .96, 215500 .97, 222000 .98, 222500 .93, 231000 .88,
  231500 .95, 013500 .80): the 6 features linearly predict the physical fallen-state and
  generalize across held-out recordings.
- Learned weights rank: **f_win .88 > floor_fall .86 > down_dur .74 > f_geom .42** — agrees with
  the hand-tuned `_fall_fuse`. **f_mlp ≈ 0** empirically confirms the firmware MLP (falling_prob
  =0) is dead weight in every current recording. `rr_absent` is NOT a fall predictor (slightly
  negative) — confirming no-RR is the CARDIAC-escalation signal on an already-detected fall, not
  a fall signal itself (so the collapse flag correctly gates on fallen-geometry first).
- HONEST SCOPE: ~7 recordings makes this a calibration / feature-validation sanity check, not a
  deployable trained classifier. The two production wins above need NO training. A real trained
  scene model needs a larger labelled set (fresh empty/furniture scene-format captures included —
  the current empty_/emptychair_ files are the old vitals-cube schema and can't replay `_scene`).

### Regression
All `fall_state` episode counts UNCHANGED (215500→1, 222000→1, 222500→2, 231000→2, 231500→2,
000000→2, 013500→5) — the additions are new flags/outputs and do not touch the validated
decision path.
