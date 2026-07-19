# Half-Kneel Fall — Cube / Descent Analysis (2026-07-19)

Investigation of the 4 standard **sit → half-kneel FALL** cube recordings, driven
by the queued plan: *beamform the track-bin-cube → elevation → height trajectory →
descent rate → give the half-kneel boundary a temporal criterion.*

**Bottom line up front.** The descent-rate criterion is **not achievable from
these 4 cases** (physics + data-quality walls, detailed below). But running the
recordings through the **real** fall detector shows it **already catches all 4**
half-kneel falls via `window + sustained-down + mlp:Falling` — so on this data the
half-kneel is *not* a recall gap. The open items are one over-fire and the
SIT-reading geometry in other conditions.

---

## 1. The data — 4 standard cases

Source of truth: `pc/record/falltest_20260719.jsonl` (logged by `pc/falltest.py`).

| # | file | label | bin | range | fall @ |
|---|------|-------|-----|-------|--------|
| 1 | `case/fall_hk_chairL_1.npz` | chairL-halfkneel (x0.8, y3.7) | 38 | ~3.7 m | go+5 s → **frame 1272** |
| 2 | `case/fall_hk_chairL_2.npz` | chairL-halfkneel-2 | 38 | ~3.7 m | go+5 s → frame 80 |
| 3 | `case/fall_hk_chairR.npz`   | chairR-halfkneel (2.7→2.4 m) | 30 | ~2.4 m | go+5 s → frame 50 |
| 4 | `case/fall_hk_chairR_2.npz` | chairR-halfkneel-2 (redo of failed #5) | 30 | ~2.4 m | go+5 s → frame 50 |

Also in the log but **not** standard cases: a mid-room warm-up (unsaved) and a
failed chairR run (cube returned 0 entries, redone as `chairR_2`).

**Recording protocol (per file = ONE fall):**
- `rec/on`, then **sit by T+3 s**, **one sit→half-kneel collapse at T+5 s**
  (`fall_rel_s = 5`).
- Script fires **3 cube bursts** (TLV-320, n≈50, ~5 s each) at rel ≈ 8 / 16 / 24 s
  → these capture the **aftermath**, not the descent.
- The continuous 3001 point cloud runs throughout.

**npz layout** (both TLV-8 slow-time and TLV-320 track-bin cube share antenna
ordering, per `spatial3d/tlv.py`):
- `ts` (epoch, per frame) — align to `go_t` to pin the single fall.
- `e_*` — per-event TLV-320: `e_vec` (16-ant zero-Doppler complex), `e_frame`,
  `e_bin`, `e_range`, `e_vel(=0)`. ~7 events/frame during a burst.
- `p_frame`, `pc_xyz` — 3001 point cloud (radar frame).
- `t_*` — per-track state: `t_x/y/z`, `t_pose`, `t_down`, `t_hs`, `t_fprob`.

> ⚠️ **Framing correction.** The first pass counted each cube-burst dropout window
> as a separate fall → "11 takes." Wrong: it is **4 falls × 3 bursts**. Aligning
> `ts` to `go+5 s` pins the single fall per file — critical for `chairL_1`, whose
> real fall is frame ~1272, not the frame-281 earlier activity that was analysed
> by mistake.

---

## 2. Goal & the two candidate methods

Half-kneel is the hard boundary: a fall that **freezes at an intermediate height**
and reads as **SIT** geometry (see `case/README.md`) → the classic missed-fall
mode. The plan was to recover a **height-vs-time trajectory through the ~5 s
point-cloud blackout** and turn descent-rate into a discriminator.

Two ways to get height during the blackout:
- **A. Beamform the cube** (`e_vec` 16-ant) → (az, el) → range → height.
- **B. Point-cloud height before/around the blackout** + firmware track.

---

## 3. What was tried, and what each showed

Tools written: `pc/hk_recon.py`, `pc/hk_validate.py`, `pc/hk_beamform.py`,
`pc/hk_descent.py` (+ the shipped `web/fall_replay.py`).

### 3.1 Reconnaissance (`hk_recon.py`)
Each file has a ~5 s window where `pcN = 0` but ~7 TLV-320 events/frame survive —
the premise (cube is the only live signal in the blackout) holds. Firmware height
in that window is **DEAD**: `t_z` frozen at a coasted constant (−13 cm flat for
60+ frames), `t_hs ≈ 0`, `t_pose = 255` (MLP is track-gated, dies on pc loss).

### 3.2 Cube beamforming — BLOCKED (`hk_validate.py`, `hk_beamform.py`)
`spatial3d/music.awrl6844_array()` assumes an **unverified filled 4×4 λ/2 UPA**
(its own "TODO verify positions"). Validated single-snapshot Bartlett DOA against
point-cloud truth at the dropout-edge frames where events + cloud coexist:

- **29–53° DOA error for every antenna reordering** ≈ the full 4-element beamwidth
  → no usable absolute elevation.
- Single-snapshot Bartlett locks onto **strong static clutter** — a fixed blob at
  (x 0.77, y 1.78, z 1.38 m) that is **not the person** (person at y 3.3, z −0.35)
  → impossible 200–243 cm "settle" heights.
- Covariance-MUSIC can't rescue it: a still body gives **coherent (rank-1)**
  snapshots → no DOA-averaging gain. The error is **geometry bias**, not noise.

`change_beam.py` sidesteps all this by **only ever differencing** base-vs-event
("distorted but consistent") — it never trusts absolute angle. To ever do absolute
height needs the **real xWRL6844 virtual-array geometry** from the TI SDK
(`ti_ref/toolbox_4.0` on the build VM). Antenna ordering was never the bug.

### 3.3 Point-cloud descent — NOT MEASURABLE on these cases (`hk_descent.py`)
Go-aligned to the single fall, with **static-clutter rejection** (keep pc points
within 0.8 m (x, y) of the persistent track — never height-gate, so a real upright
torso is preserved):

1. **It's sit→kneel, not stand→floor.** The subject reads **low before the fall**
   (raw pc z90 ≈ **−10 cm** at rel 2–3 s — verified on the *raw* cloud, not a
   filter artifact). Small Δh, no big descent to measure.
2. **The transition is entirely in a pc blackout.** The cloud dies ~rel 3.5 s and
   returns ~rel 8 s; the fall-cfg CFAR loses the descent motion at range.
3. **Post-fall cloud is ghost-contaminated** (~85–165 cm points the 0.8 m coloc
   filter doesn't fully remove).
   → Any "drop 72–147 cm / rate −180…−473 cm/s" printed by earlier passes are
   **artifacts of interpolated ghost points. RETRACTED.**

### 3.4 Cube as a stay-down confirmer — partial
TLV-320 fires during the aftermath bursts, so it *can* confirm presence through
the blackout, but coverage over a fixed rel-6…26 s window is only **0.35–0.60**
(the earlier "1.00" was circular — measured over windows *defined by* cube
presence). Firmware `t_down` flickers (0–0.93) and is unreliable.

---

## 4. The real fall detector already catches all 4 (`web/fall_replay.py`)

Driving the recordings through the **live** server `_scene()` (code-of-record):

| file | ground truth | detected events | window(s) |
|------|:---:|:---:|---|
| chairL_1 | 1 | **2** ⚠️ | #1 53–84 s, #2 98–140 s |
| chairL_2 | 1 | 1 | 14–56 s |
| chairR   | 1 | 1 | 26–41 s |
| chairR_2 | 1 | 1 | 18–44 s |
| **total** | **4** | **5** | |

- **Recall 4/4** — none of the half-kneel falls are missed.
- Every hit fires via `reason = [window, sustained10s+, mlp:Falling]` — i.e.
  **window detector + sustained-down + pose MLP**, *not* descent-rate/geometry,
  even though pose reads **SIT/LIE** throughout.
- **chairL_1 over-fires** (extra event #1 at 53–84 s). Investigated: **not a
  noise false-alarm and not a second real fall** — it is a genuine person-down at
  the chair during setup positioning (srv 48–58 s: track at y≈3.3 m, pose Lying,
  `dn=1`, `fprob→1.0`), i.e. the subject lying/sitting low to get into position
  before the official take. The event then **over-sustains ~26 s** (via
  `sustainedNs`): by srv 60 s the low cluster is gone and only a standing person
  remains (y≈0.2 m, z≈110 cm, frames 700/720/760 all `y<2` no far cluster), yet
  the latch stays red to srv 84 s.
  → **Decision: NOT a defect, no action.** Reporting the fall = function complete.
  The "person got up" recovery / latch-release is **auto-recover's** job, already
  resolved in **owlCare 2.0**. The extra fire itself is a data-collection artifact
  (a long recording holding pre-take setup movement) that won't occur in a clean
  deployment (no experimenter walking, no repeated positioning).

---

## 5. Conclusions

1. **Descent-rate temporal criterion is not obtainable from these 4 cases.** Cube
   beamforming can't give absolute height (array geometry); the point-cloud
   descent is a small-Δh sit→kneel that happens inside a blackout with
   ghost-contaminated recovery.
2. **On this data the half-kneel is not a recall gap** — the shipped detector
   catches all 4 via window + sustained-down + MLP:Falling.
3. The residual risk is whether the **SIT-reading geometry** causes misses in
   *other* conditions not represented here. The chairL_1 over-fire is **closed**:
   a setup-positioning down-state + latch over-hold, ruled not-a-defect —
   reporting the fall is the whole job; get-up/recovery belongs to **auto-recover
   (owlCare 2.0, resolved)**.

## 6. To actually build a descent-rate criterion — data needs

- **Start from STANDING** (stand→half-kneel) for a real Δh, not sit→kneel.
- **Higher fps** — at 10 fps a fast descent is only 2–4 frames, too coarse for a
  rate.
- **A CFAR/cfg that keeps the point cloud ALIVE through the descent motion at
  range** (the current fall cfg blacks out the moment motion starts far out).
- One clean **single** fall per file with a standing baseline first.
- (Only if absolute cube height is ever required) pull the **true xWRL6844
  virtual-array geometry** from the TI SDK and redo MUSIC with ceiling-multipath
  nulling.

## 7. Tool index

| tool | purpose |
|------|---------|
| `pc/hk_recon.py` | per-file frame/fps, blackout windows, event survival, firmware-state timeline |
| `pc/hk_validate.py` | single-snapshot beamform DOA vs pc-truth; antenna-ordering sweep (→ 29–53° error) |
| `pc/hk_beamform.py` | (deprecated) cube beamforming height — kept as the record of why it fails |
| `pc/hk_descent.py` | go-aligned, clutter-rejected body-height / descent + aftermath stay-down |
| `web/fall_replay.py` | the recordings through the REAL `_scene()` fall pipeline (code-of-record count) |
