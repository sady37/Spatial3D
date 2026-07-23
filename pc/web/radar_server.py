"""SERVER layer — DISPLAY ONLY. Holds a Source, runs the compute layer on the
latest window, serves the dashboard + JSON. No vitals math lives here; swap the
algorithm in radar_pipeline.py and this file is unchanged.

    python3 radar_server.py live
    python3 radar_server.py chairL_hr_val_20260713/chairL_sit_20260713_225001.npz@chairL_hr_val_20260713/watch_hr_0713.csv
    # then open http://localhost:8765
"""
from __future__ import annotations
import json
import os
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # pc/ -> falldet
import radar_pipeline as pipe
from radar_source import make_source, LiveSource
from falldet.window import FloorMap, WindowDetector
from falldet.clean import Cleaner
from falldet.extramlp import ExtraMLP
from falldet.floor_track import FloorTracker


# --- audible fall tier (verification aid): 1 chirp on SUSPECTED, rapid ALARM burst on CONFIRMED.
# Toggle with FALL_BEEP=0. Runs in a daemon thread so the sequential playback never stalls
# _scene(). macOS `afplay` (short Tink so N beeps stay countable); terminal-bell fallback over
# SSH / Linux.
_FALL_BEEP = os.environ.get("FALL_BEEP", "1") != "0"
# ⭐ DISTINCT TIMBRE per tier so they're tellable apart by EAR, not by counting beeps (user
# 2026-07-20: frequent TI triggers made the 1-beep SUSPECTED indistinguishable from the CONFIRMED
# burst when both were the same sound). SUSPECTED = light Tink click; CONFIRMED = heavy Sosumi alarm.
_BEEP_SND_CONFIRM = os.environ.get("FALL_BEEP_SND", "/System/Library/Sounds/Sosumi.aiff")  # red
_BEEP_SND_SUSPECT = os.environ.get("FALL_BEEP_SND_SUSPECT", "/System/Library/Sounds/Tink.aiff")  # orange
_BEEP_VOL = os.environ.get("FALL_BEEP_VOL", "4")   # afplay -v amplifier (>1 boosts)


def _beep(n=1, gap=0.12, overlap=False, snd=None):
    # overlap=False: sequential single chirps (SUSPECTED heads-up). overlap=True: fire the
    # next play before the previous finishes -> a fast, stacked ALARM burst (CONFIRMED fall).
    # snd: sound file (defaults to the CONFIRMED alarm) -- pass the SUSPECT click for the heads-up.
    if not _FALL_BEEP:
        return
    _snd = snd or _BEEP_SND_CONFIRM
    def _run():
        for _ in range(n):
            try:
                cmd = ["afplay", "-v", _BEEP_VOL, _snd]
                if overlap:
                    subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                else:
                    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                try:
                    sys.stdout.write("\a")
                    sys.stdout.flush()
                except Exception:
                    pass
            time.sleep(gap)
    threading.Thread(target=_run, daemon=True).start()


_beep_last_state = [""]      # last fall_state we sounded on (rising-edge detect for SUSPECTED)
# ⭐ WALL-CLOCK beep debounce (2026-07-20): the audible alarm is DECOUPLED from the fall_state /
# event-onset logic, which re-fires on cloud fragmentation (prim flickers low<->gone<->standing ->
# cloud_up flips -> onset re-arms -> the 6-beep replays = 狂响). These caps mean each tier sounds at
# most once per interval no matter how the state flickers; the dashboard/event count is untouched.
_suspect_beep_t = [0.0]; SUSPECT_BEEP_MIN_S = 12.0
_confirm_beep_t = [0.0]; CONFIRM_BEEP_MIN_S = 20.0

WIN_S = 20.0            # analysis window (user 2026-07-14, per sleepad validation 20-30s) —
                        # shorter window = faster real-time response to holds / person leaving
HERE = os.path.dirname(os.path.abspath(__file__))

# Presence PERSISTENCE: living_window (single window) false-positives ~11% on an
# elevated-noise empty room (12um clutter passes the concentration/cluster test now and
# then), which leaked 'RR=18' with nobody present. Require a MAJORITY of the last
# PERSIST_S of per-window verdicts (the validated living_present logic, applied to the
# real-time stream) — a lone flicker no longer declares a person.
from collections import deque
PERSIST_S = 25.0
PERSIST_FRAC = 0.5
_present_hist = deque()   # (t, present_bool) from each fresh compute
# Presence: RR (breathing) OPENS THE DOOR (living_window establishes a person via the RR-band
# spatial concentration, and rejects empty-room noise). Once open, the static BODY REFLECTION
# HOLDS presence for up to HOLD_MAX_S during a breath-hold (person there, not breathing) — a
# breath-hold otherwise reads 'absent' and shuts off the pause counter when it should climb.
# Reflection lost, or the hold exceeds HOLD_MAX_S -> 'left'.
HOLD_REFL_FRAC = 0.6
HOLD_MAX_S = 60.0         # body reflection holds presence 60s after breathing stops (user spec)
_refl_ema = None
_last_present_t = 0.0
_person_bin = None       # remembered range-bin of the person (for range-specific hold-over)

_src = None
_meta = None
_shutdown = None     # set in main() to the graceful stop fn (for /api/quit)
# Mounting geometry for the height (H) computation only. Fixed physical install
# (--tilt/--mount override for a different rig). NO fall/posture logic lives here —
# the scene reports H (world height of the track) and the raw track coords; fall
# detection is done elsewhere (energy-density pipeline), not in this server.
TILT = 25.0          # radar DOWN-tilt from horizontal, deg (measured install 20260716)
MOUNT = 2.0          # sensor height above floor, m
FLOOR_Z = 0.4        # m; floor band top (matches radar_pipeline FLOOR_Z) — fall = energy below this
# Elevation angular ACCURACY (single-target localization), NOT the 29° two-target resolution.
# One person's height blur is governed by accuracy (~3-6° on-axis), not the ability to separate
# two targets. Beam blur grows with range: a point at slant range R has vertical uncertainty
# ~R·sin(acc). The SCENE local floor (true zero) at range R drops by HALF a cell: -0.5·rad(acc)·R,
# so a point/marker is "below floor" (=> real Fall) only if it sinks past that. Validated on the
# fall clip: at 6° the fall/crawl segment puts 5.3% of energy below the local floor vs 1.4% while
# walking (3.8x separation); the doc's 29° RESOLUTION kills it (0.2% vs 0.5%, no separation).
# Lower (3°) = more sensitive, higher = stricter. Exposed to dashboard for the sloped zero line.
ELEV_ACC_DEG = 6.0
_cache = {"t": 0.0, "key": None, "state": None}
_lock = threading.Lock()

# ---- live falldet pipeline (server-side reference; mirrors the on-chip Phase-3 window) ----
RANGE_STEP = float(os.environ.get("RANGE_STEP", "0.085"))  # m per range bin; MUST match the flashed
                            # cfg's dR (5m/pose/pose65=0.085; the 128-samp pose65s=0.106). Set via env
                            # per cfg so the cubeQuery bin mapping stays aligned with the firmware bins.
_floor = FloorMap(cell=0.5)                                   # rolling floor map H_g(x,y)
_floor_pts = []                                              # recent world points for calibration
_window = WindowDetector(_floor, margin=0.45, sustain=3, clear=3)  # sustain in SCENE-calls (~3/s)
_cleaner = Cleaner(mlp_trig=0.5, persist=2, floor_frac_min=0.7)
# ⭐ ExtraMLP: the always-on 3001-height "is a person LYING on the floor here" classifier
# (trained logistic, pose/extramlp_train.py). It OWNS the lie-vs-stand verdict; RR (cube) is a
# separate person-vs-furniture gate applied on the fetched cube, NOT a lying feature. Falls back
# to the geometric rule if the weights file is missing.
_extramlp = ExtraMLP()
EXTRAMLP_LIE_THR = 0.5          # P(lying) above this -> the cluster is a body on the floor
# kill-switch / A-B toggle: EXTRAMLP=0 reverts to the geometric lying rule AND drops the
# lying_confirmed->red promotion, reproducing the pre-ExtraMLP behavior for baseline comparison.
_EXTRAMLP_ON = os.environ.get("EXTRAMLP", "1") != "0"
# ⛔ CUBE-FREE fall branch (user 2026-07-22): CLOSED by default. There is NO cube-free fall
# detection -- a real fall = a still body whose 3001 cloud COLLAPSES, exactly when the cloud is
# useless, so only the cube's covariance ENERGY (思路B) still sees the floor body. Declaring red
# on the sustained-down window/floor_fall leg WITHOUT a cube confirm is exactly the walk-by
# false-fire (residual floor clutter + churning track deaths). RED now REQUIRES dec["fall"] (the
# cube second-check). Set CUBEFREE_FALL=1 to restore the old sustained-down-without-cube leg (A/B).
_CUBEFREE_FALL = os.environ.get("CUBEFREE_FALL", "0") != "0"
_cleaner.require_cube = not _CUBEFREE_FALL   # closes the cleaner's cube-free (extra_strong) confirm too
# ⭐ Z40_PRESENCE (default ON, validated 2026-07-22): use the 思路B z<=40 差值/基值 vs the FIXED install
# background (empty_20260721) as the cleaner's PRESENCE signal (floor_frac), replacing the raw MUSIC
# floor energy. Furniture/static clutter subtracts out (it's IN the background) -> fixed the LIVE
# "standing behind ChairL -> false fall" FP: the chair's MUSIC ffrac=1.0 fooled presence, but z40 sees
# NO new body vs the background (5 red -> 0) while all real falls hold (222500/000000/215500/013500).
# Set Z40_PRESENCE=0 to revert to MUSIC floor_frac. CAVEAT: relies on empty_20260721 being the valid
# background -- rearranging furniture would need a fresh install background. See fall-cube-free-gate.
_Z40_PRESENCE = os.environ.get("Z40_PRESENCE", "1") != "0"
# ⭐ PRESENCE verdict thresholds (user 2026-07-22, FROZEN design [[todo-cube-fall-judgment]]):
# cube_ff is PRIMARY, z40 is the FALLBACK -- CORRECTS the A+B override where z40 wrongly won
# whenever present. cube_ff >= CUBE_FF_THR = a reliable self-contained floor-body positive
# (near/breathing body: MUSIC floor-band motion energy, calibrated bimodal good 0.55-0.92 vs
# far/still 0.00). cube_ff weak/0/uncomputable/多簇 -> fall back to z40 (差值/基值 vs the fixed
# empty baseline): z40 >= Z40_LYING_THR = a lying body vs empty (far/occluded workhorse, body
# ~28-97 >> empty ~0; `down` already excluded stand/walk upstream so 0.4 only clears empty-noise).
CUBE_FF_THR = 0.5
# ⭐ WIDE CUBE (A-stage, user 2026-07-23): the firmware buffers TBC_MAX_ENTRIES=40 entries/frame, so
# ONE query can carry up to +-19 bins (39 bins ~ 4.2 m at dR 0.107) -- we were using +-3 (7 bins,
# 0.75 m). The guard budget (cubeGuardCfg 300/300/3000) counts cube-FRAMES, not entries, so a wide
# burst costs the SAME budget as a narrow one: we were spending the whole budget to look at 0.75 m.
# WHY IT MATTERS: every cube failure this session came from committing to a target bin BEFORE any
# data existed, using the least reliable input (a fragmenting track / anchor) -- RANGE_STEP skew put
# the query 10 bins out (4.39 m body queried at 5.54 m), near-leg pollution put it 30 bins out
# (void(loc=0 qbin=41 curbin=11)), old anchors scattered to bins 57/18/5/-10. A wide burst moves the
# targeting decision to AFTER the data: pick the bin from the burst's own presence profile.
# EVIDENCE (case/CAPTURES_20260721.md ground truth, 63-bin sweeps vs the boxes-removed baseline):
# the profile peak located a seated person to 4 cm (ChairL peak 4.24 m vs truth 4.20 m) and 0.5 cm
# (ChairR 3.29 vs 3.28). Amplitude is weak for seated bodies (+1.24 / +0.36) as expected -- 差值/基值
# rests on the LYING body-floor dihedral -- but A-stage only needs the peak POSITION to be right.
# COST: 39 bins x ~72 B x 10 fps ~ 28 KB/s of the 125 KB/s DATA UART for the burst (vs ~5 KB/s), the
# axis that WEDGES the sensor (needs a power-cycle). cube_sweep.py has run half_win=19 for years of
# captures at 25 frames/shot; 60 frames at that width is NOT yet validated -- if the 320 wedges,
# relaunch with CUBE_HALF_WIN=3 to restore the old narrow behaviour instantly.
CUBE_HALF_WIN = int(os.environ.get("CUBE_HALF_WIN", "19"))   # A-stage query width (+- bins); 3 = old
CUBE_VERDICT_HW = 3          # B-stage: the verdict (RR / cube_ff / z40) runs on a 7-bin burst at the
                             # A-stage peak bin -- narrow, so it can integrate the full ~6 s cheaply.
CUBE_A_FRAMES = int(os.environ.get("CUBE_A_FRAMES", "20"))   # A-stage LOCATE burst = 2 s (presence
                             # converges in ~8 snapshots; keeping the wide burst SHORT is what caps
                             # the DATA-UART high-pressure window at 2 s instead of the RR window's 6 s
                             # -- a single wide+long burst wedged the sensor live on 2026-07-23).
_empty_prof = [None]         # cached (bins, trace-power) of the fixed empty-room install baseline
# ⭐ PERIODIC WHOLE-ROOM SWEEP (user spec 2026-07-23). Two shots cover the room (cube_sweep's
# geometry: bins 1-39 + 32-64), 2 s each, every SWEEP_PERIOD_S. Two jobs:
#   1. mask out every bin within SWEEP_MASK_R_M of a track / below-floor cloud -- a body contaminates
#      NEIGHBOURING bins through multipath, so masking only the occupied bin is not enough;
#   2. feed baseline_store: rolling power/variance for the live threshold + a永久 covariance archive
#      for future room-drawing / training.
# Budget: 2 shots x 20 frames = 40 cube-frames per sweep against cubeGuard's 300 per 3000 frames --
# at a 2 h cadence that is negligible, and it NEVER runs while a fall episode is live.
SWEEP_PERIOD_S = float(os.environ.get("SWEEP_PERIOD_S", "3600"))   # 1 h (user spec 0723):
                             # one sample per (bin, hour) per day -> the dispersion that matters is
                             # ACROSS DAYS at a fixed hour (diurnal controlled). Within-hour spread is
                             # NOT wanted: it measures 2 s measurement noise, not the drift a threshold
                             # must survive. See baseline_store.hour_stats().
SWEEP_SHOTS = ((17, 16), (47, 16))    # (center, half_win) -> bins 1-33 + 31-63, full coverage.
                             # SYMMETRIC 33-bin shots, 7 entries of headroom under TBC_MAX_ENTRIES=40.
                             # Width is NOT what makes a shot fail (measured: 39 bins came back
                             # 39/39 while a 25-bin shot returned nothing -- see the retry note in
                             # _sweep_bg), but equal-load shots make any future asymmetry legible,
                             # and running flush against the firmware ceiling leaves no slack if a
                             # burst ever emits one entry more than expected.
SWEEP_FRAMES = 20                     # 2 s per shot: plenty for a trace-power/covariance estimate
SWEEP_MASK_R_M = 1.0                  # mask +-1 m around any occupant (multipath contamination)
SWEEP_SHOT_GAP_S = 12.0               # idle gap between the 2 shots -- see the measurement note
                                      # at the sleep: queries closer than ~8 s fail in alternation
_sweep_last = [0.0]
_sweep_busy = [False]
# cfg pushed at startup when the EVM is idling at its CLI prompt (see the self-heal in main()).
# The in-repo copy is the source of truth and was verified byte-identical to the flashed-era file
# under ~/project/TI/Tiinstall on 2026-07-23. It is the 128-sample variant: dR = 0.1065 m/bin,
# which is what RANGE_STEP=0.107 matches. The 160-sample sibling gives 0.085 -- the server's old
# default, and the reason cube queries were aimed ~1.1 m past the body at 4-5 m.
AUTO_CFG = os.environ.get("AUTO_CFG") or os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "firmware", "people_tracking_6844", "chirp_configs", "sbr_3dpt_6p5m_pose_128.cfg")
_last_client_scene = [0.0]   # last time an HTTP client pulled /api/scene (self-tick gate)
Z40_LYING_THR = 0.4
# ⭐ PROVENANCE/LOCATION gate: a cube result used in the fall decision MUST be THIS active query's OWN
# answer AT THIS location, else it is a stale/foreign result reused as a false confirm (Q2 live 154000:
# an upright-time bin-35 cube reused 10 s later for a window blip -> 假红; and fall1's cube reused by a
# fall2 18 s later -> can't confirm it is the CURRENT state).
# (A) LOCATION (RANGE BINS, the cube's only resolved axis): the queried bin is within CUBE_LOC_MAX_BIN
#     of the CURRENT fall/lost below-floor location. Fixed 10-BIN gate (NOT 1m/RANGE_STEP): 1 bin ≈
#     10.8 cm (65 ps) so 10 bin ≈ 1 m; a fixed bin count avoids depending on RANGE_STEP (=0.085, skew).
# (B) PROVENANCE via QUERY-EPOCH (user 2026-07-22d — REPLACES the old resp_bin ±10 bin-match): bin
#     DISTANCE cannot establish return TIMING (a leftover burst from an earlier fall at a nearby bin
#     passes any bin tolerance). Instead every ISSUED query bumps `_cube_query_epoch`; the landed
#     result is stamped with the epoch it answered; the decision accepts it ONLY if its stamped epoch
#     == the current epoch. Issuing a NEW query therefore INSTANTLY 作废s the prior held result (its
#     epoch is now stale) -> a fall2 that issues its own query can never be confirmed by fall1's cube.
# Fail (A) OR (B) -> discard -> 作废 (no return) -> the 30 s retry re-queries.
CUBE_LOC_MAX_BIN = 10
_cube_busy = [False]         # a request_cube fetch is in flight (1-elem list = mutable flag)
_last_query_t = [0.0]        # last cubeQuery wall time (rate-limit: 1 per fall episode + refresh)
QUERY_REFRESH_S = 12.0       # DEPRECATED (TODO#3 replaced it with the global CUBE_RETRY_S rate
                             # limit). Kept only as documentation of the old 50%-duty reasoning:
                             # a 6 s burst back-to-back at ~60% duty accumulates and WEDGES the
                             # DATA UART; the fix is a bigger idle gap (CUBE_RETRY_S, now 30 s) UNDER
                             # the firmware cubeGuard 10% duty cap that hard-enforces the real limit.
STALE_GATE_S = 3.0           # NEVER cubeQuery when the scene is this stale: the sensor is
                             # wedged/stalled, so 320 bursts hit a DEAD firmware -- useless,
                             # and they can keep it from recovering. Gate every probe on this.
# ⭐ ALARM-DONE MODEL (user 2026-07-22e — SUPERSEDES the 0722/0722c unlimited-cadence design): the cube's
# JOB is to CONFIRM the fall (raise the alarm) + refresh vitals a couple of times -- NOT to poll forever.
# Once the alarm has fired, THIS alarm is DONE. So a fall episode fires at most MAX_CUBE_BURSTS cube
# queries: query1 @ trigger+18 s (confirm -> red), query2 @ +CUBE_RETRY_S (refresh RR), query3 @
# +CUBE_RETRY_S (refresh RR), then STOP querying. The red HOLDS on the confirmation (no more cube needed);
# it is CLEARED by RECOVERY (person got up: cloud_up sustained) -- see the recovery block in _scene.
# WHY the hard cap is back (it was removed 0722, re-added here): unlimited "keep querying until cloud_up
# / 2x-empty" can WEDGE (never-ending 320) and never terminate if recovery-detection misses -> stuck red.
# Bounding to 3 makes total cube = 3 x 6 s = 18 s/fall (<< firmware budget) -> can't wedge, and the alarm
# is self-terminating (fire, hold, clear-on-recovery). The OLD 3-cap starved fall2 because the episode
# only reset on 5 s QUIET (CUBE_RESET_S); the FIX is that RECOVERY (cloud_up) now ALSO resets the query
# budget (_cube_episode -> 0) -> a genuinely NEW fall (person got up, then fell again) re-arms its own
# fresh 3 queries. Two falls with NO recovery between = the SAME fall (alarm already fired) -> 3 is enough.
MAX_CUBE_BURSTS = 3          # ⭐ hard cap: cube queries PER fall episode (confirm + 2 vitals refreshes).
                             # Reset to 0 on RECOVERY (cloud_up) or CUBE_RESET_S quiet -> next fall re-arms.
CUBE_RETRY_S = float(os.environ.get("CUBE_RETRY_S", "60.0"))  # s between the (<=3) cube bursts of a fall
                             # (query1 @ +18 s, then +60 s, +60 s); spreads the 3 confirmations over ~2 min
_last_cube_burst_t = [0.0]   # wall time of the LAST cube burst from EITHER probe (retry timer)
_cube_query_epoch = [0]      # ⭐ (B) PROVENANCE (user 0722d): +1 on every ISSUED query; a held result is
                             # accepted ONLY if its stamped epoch == this value -> issuing a new query
                             # instantly 作废s the prior result (no cross-fall cube reuse). See gate above.
FALL_FFRAC_MIN = 0.15        # sustained-down -> red Fall ONLY if the cloud is really below the
                             # floor line. A ~0.45 m furniture cluster (floor_frac~0.02) must
                             # NOT latch a permanent fall (that was the 22-minute false latch).
_cube_episode = [0]          # cube bursts fired in the CURRENT fall episode -> HARD-capped at
                             # MAX_CUBE_BURSTS. UNIFIED across down-probe + lost-probe. NOT per
                             # floor-track id (those CHURN -> a per-id cap leaked a fresh budget each
                             # new id -> re-wedge). Reset on recovery / CUBE_RESET_S quiet.
_cube_last_active = [0.0]    # last wall time a fall was active (down / floor-fall / latched)
_quiet_reset_done = [False]  # one-shot: the CUBE_RESET_S quiet reset has already run this quiet period
CUBE_RESET_S = 5.0           # after this long with NO active fall (person up & gone), the cube
                             # episode budget resets -> the next distinct fall gets its own bursts
# ⭐ 3001-FIRST tiering emulated by DELAY (user 2026-07-18b): the deployed product bursts 18 s of
# 3001 (cheap micro-motion/living confirm) BEFORE the bandwidth-heavy cube. In the dev prototype
# 3001 streams continuously, so we get the SAME data ordering for free by just holding the cube
# query CUBE_DELAY_S after the episode's trigger: 0-18 s the living gate = the 3001 below-floor
# cloud (floor_fall, inherently micro-motion-keyed) + sustained-down -> red WITHOUT cube; at 18 s
# the cube adds RR/floor-energy (tier 2) and gives the server ExtraMLP its cube features.
# ⭐ TIER-1 6 s FREE FILTER (user 2026-07-22f): "许多是误报" -- most TI/window alarms are TRANSIENT misfires.
# COST LADDER, cheapest first: (1) the FREE firmware window-Z/MLP `down` must PERSIST FALL_PERSIST_S before
# anything expensive engages; a misfire that clears < 6 s costs nothing. (2) THEN the 3001 tier (CUBE_DELAY_S
# of below-floor cloud / ExtraMLP lying). (3) THEN the cube (bandwidth-heavy, ≤3). The 3001 episode clock
# (`_cube_episode_t0`) only starts AFTER the 6 s survives, so 3001 decisions + cube are all gated behind it.
FALL_PERSIST_S = 6.0         # tier-1: `down` must be sustained this long before the 3001/cube tiers engage
CUBE_DELAY_S = 18.0          # tier-2: hold the FIRST cubeQuery this long after the 6 s-survived trigger (3001 first)
_cube_episode_t0 = [0.0]     # wall time the current cube episode's trigger fired (0 = no episode)
_real_since = [0.0]          # last time the real-person gate was instantaneously true
REAL_GRACE_S = 2.0           # hold real-person through brief point-count dips (see below)
_fall_latch_until = [0.0]    # a confirmed red Fall LATCHES the display until this wall time
# ⭐ RED STATE MACHINE via CUBE VERDICTS (user 2026-07-22g): the <=3 cube queries of a round DECIDE red.
#   升红 (raise) = ONE Y query (lying + isPerson Y/not-sure).
#   撤红 (clear) = TWO CONSECUTIVE N queries.
#   作废 (None)  = counts as NEITHER (skipped -- does not raise, count, or reset).
#   a Y RESETS the negative run (阴性清零); quota (3 queries) exhausted w/o 2 N -> red HOLDS.
# `_cube_confirmed_episode` is the persistent RED hold (raised on Y, held across the 60 s query gaps and
# through 作废/single-N, cleared by 2 consecutive N, by 中途-up RECOVERY (below), OR by the round ending on
# down-clear). The cube's own N verdict AND the get-up recovery are both clears.
_cube_confirmed_episode = [False]  # persistent RED hold (cube Y raised it; 2N / recovery / down-clear clears it)
_cube_neg_run = [0]          # consecutive cube-N run; None(作废) neither counts nor resets; Y resets to 0
_cube_eval_t = [0.0]         # _cube_result["t"] last counted (dedup: each query's verdict counts once)
CUBE_CANCEL_NEG = 2          # 撤红: this many consecutive cube N clears the red hold
FALL_HOLD_S = 30.0           # red latch: bridges brief `down`/dec flicker on top of the confirmed hold
# ⭐ 中途-UP RECOVERY (user 2026-07-22h): the person got UP mid-round -> 撤警 + FULL CLEAR -> standby (round 2
# re-triggers fresh). TWO legs, either fires:
#  LEG 1 cloud_up (got up, walked or not): the WHOLE-CLOUD median world height (mount+z*cos-y*sin) > RECOVER_ZMED,
#        sustained RECOVER_S, AND real_inst. NOT any box / track z -- the whole-cloud median sits in a wide gap
#        (lying -0.2..-0.5 vs standing/sitting +0.5..+0.9), so 0.4 splits it cleanly; per-box metrics thrash as
#        the body fragments but the median stays ~+0.7. 2 s guards single-frame spikes. Covers get-up-in-place /
#        sit-on-bed / lean-on-table (mass rises but doesn't walk far).
#  LEG 2 walk-away 3-gate (got up AND walked, before cloud_up's 2 s accumulates -- fast exit): per-track
#        candidate AND-chain of the THREE positive physical gates -- (1) origin, (2) displacement, (3) speed
#        limit. Fires as soon as all 3 hold (no extra sustain -- the walk IS the time). See the gate list at
#        the recovery block.
#        ⛔ the old VETO gates are GONE (user 2026-07-23): track-Z upright (4) vetoes a person who stands up
#        while their track Z is still frozen/low from the fall, and the TI-silent veto (5, w_down /
#        falling_prob) re-uses the very evidence the walk is supposed to OVERRIDE -- a stale `down` would
#        block the recovery forever. Displacement under a speed cap is unfakeable on its own: coasting drift
#        can't reach 1.5 m, and a fragment re-attach shows up as a teleport step, which gate (3) disqualifies.
RECOVER_ZMED = 0.4           # LEG1: whole-cloud median world height above this = mass is UP
RECOVER_S = 2.0              # LEG1: sustain the cloud_up recovery this long (single-frame spike guard)
RECOVER_ORIGIN_M = 0.8       # LEG2 (1): candidate track must first appear within this of the fall spot
RECOVER_DISP_M = 1.5         # LEG2 (2): straight-line displacement from origin to count as walked-away
RECOVER_SPEED_MAX = 1.2      # LEG2 (3): per-step speed cap (m/s); a faster step = teleport
RECOVER_STEP_NOISE = 0.3     # LEG2 (3): only flag a teleport when the step also exceeds this (position noise)
# ⛔ ground_clear (raw below-floor point count near the anchor) DELETED (user 2026-07-22j): DEADLOCKED by
# static furniture -- a table/chair leg permanently sits below the floor line, so the count never empties
# -> recovery impossible, red STUCK forever. Furniture immunity comes from the cube z40 (差值/基值 vs the
# empty-room baseline), not a raw point count. "Is the person UP" -> energy-centroid height (TODO), not points.
_recover_since = [0.0]       # LEG1: wall time cloud_up has been continuously true
_recover_cand = {}           # LEG2: tid -> {ox, oy, lx, ly, lt} candidate origin + last pos/time
_last_low_xy = [None]        # radar-frame (x, y) of the most recent below-floor cloud mass
_down_since = [0.0]          # wall time the current sustained-down episode started (0 = none)
_lying_since = [0.0]         # wall time the ExtraMLP lying_state has been continuously true
_lying_last = [0.0]          # last time lying_state was true (bridge brief flicker gaps)
LYING_SUSTAIN_S = 3.0        # lying_state (cloud-height) held this long (real body) -> red. Sustain
                             # filters the start-transient + dynamic-transition flicker seen on falls.
LYING_GAP_S = 2.0            # bridge a lying-flag dropout this long before resetting the sustain
_down_last = [0.0]           # last wall time `down` was true (to bridge brief flicker gaps)
FALL_SUSTAIN_S = 10.0        # sustained window-down this long (real person, can't get up) ->
                             # red Fall EVEN without cube RR. Catches a kid / weak-breathing
                             # body the cube-RR second-check can't lock onto. The cube still
                             # gets its ~6 s first; this is the fallback for real sustained down.
DOWN_GAP_S = 2.5             # `down` may drop out this long without resetting the sustain timer
# ---- ⭐ Cardiac / collapse-suspect flag ------------------------------------------------
# The MOST critical output. A fallen body that is IMMOBILE with NO breathing (RR) is a
# chest/cardiac collapse -- the ABSENCE of RR is the SIGNAL, not a reason to downgrade. The
# cube-RR gate ("no living body on the floor -> not red") is BACKWARDS for a heart/chest
# emergency: weak/absent breathing IS the emergency. So a sustained red Fall whose geometry is
# fallen (floor_fall OR lying/folded OR a below-floor mass) and where breathing was NEVER
# confirmed on this episode ESCALATES to collapse-suspect -- it must never clear on "no RR".
# fall_013500 #5 (chest-blockage half-kneel, rr=None throughout) is the gold positive case; a
# fall where the cube DID lock RR (e.g. 222500 #1) is a breathing fall, NOT collapse-suspect.
COLLAPSE_SUSTAIN_S = 12.0    # immobile-down this long with no living vital sign -> collapse-suspect
_fall_had_rr = [False]       # any confirmed cube RR seen during the CURRENT fall episode
_fall_living = [False]       # ⭐ living body confirmed this episode by RR *OR* micro-motion. This
                             # is the person-vs-object AND the "not a cardiac collapse" signal: a
                             # breathing person whose RR the estimator couldn't LOCK still shows
                             # chest micro-motion, so it must NOT escalate to 💔 (the 4 rehearsals).
_fall_measured = [False]     # cube actually RETURNED usable data this episode (vs never measured):
                             # separates measured-but-silent (strong apnea) from unassessed (weak).
_collapse_since = [0.0]      # wall time collapse-suspect first latched this episode (0 = none)
# ---- Fall-ONSET event counter (distinct events; re-segments latch-merged falls) --------
# The 30 s display latch MERGES two falls <30 s apart into ONE fall_state episode, so a person
# who falls, is helped up, then falls again reads as a single event -- that is why 222500 (3
# real falls) and 231000 (3) only ever counted 2. We count DISTINCT onsets from the PRE-latch
# red trigger: +1 the first red frame of an episode, then re-arm only after the raw `down`
# signal has been clear FALL_EVENT_GAP_S (the person got back up between falls). Latch-blind,
# so it separates the merged falls while the display stays red the whole time.
FALL_EVENT_GAP_S = 4.0       # raw `down` must be clear this long before a new onset can count
_fall_event_n = [0]          # distinct fall events seen this run (monotonic)
_fall_onset_armed = [True]   # ready to count the next onset (re-armed when down clears >= gap)
_down_clear_since = [0.0]    # wall time raw `down` has been continuously clear (0 = down now)
# ---- Track-INDEPENDENT floor-fall leg -------------------------------------------------
# window/real/cube all hang on `prim` (a GTRACK box or a FloorTracker person id). A spread
# lying body FRAGMENTS into churning orphan bits, so FloorTracker can't hold its identity
# (id churns, person stays False) and prim=None -> the whole pipeline goes dark on a clean
# floor fall (231000-A @2.5m: 500 pts, 100% below the floor line, 60 s -> MISSED). This leg
# reads the AGGREGATE below-floor cloud directly, armed by a GTRACK death nearby (a person
# went DOWN here, not walked past), and stays sticky while the below-floor blob persists --
# so it also holds far falls (222500-mid / 231500-#2 @4.5m) through the cloud collapse that
# drops the n>=12 real-person gate. Furniture never arms it (no one ever fell there).
# ⭐ ExtraMLP STATE classifier (2026-07-19, POC-validated 94% on 21 near/mid/far posture samples):
# "is there a person LYING on the floor here" from the 3001 cloud HEIGHT of a box -- the box's z1
# (95th-pct world-z = robust "top of body", == the validated `hi2`) below LYING_TOP_Z AND floor_frac
# high. This RESOLVES the far-SIT trap (track wz reads +0.09 = looks lying via elevation bias, but
# the 3001 z1 stays ~1.7 = upright) -> use the CLOUD height, NOT track-Z, NOT RR (RR is flat across
# lie/upright + dies on prone/perpendicular). Furniture-robust because 3001 is the micro-motion cloud.
# ⭐ AGGREGATE (per-CLUSTER, not per-box) + RANGE-AWARE (2026-07-19): the box-based version was
# structurally BLIND to far falls -- a far lying body GTRACK drops forms NO box (231000: boxes=[]
# for 47 s of a confirmed red), so lying_state=0 (all 92 type-2 conflicts). But it DOES form a
# CLUSTER (connected component), which carries floor_frac + a top-of-body z90. Distance matters:
# far bodies collapse to FEWER points (231500 n=9) -> range-aware min-n; and the elevation bias
# can lift a far lying body's apparent top -> loosen the top-z threshold with range.
LYING_FFRAC = 0.7           # cluster fraction below the floor band
LYING_TOP_Z = 0.4           # cluster top-of-body (z90, 90th-pct world-z) below this = whole body on
                            # the floor (a far SIT keeps z90~1.7 -> rejected; a far LIE z90<0 -> kept)
LYING_TOP_RANGE_K = 0.12    # loosen LYING_TOP_Z by this per metre of range beyond 2 m (elevation bias)
LYING_MIN_N = 6             # min cluster points (range-robust; a far lie collapses to <10 pts).
                            # floor_frac + z90 keep it precise at this low count.
FALL_LEG_MIN_PTS = 12        # min below-floor points to call it a body (rejects standing feet)
FALL_CLUSTER_MIN_N = 8       # per-cluster path: a floor-DOMINATED cluster this small still counts
                             # (a FAR faller collapses to <12 pts but is ~100% below floor). The
                             # strict floor_frac + med_wz gates below keep noise out at low n.
FALL_LEG_FRAC = 0.8          # the local region must be >=this fraction below the floor band. A
                             # LYING body is ~0.9-1.0; a SITTER's torso is above (frac ~0.4-0.6)
                             # so 0.8 rejects sitting/standing-passing-through, accepts lying.
FALL_LEG_ZMED = 0.15         # AND the local region's MEDIAN world height must be below this: the
                             # body mass is truly on the floor. FLOOR_Z=0.4 alone is knee-height
                             # -- a STANDING/walking person at range has a big below-0.4 leg cloud
                             # (231000 100-139s: z_med +0.2..+0.9 yet below_n 18-87) and dragged
                             # the armed region across the room. A lying body's z_med is -0.2..-0.5.
FALL_REGION_M = 1.6          # region radius (a lying body spreads ~1.5 m; arm + sustain gate)
FALL_UPRIGHT_M = 0.4         # a LIVE GTRACK track in the region whose world height is above this
                             # = a person standing / getting up here, NOT a fallen body -> VETO
                             # the leg. Kills the "got up from a prior lie" residual-cloud false
                             # trigger (231000 open: residual floor pts + churning track deaths
                             # armed floor_fall for 7 s -> hit sustained -> 30 s false red latch).
FALL_DEATH_S = 8.0           # a GTRACK track that died this recently near the blob = fell here
FALL_EXIT_GRACE_S = 3.0      # the below-floor blob may vanish this long before the leg disarms
_gtrack_prev = {}            # tid -> (x, y) last frame, for GTRACK-death detection
_fall_deaths = []            # [(t, x, y)] recent GTRACK deaths (a person may have gone down)
_fall_region = {"since": 0.0, "last": 0.0, "x": 0.0, "y": 0.0}  # sticky armed fall region
_fall_anchor = [None]        # world GROUND range (wy) of the selected fallen cluster -> cube target
                             # (per-cluster selection; None -> fall back to aggregate _fall_range_bin)
_clusters_now = [[]]         # THIS frame's clusters (set after the cluster loop) -> _cube_target_bin uses
                             # the CURRENT-frame floor-dominated cluster, not a stored/stale anchor (0722j)
_fall_trigger_anchor = [None]  # ⭐ (x, y) of the FALL/LOST track the firmware flagged (user 2026-07-22k):
                             # y = world GROUND range -> the cube queries WHERE TI localized the faller, NOT
                             # the cloud median (near legs pollute it -> bin12 vs the real 4.7m). ALSO anchors
                             # the tier-1 6 s filter: `down` must PERSIST within R<=FALL_ANCHOR_R of this spot
                             # (a wandering down = a walk-by, not a fall). Captured from the down-triggering
                             # `prim` box (w_down) or the lost track; cleared on episode reset / recovery.
FALL_ANCHOR_R = 0.5          # the 6 s down-persistence must stay within this of the fall/lost spot (m)
_fall_trigger_tid = [None]   # tid of the triggering track (identity continuity for the recovery-cancel)
_fall_persist_since = [0.0]  # wall time the 6 s arming clock started (locked at the first down/lost)
_arm_recover_since = [0.0]   # wall time the ID-backed recovery evidence has been continuous (debounce)
ARM_CANCEL_S = 1.0           # recovery evidence must persist this long to CANCEL the arming (else ignored)
_range_step_warned = [False] # one-shot: warn if RANGE_STEP disagrees with the stream's dr (grid mismatch)
# Lost-track RR probe: when GTRACK drops a still person's track (FloorTracker inherits it),
# actively cubeQuery that spot to get RR -- confirms a living body (vs furniture) and shows
# the RR for a sitting/fallen still person. WAIT 2 s first: most track losses are brief
# flickers that re-acquire, and we must not spend a ~6 s cube burst on those.
LOST_WAIT_S = 2.0            # a track must stay lost this long (not a flicker) before probing
LOST_QUERY_REFRESH_S = 12.0 # DEPRECATED (TODO#3): the lost-probe now shares the global CUBE_RETRY_S
FAR_FORCE_M = 4.5           # beyond this the 3001 cloud collapses/goes specular -> can't classify a
                            # lying person; a TI lost/fall trigger here FORCES a cubeQuery regardless
                            # of the 3001 person/veto gate (RR/micro then confirm). Breaks chicken-egg.
_lost_since = {}            # floor-track id -> wall time it became inherited (lost); cleared on re-acquire
_lost_query_t = {}          # floor-track id -> last lost-probe cubeQuery wall time
# NOTE: the z-DESCENT / windowed-2nd-highest-z signal lives ON-CHIP, not here -- the deployed
# server (ESP32-C5 link) has no continuous point cloud to compute it. Firmware Phase 2 (MLP,
# POINT-CENTROID velZ -> Falling) + Phase 3 (window leg, 2nd-highest world-z sustained-down,
# TLV 321) already emit the descent/down TRIGGER; the server only does the cube 2nd-check.


_cube_result = {"rr": None, "strength": 0.0, "t": 0.0, "floor_frac": 0.0, "bin": None,
                "cube_ff": None,                     # cube's OWN MUSIC power-weighted floor band
                "resp_bin": None,                    # RESPONSE median range bin (diagnostic only now)
                "epoch": None,                       # (B) provenance: the query-epoch this result answers
                "micro": False, "measured": False}  # latest cube 2nd-check (+micro-motion/measured)
# FloorTracker: gives a GTRACK-dropped STILL body (fallen OR just sitting motionless) a
# continuous track_id from its point cloud (inherits the lost tid; furniture rejected via
# no-history + no-RR). death_grace_s=30 because a sitting person's GTRACK track can flicker
# off for 15+ s -- a spot tracked within the last 30 s is still that person, not furniture.
# See falldet/floor_track.py.
_floor_tracker = FloorTracker(death_grace_s=30.0)


def _rr_from_cube(entries, fps=10.0):
    """Breathing RR from a fetched 320 burst, using the SAME estimator as the vitals
    path -- ONE RR module (bcg_vitals.demod_channels -> estimate_rr), not a second
    ad-hoc FFT. Per range bin, stack the 16-antenna zero-Doppler vectors into a (T, nAnt)
    slow-time cube; demod_channels coherently combines the antennas and phase-demodulates
    to a mm-displacement channel; estimate_rr picks the SQI-top bins and takes the median
    breathing-band peak. Breathing 'present' (the fall gate) = the top bins AGREE on the
    RR (low spread); a dropped object gives scattered per-bin peaks. Returns
    (rr_bpm | None, strength 0..1, micro-motion present bool, measured bool). `micro` = living
    chest micro-motion even when RR won't LOCK (person confirmed); `measured` = cube returned
    usable data (to separate measured-silence=apnea from unassessed=far/no-data)."""
    import numpy as _np
    from collections import defaultdict
    from bcg_vitals import demod_channels, estimate_rr
    if not entries:
        return None, 0.0, False, False            # cube returned nothing -> unassessed
    byb = defaultdict(list)
    for e in entries:
        byb[int(e.range_bin)].append(_np.asarray(e.vec, complex))   # 16-ant vec per frame
    bins = [b for b, s in byb.items() if len(s) >= 12]
    if not bins:
        return None, 0.0, False, False            # too few frames per bin -> unassessed
    T = min(len(byb[b]) for b in bins)                  # align lengths across bins
    C = [_np.stack(byb[b][:T]) for b in bins]           # C[i] = (T, nAnt) per bin
    chans = demod_channels(C, bins)                     # (nbin, T) mm displacement
    # interp=True: parabolic sub-bin peak so RR isn't quantized to the window's FFT grid
    # (a 6 s burst -> 10 rpm bins made RR look stuck at 10/20). A longer burst still helps
    # SNR + resolution; interp removes the quantization at any length.
    rr, _f0, spread, per_bin = estimate_rr(chans, fps, interp=True)
    if rr is None or not per_bin:
        # cube returned usable bins but the estimator found no rhythm at all -> measured, silent
        return None, 0.0, False, True
    strength = max(0.0, 1.0 - spread / 12.0)            # bins agree (low spread) -> confident
    # ⭐ MICRO-MOTION / living-body confirm -- SOFTER than the RR lock. A living body's chest has
    # breathing-band energy even when the RR estimator can't AGREE across bins (few bins / short
    # burst / harmonic). Proven on the 4 breathing rehearsals the strict RR gate missed: they
    # carry band-frac 0.11-0.28 and a plausible per-bin rhythm (spread<15) though strength<0.2.
    # A STATIC object (furniture) has no periodic chest energy -> band-frac ~noise floor. So this
    # says "it's a PERSON, not an object" without needing a confident RR. See fall-modular-pipeline.
    import math as _m
    band_fracs = []
    for ch in chans:
        c = ch - ch.mean()
        sp = _np.abs(_np.fft.rfft(c * _np.hanning(len(c)))) ** 2
        fq = _np.fft.rfftfreq(len(c), 1.0 / fps)
        tot = float(sp[fq > 0.05].sum())
        band_fracs.append(float(sp[(fq >= 0.15) & (fq <= 0.5)].sum()) / max(tot, 1e-9))
    band_frac = float(_np.median(band_fracs)) if band_fracs else 0.0
    micro = bool(band_frac > 0.10 or spread < 15.0)     # living chest micro-motion present
    return (round(rr, 1) if strength > 0.2 else None), round(strength, 2), micro, True


def _cube_floor_energy(entries, fps=10.0):
    """⭐ Tier-2 cube 'floor energy band' -- the CFAR-free MUSIC power-weighted floor fraction from
    the 320 burst itself (NOT the sparse 3001 floor_frac passed in). Reuses radar_pipeline's
    _pose_from_motion: per bin bandpass slow-time to the motion band, covariance of the residual
    (static clutter rejected) -> MUSIC DOA -> power-weighted Z -> fraction of MOTION energy in the
    floor band. This is what sees a FAR/specular lying body where the 3001 cloud collapsed to 0.
    Returns floor_frac in [0,1], or None if no motion / MUSIC failed."""
    import numpy as _np
    from collections import defaultdict
    from spatial3d.range_music import DR_M
    if not entries:
        return None
    byb = defaultdict(list)
    for e in entries:
        byb[int(e.range_bin)].append(_np.asarray(e.vec, complex))
    bins = [b for b, s in byb.items() if len(s) >= 12]
    if not bins:
        return None
    T = min(len(byb[b]) for b in bins)
    cube_win = [_np.stack(byb[b][:T]) for b in bins]          # [i] = (T, nAnt), == _pose_from_motion's
    dr = float((_meta or {}).get("source", {}).get("dr") or DR_M)
    try:
        mp = pipe._pose_from_motion(cube_win, _np.asarray(bins), dr, fps, TILT, MOUNT)
    except Exception:
        return None
    return None if not mp else float(mp.get("floor_frac", 0.0))


def _cube_presence_profile(entries):
    """⭐ A-stage: per-bin 差值/基值 presence profile from ONE wide burst.

    ratio[b] = (P_live[b] - P_empty[b]) / P_empty[b], P = trace(covariance) -- the same metric as
    occupancy_ratio.detect_occupancy, but computed from live entries instead of a saved npz. The
    RATIO (not the difference) is what makes near and far comparable (近大远小): a near reflector has
    a huge base AND a huge diff; the fraction cancels the R^4.
    Returns [(bin, ratio, P_live)] sorted by bin, or [] if the baseline is unavailable."""
    import numpy as _np
    from collections import defaultdict as _dd
    if not entries:
        return []
    byb = _dd(list)
    for e in entries:
        byb[int(e.range_bin)].append(_np.asarray(e.vec, complex))
    if _empty_prof[0] is None:
        try:
            _p = os.path.join(os.path.dirname(__file__), "..", "case", "empty_20260721.npz")
            _d = _np.load(_p, allow_pickle=True)
            _b = _d["bins"].astype(int)
            _P = _np.array([float(_np.real(_np.trace(_d["covariances"][i]))) for i in range(len(_b))])
            _empty_prof[0] = {int(b): p for b, p in zip(_b, _P) if p > 0}
        except Exception:
            _empty_prof[0] = {}
    base = _empty_prof[0]
    if not base:
        return []
    out = []
    for b in sorted(byb):
        s = _np.stack(byb[b])
        if len(s) < 4 or b not in base:
            continue
        P = float(_np.real(_np.trace((s.conj().T @ s) / len(s))))
        out.append((b, (P - base[b]) / base[b], P))
    return out


def _sweep_bg(occupied_r, tracks_xy):
    """Background: the 2-shot whole-room sweep -> baseline_store. `occupied_r` = ground ranges (m)
    of everything alive right now (tracks + below-floor cloud mass); every bin within
    SWEEP_MASK_R_M of one is masked OUT of the baseline."""
    import numpy as _np
    from collections import defaultdict as _dd
    try:
        import web.baseline_store as _bs
    except Exception:
        try:
            import baseline_store as _bs
        except Exception:
            _sweep_busy[0] = False
            return
    try:
        byb = _dd(list)
        for center, hw in SWEEP_SHOTS:
            # ⭐ RETRY ONCE ON EMPTY (measured 2026-07-23): cubeQuery fails in strict ALTERNATION --
            # every other query comes back [NO-Done] with zero entries, and the gap length does not
            # matter (observed at 0.5 s, 4 s and 12 s spacing alike; the WIDEST window, `20 19` =
            # 39 bins at the TBC_MAX_ENTRIES=40 ceiling, returned a full 39/39 on its good cycles).
            # That is a state TOGGLE, not a rate limit -- it looks like tbcQueryActive surviving a
            # completed burst, so the next query is rejected and the rejection clears it. Hence the
            # long-standing "near-half failed" in CAPTURES_20260721.md was never about the near
            # bins. A single retry lands on the good cycle. NOTE: the fall path's own cube query is
            # exposed to the same toggle -- an unlucky cycle returns nothing and the verdict 作废.
            ents = None
            for _try in range(2):
                ents = _src.request_cube(center, n_frames=SWEEP_FRAMES, half_win=hw,
                                         timeout=SWEEP_FRAMES / 10.0 + 3.0)
                if ents:
                    break
                time.sleep(1.0)
            for e in (ents or []):
                byb[int(e.range_bin)].append(_np.asarray(e.vec, complex))
            # ⭐ SHOT GAP (measured 2026-07-23): back-to-back cubeQueries fail in strict
            # ALTERNATION -- probing 5 shots 4 s apart gave ok/FAIL/ok/FAIL/ok regardless of width
            # (the WIDEST, `20 19` = 39 bins at the TBC_MAX_ENTRIES=40 ceiling, returned a full
            # 39/39). So the long-standing "near-half failed" in CAPTURES_20260721.md was never
            # about the near bins or the window size: two queries simply landed too close together.
            # 0.5 s between shots is what made this sweep's shot 1 come back [NO-Done] and store
            # only the far half (32 of 63 bins). A successful query needs the firmware idle for
            # roughly 8 s, so leave a wide margin -- the sweep is hourly, latency is free here.
            time.sleep(SWEEP_SHOT_GAP_S)
        if not byb:
            return
        cov_by_bin, pow_by_bin = {}, {}
        for b, s in byb.items():
            if len(s) < 4:
                continue
            S = _np.stack(s)
            C = (S.conj().T @ S) / len(S)
            cov_by_bin[b] = C
            pow_by_bin[b] = float(_np.real(_np.trace(C)))
        masked = {b for b in pow_by_bin
                  if any(abs(b * RANGE_STEP - r) <= SWEEP_MASK_R_M for r in occupied_r)}
        n = _bs.record_sweep(pow_by_bin, masked)
        _bs.archive_sweep(cov_by_bin, masked, tracks_xy)
        _line = (f"[fall] SWEEP bins={len(pow_by_bin)} masked={len(masked)} "
                 f"occupied_r={[round(r, 2) for r in occupied_r]} rolling_n={n}")
        print(_line, flush=True)
        try:
            with open(os.path.join(os.path.dirname(__file__), "..", "record",
                                   "fall_debug.log"), "a") as _fh:
                _fh.write(_line + "\n")
        except Exception:
            pass
    except Exception as _e:
        print(f"[fall] SWEEP failed: {type(_e).__name__}: {_e}", flush=True)
    finally:
        _sweep_busy[0] = False
        _sweep_last[0] = time.time()


def _fetch_cube_bg(range_bin, floor_frac, n_frames=60, epoch=None):
    """Background: burst 320 at range_bin, then compute RR from it (the cube second-check).
    n_frames sets the integration window: 60 (~6s, ~2 breaths) for a quick fall confirm;
    the lost-track/still-person probe uses a LONGER window (~15s) -- a still body is not
    time-limited, and longer coherent integration lifts weak breathing above the ~1um
    noise floor (SNR ~ sqrt(T)) AND sharpens RR resolution. Non-blocking for /api/scene."""
    try:
        if hasattr(_src, "request_cube"):
            # ⭐ TWO-STAGE cube (2026-07-23, after a wide 39-bin x 60-frame burst WEDGED the sensor
            # live -- frames stopped seconds after `CUBE-A ... shift=+9` and never came back). The
            # single wide+long burst was the WORST case for the DATA UART: RR needs ~6 s, so all 39
            # bins streamed for 6 s = ~28 KB/s of the 125 KB/s link held for the whole window. Split
            # the jobs by their real integration need:
            #   A) LOCATE -- wide but SHORT. Presence (covariance) converges in ~8 snapshots, so a
            #      2 s burst is plenty to pick the body's bin from the profile peak. High byte rate,
            #      but for 2 s, not 6.
            #   B) MEASURE -- NARROW and long. RR/z40 need ~6 s of coherent integration, but only at
            #      the peak bin, so a 7-bin burst = ~5 KB/s (the pre-wide load) for those 6 s.
            # Peak byte rate is unchanged but the high-pressure window drops 6 s -> 2 s, and total
            # bytes nearly halve. When CUBE_HALF_WIN <= CUBE_VERDICT_HW the stages collapse to one
            # narrow burst (the instant fallback if the wide stage is ever implicated again).
            _wide = CUBE_HALF_WIN > CUBE_VERDICT_HW
            if _wide:
                a_ents = _src.request_cube(range_bin, n_frames=CUBE_A_FRAMES, half_win=CUBE_HALF_WIN,
                                           timeout=CUBE_A_FRAMES / 10.0 + 3.0)
                verdict_bin = int(range_bin)
                prof = _cube_presence_profile(a_ents) if a_ents else []
                if prof:
                    verdict_bin = max(prof, key=lambda t: t[1])[0]
                    _top = sorted(prof, key=lambda t: -t[1])[:3]
                    _dbg_a = ("[fall] CUBE-A req=%d peak=%d (%.2fm) shift=%+d | top3 %s" % (
                        range_bin, verdict_bin, verdict_bin * RANGE_STEP, verdict_bin - range_bin,
                        " ".join("b%d:%+.2f" % (b, r) for b, r, _ in _top)))
                    print(_dbg_a, flush=True)
                    try:
                        with open(os.path.join(os.path.dirname(__file__), "..", "record",
                                               "fall_debug.log"), "a") as _fh:
                            _fh.write(_dbg_a + "\n")
                    except Exception:
                        pass
                # B) narrow long burst at the located bin for RR/z40 (the verdict integration)
                ents = _src.request_cube(verdict_bin, n_frames=n_frames, half_win=CUBE_VERDICT_HW,
                                         timeout=n_frames / 10.0 + 3.0)
            else:
                verdict_bin = int(range_bin)
                ents = _src.request_cube(range_bin, n_frames=n_frames, half_win=CUBE_VERDICT_HW,
                                         timeout=n_frames / 10.0 + 3.0)
            # ⭐ RANGE_STEP SELF-CHECK (user 2026-07-22j, 2nd bug): the firmware's true dR = range_m/range_bin
            # of the returned 320 entries. If the server's RANGE_STEP disagrees (server default 0.085 vs the
            # 128-samp pose65s cfg's 0.106), we compute the WRONG target bin -> the firmware queries a
            # different metre range (55*0.106=5.83m, into the wall). WARN once; fix by launching with
            # RANGE_STEP=<firmware dR> (proper fix: read dR from the cfg/TLV at start).
            if not _range_step_warned[0] and ents:
                _drs = [e.range_m / e.range_bin for e in ents if getattr(e, "range_bin", 0)]
                if _drs:
                    _dr_fw = sorted(_drs)[len(_drs) // 2]
                    if abs(_dr_fw - RANGE_STEP) > 0.005:
                        _range_step_warned[0] = True
                        import sys as _sys
                        _sys.stderr.write("WARN: RANGE_STEP=%.4f but firmware dR=%.4f (320 range_m/bin) -- "
                                          "target bins are WRONG. Relaunch with RANGE_STEP=%.4f\n"
                                          % (RANGE_STEP, _dr_fw, round(_dr_fw, 3)))
            # ⭐ (B) provenance: the RESPONSE's own range (median entry bin) -- checked against the
            # requested bin at decision time so a leftover/foreign burst is discarded (作废).
            _rbs = sorted(int(e.range_bin) for e in ents) if ents else []
            resp_bin = _rbs[len(_rbs) // 2] if _rbs else None
            range_bin = verdict_bin              # stamp/report the bin the verdict actually used
            rr, strength, micro, measured = _rr_from_cube(ents)
            cube_ff = _cube_floor_energy(ents)          # MUSIC power-weighted floor band (or None)
            z40 = None
            if _Z40_PRESENCE and ents:                  # 思路B presence vs the fixed background (XY-cell)
                try:
                    import numpy as _np2
                    from collections import defaultdict as _dd
                    from spatial3d.occupancy_ratio import z40_xy_present_from_cov
                    byb = _dd(list)
                    for e in ents:
                        byb[int(e.range_bin)].append(_np2.asarray(e.vec, complex))
                    lbins = sorted(b for b in byb if len(byb[b]) >= 8)
                    if lbins:
                        lcov = [((_np2.stack(byb[b]).conj().T @ _np2.stack(byb[b])) / len(byb[b]))
                                for b in lbins]
                        # ⭐ dr = RANGE_STEP (the FLASHED cube range grid; e_range confirms bin*RANGE_STEP
                        #    = range). The old _DRM (range_music, 0.0234) is a DIFFERENT, finer grid ->
                        #    z40 read bin 49 as 1.15m not 5.2m, got filtered by near/wall -> ALWAYS 0.
                        # ⭐ XY-cell comparison (user 0722: compare in x,y, not per range-bin).
                        z40 = round(z40_xy_present_from_cov(lcov, lbins, RANGE_STEP,
                                                            fall_range_m=range_bin * RANGE_STEP), 2)
                except Exception:
                    z40 = None
            _cube_result.update(rr=rr, strength=strength, t=time.time(),
                                floor_frac=round(float(floor_frac), 2), bin=int(range_bin),
                                resp_bin=resp_bin, epoch=epoch,   # (B) provenance stamp: which query this answers
                                cube_ff=(None if cube_ff is None else round(cube_ff, 2)),
                                micro=micro, measured=measured, z40=z40)
    except Exception:
        pass
    finally:
        _cube_busy[0] = False


def _cube_lying_verdict(cres):
    """⭐ PRESENCE verdict (user 2026-07-22, FROZEN): cube_ff PRIMARY / z40 FALLBACK.
    Returns True (a body is on the floor -> lying Y), False (empty/ghost -> lying N), or
    None (作废 -- no cube data to assess; the caller must NOT veto on this).
      * cube_ff >= CUBE_FF_THR -> reliable self-contained positive (near/breathing body).
      * cube_ff weak (<thr) / 0 / uncomputable / 多簇 -> fall back to z40 (差值/基值 vs empty):
        z40 >= Z40_LYING_THR -> body present, else empty/ghost.
      * both absent (0-entry cube: same covariance -> cube_ff None AND z40 None) -> 作废.
    CORRECTS the A+B z40-primary override (z40 wrongly won whenever present)."""
    ff = cres.get("cube_ff")            # None or float (MUSIC floor-band MOTION fraction)
    z40 = cres.get("z40")               # None or float (差值/基值 XY presence vs empty)
    if ff is not None and ff >= CUBE_FF_THR:
        return True                     # cube_ff PRIMARY: strong self-contained positive
    if _Z40_PRESENCE and z40 is not None:
        return z40 >= Z40_LYING_THR     # z40 FALLBACK: the far/occluded/多簇 workhorse
    if ff is not None:
        return False                    # cube_ff computed but weak AND no z40 fallback -> N
    return None                         # both absent -> 作废 / no assessment (do not veto)


def _cube_target_bin(sc, fallback=None):
    """Cube-query range bin = the DENSE below-floor cloud's median GROUND range = the fallen body.
    ⭐ user 2026-07-22: the below-floor cloud is the FULLEST, range-robust locator (差值/基值 z40 must
    read the covariance AT the body's bin). A FRAGMENTED far track's death/anchor coordinate scatters
    to 6 m / 1.9 m / even behind the radar (live 042500: 31 queries hit bin 57/18/5/-10, all empty,
    while 16417 below-floor points sat at bin 46) and must NOT drive the query. `fallback` (a
    death-coordinate bin) is used ONLY when there is no below-floor mass (cloud fully gone)."""
    import numpy as _np, math as _m
    # ⭐ TARGET SELECTION (user 2026-07-22j, live 195000): the AGGREGATE below-floor median MIXES people --
    # `below = wz < floor+0.5` also counts a NEAR standing/sitting person's LEGS/FEET, and near echoes are
    # dense (R^4), so the median is out-voted to ~1 m (bin 11) while the 4.7 m collapsed body (8-30 pts, bin
    # ~50) is ignored -> the fall was queried at empty bin 11 -> ghost/N -> never red. FIX = restore the
    # per-cluster pick to PRIMARY, but from the CURRENT frame's clusters (NOT a stored anchor -- 042500's real
    # bug was STALE anchor drift, not cluster selection): (1) a floor-DOMINATED cluster (floor_frac >= 0.7,
    # n >= FALL_CLUSTER_MIN_N) -> the one with the MOST points (a lying body ffrac~1.0; standing legs ffrac
    # ~0.1-0.2 are excluded); (2) else the aggregate below-floor median (single-body collapse, 042500 stable);
    # (3) else the caller's fallback (death coord / whole-cloud median).
    # (1) ⭐ floor-DOMINATED cluster (user 0722j): the faller's DENSE below-floor mass -- CLEAN (the ffrac
    # gate drops a near stander's low-ffrac legs) AND PRECISE (its wy_med is the breathing mass centre, where
    # z40/RR live). This is the primary target -- more precise than the trigger anchor's coarse box centre.
    _cl = [c for c in _clusters_now[0] if c.get("floor_frac", 0) >= 0.7 and c.get("n", 0) >= FALL_CLUSTER_MIN_N]
    if _cl:
        return int(round(float(max(_cl, key=lambda c: c["n"])["wy_med"]) / RANGE_STEP))
    # (2) ⭐ TRIGGER ANCHOR: WHERE TI localized the fall/lost track (user 0722k) -- the FALLBACK when the
    # cloud is too sparse/far to form a floor-dominated cluster (a far collapse). Query bin = anchor GROUND
    # range (y). This is what keeps a far body queried at the RIGHT spot when the cluster path can't.
    if _fall_trigger_anchor[0] is not None:
        return int(round(float(_fall_trigger_anchor[0][1]) / RANGE_STEP))
    pcx = sc.get("pc_xyz")
    if pcx is not None and len(pcx):
        th = _m.radians(TILT or 0.0)
        py, pz = pcx[:, 1], pcx[:, 2]
        wy = py * _m.cos(th) + pz * _m.sin(th)                # world GROUND range (matches 320)
        wz = MOUNT + pz * _m.cos(th) - py * _m.sin(th)        # world height
        below = wz < (_floor.default + 0.5)
        if int(below.sum()) >= FALL_LEG_MIN_PTS:              # (2) aggregate below-floor median
            return int(round(float(_np.median(wy[below])) / RANGE_STEP))
    return fallback                                           # (3) caller fallback


def _fall_range_bin(sc):
    """Range bin to cubeQuery = the fallen body's WORLD GROUND range wy (= py*cos(tilt) +
    pz*sin(tilt)). VERIFIED on 233000: the 320 breathing bins (29-44, median 35) match the
    low cloud's GROUND range (bin 34), NOT its SLANT range (bin 44, ~1 m too far). So 320
    fires at the ground-projected range, and we compute it straight from the cloud —
    track-INDEPENDENT and ALWAYS computable (this is what fixes the recent-320-bin bootstrap
    deadlock: a fall with no prior 320 — e.g. 235000 — now still gets a valid query range,
    so cubeQuery fires and bootstraps 320). Returns None only if there is no cloud.

    Multi-person: PREFER the per-cluster fall anchor (`_fall_anchor`, the ground range of the
    selected FALLEN cluster) over the aggregate below-floor median -- the aggregate mixes a
    seated 2nd person with the faller and lands between them (190500), so the cube query missed
    the far faller's bin. The aggregate stays as the fallback when no cluster was selected."""
    import numpy as _np, math as _m
    # ⭐ below-floor cloud FIRST (user 0722): the dense fallen-body mass is the reliable target, NOT
    # the drifted per-cluster anchor -- `_fall_anchor`/death scatter on a fragmented far track and
    # made 31 live queries miss the body. Anchor/full-median only when NO below-floor mass exists.
    tb = _cube_target_bin(sc)
    if tb is not None:
        return tb
    if _fall_anchor[0] is not None:
        return int(round(float(_fall_anchor[0]) / RANGE_STEP))
    pcx = sc.get("pc_xyz")
    if pcx is None or not len(pcx):
        return None
    th = _m.radians(TILT or 0.0)
    py, pz = pcx[:, 1], pcx[:, 2]
    wy = py * _m.cos(th) + pz * _m.sin(th)                # world GROUND range (matches 320)
    return int(round(float(_np.median(wy)) / RANGE_STEP))


def _pose_of(x0, x1, y0, y1, z0, z1, ez=None):
    """Pose from the two projections' extents of a per-track MERGED box:
    L = horizontal footprint (XY, longest side), Zv = vertical extent (XZ/YZ).
    A fallen body lies FLAT -> L long + Zv small (平铺). Ratio is scale-free (robust
    to the ~1 m track-Z drift, since it compares extents not absolute height).
    ez = energy-center world-z (point-weighted mean height of the 3001 cloud, NOT the
    drifting track-Z -> reliable). Server has no on-chip resource limit, so we add it as an
    extra factor to arbitrate the SIT/STAND boundary the ratio alone gets wrong: a standing
    body's mass sits high (~>=0.85 m), a seated one low (~<=0.55 m)."""
    L = max(x1 - x0, y1 - y0)
    Zv = z1 - z0
    if L >= 0.9 and Zv < 0.6:
        return "LIE"                   # flat + spread out = 平铺倒地
    if Zv >= 1.0:
        return "STAND"                 # clearly tall column
    asp = Zv / max(L, 0.05)
    base = "STAND" if asp > 1.5 else ("LIE" if asp < 0.7 else "SIT")
    if ez is not None and base in ("SIT", "STAND"):   # energy-center height arbitrates SIT vs STAND
        if ez >= 0.85:
            return "STAND"
        if ez <= 0.55:
            return "SIT"
    return base


def _flatness(x0, x1, y0, y1, z0, z1):
    """Fall GEOMETRY evidence in [0,1] from the XY/XZ/YZ extents (same basis as _pose_of):
    a LYING body is flat & spread (L long, Zv small) -> ~1; a STANDING column (Zv tall) -> 0.
    Scale-free vertical/horizontal aspect, robust to the ~1 m track-Z drift."""
    L = max(x1 - x0, y1 - y0)
    asp = (z1 - z0) / max(L, 0.05)                 # lying < 0.7, sitting ~1, standing > 1.5
    return round(max(0.0, min(1.0, (1.3 - asp) / 1.3)), 2)


def _fall_fuse(mlp_p, win_down, cloud_wz, below_frac, rr_ok, geom_flat, floor_fall):
    """Server-side fusion -> fall probability P in [0,1] (dashboard '跌倒概率 P 融合').
    Extends TI's on-chip MLP with the scene features the firmware can't see. Transparent
    weighted evidence (no trained model -- no labelled scene-level data yet):
      MLP P(falling) | 5-frame sustained-down window | 3001 cloud-centroid HEIGHT |
      below-floor ENERGY density | XY/XZ/YZ GEOMETRY flatness ; gated by cube RR (a breathing
      body on the floor = a real person, not a dropped object)."""
    e_h = 0.0 if cloud_wz is None else max(0.0, min(1.0, (FLOOR_Z - cloud_wz) / 0.7))
    ev = (0.22 * float(mlp_p or 0.0)            # TI MLP falling-motion prob
          + 0.20 * (1.0 if win_down else 0.0)   # sustained-down time window
          + 0.22 * e_h                          # cloud centroid below the floor line
          + 0.18 * max(0.0, min(1.0, float(below_frac) / 0.8))   # below-floor energy density
          + 0.18 * float(geom_flat))            # flat lying geometry
    if floor_fall:                              # the track-free floor leg alone is strong
        ev = max(ev, 0.6)
    person = 1.0 if rr_ok else 0.7              # RR present -> living body confirmed (boost)
    return round(max(0.0, min(1.0, ev)) * person, 2)


def _state(bin_lo, bin_hi):
    """Compute (cached ~0.5s) the current state for the given HR bin window."""
    key = (bin_lo, bin_hi)
    now = time.time()
    with _lock:
        if key == _cache["key"] and now - _cache["t"] < 0.5:
            return _cache["state"]
    win = _src.window(WIN_S)
    if win is None:
        st = {"present": None, "warming": True}
    else:
        cube_win, bins, dr, fps, t_wall, watch_hr = win
        st = pipe.analyze(cube_win, bins, dr, fps, hr_bin_lo=bin_lo, hr_bin_hi=bin_hi,
                          tilt_deg=TILT, h_mount=MOUNT)
        st["t_wall"] = round(t_wall, 2)
        st["watch_hr"] = watch_hr
        st["warming"] = False
    with _lock:
        # NOTE: a static-reflection 'breath-hold hold-over' was tried and REMOVED — the
        # background/empty-chair reflects ~the same as the still body, so it can't tell a
        # breath-HOLD from a person LEAVING (both = no breathing), and it kept RR showing
        # for 60s after the person left. Presence now strictly follows BREATHING (below).
        # Consequence: a breath-hold also reads 'no breathing' — this radar cannot separate
        # hold from departure by static reflection. Prioritize: no breathing => no RR.
        # presence PERSISTENCE: a single living_window verdict flickers to 'present' on
        # an elevated-noise empty room (leaked RR=18 with nobody there). Keep the last
        # PERSIST_S of verdicts and require a majority — a lone flicker no longer counts.
        p = st.get("present")
        if p is not None and not st.get("warming"):
            _present_hist.append((now, bool(p)))
            while _present_hist and now - _present_hist[0][0] > PERSIST_S:
                _present_hist.popleft()
            flags = [f for _, f in _present_hist]
            frac = sum(flags) / len(flags) if flags else 0.0
            st["present_frac"] = round(frac, 2)
            if len(flags) >= 5 and frac < PERSIST_FRAC and st.get("present"):
                st.update(present=False, hr=None, rr=None, hr_strength=None,
                          hr_confident=False, hr_level="none", hr_reason="empty",
                          fall=False, pose="empty")
        _cache.update(t=now, key=key, state=st)
    return st


def _scene():
    """Live People_Tracking scene: points + tracks + world height H of the track.
    H = mount + posZ*cos(tilt) - posY*sin(tilt) (TI returns posX/Y/Z; H is this
    deterministic rotation to world coords). NO posture/fall classification here."""
    if not hasattr(_src, "scene"):
        return {"live": False}
    import math, numpy as _np
    sc = _src.scene()
    # UNIFIED cube episode: reset the per-episode state once the scene has been quiet (no active
    # fall) for CUBE_RESET_S -> the next distinct fall re-confirms from scratch. (The old hard
    # burst cap is gone -- TODO#3 rate limit -- but the episode boundary still gates the confirm/
    # vitals/negative state.)
    # ⭐ EDGE-TRIGGERED (2026-07-23): this used to run EVERY frame for as long as the scene stayed
    # quiet, so `_cube_query_epoch += 1` ticked repeatedly -- including across the ~6 s a cube burst
    # is in flight. Live 014000: a textbook fall (body on the floor 55 s, cloud never collapsed, cube
    # aimed dead-on at bin 40 vs the cloud's bin 39-41, z40=72.08 vs a 0.4 threshold, RR 10.5) was
    # THROWN AWAY as 作废 with `void(loc=1 qbin=40 curbin=41, epoch=0 51vs54)` -- the answer came back
    # stamped 51 while the counter had already reached 54. Nothing physical failed; the bookkeeping
    # invalidated a correct answer. Now the reset fires ONCE per quiet period, not once per frame.
    if time.time() - _cube_last_active[0] > CUBE_RESET_S and not _quiet_reset_done[0]:
        _quiet_reset_done[0] = True
        _cube_episode[0] = 0
        _cube_episode_t0[0] = 0.0     # episode ended (round over) -> restart the 6 s/18 s clocks next round
        _last_low_xy[0] = None        # forget the fall spot
        _fall_anchor[0] = None        # forget the sticky faller-cluster anchor
        _fall_trigger_anchor[0] = None; _fall_trigger_tid[0] = None   # forget the trigger-localized faller
        _fall_persist_since[0] = 0.0; _arm_recover_since[0] = 0.0     # reset the tier-1 arming clocks
        _cube_confirmed_episode[0] = False   # round over (down cleared) -> drop the red hold
        _cube_neg_run[0] = 0
        # ⭐ never invalidate the query that is CURRENTLY being answered (2026-07-23): the epoch
        # exists to drop a HELD result across a state reset, not to void the burst still in flight.
        # (A) location still guards against a foreign/misplaced answer.
        if not _cube_busy[0]:
            _cube_query_epoch[0] += 1     # ③ invalidate any HELD cube result so it can't revive red
        _recover_cand.clear()
        _fall_had_rr[0] = False       # new physical fall -> re-assess vitals from scratch
        _fall_living[0] = False; _fall_measured[0] = False
        _collapse_since[0] = 0.0
        _dbg_reset = (f"[fall] QUIET-RESET epoch={_cube_query_epoch[0]} busy={int(_cube_busy[0])} "
                      f"(bumped={int(not _cube_busy[0])}) quiet={time.time()-_cube_last_active[0]:.1f}s")
        print(_dbg_reset, flush=True)
        try:
            with open(os.path.join(os.path.dirname(__file__), "..", "record", "fall_debug.log"), "a") as _fh:
                _fh.write(_dbg_reset + "\n")
        except Exception:
            pass
    elif time.time() - _cube_last_active[0] <= CUBE_RESET_S:
        _quiet_reset_done[0] = False      # scene active again -> re-arm the one-shot for the NEXT quiet
    pts_raw = sc.get("points")
    tgts = sc.get("targets") or []
    mount_m = (MOUNT if MOUNT is not None else 1.0)
    mount_cm = mount_m * 100.0
    th = math.radians(TILT or 0.0)          # radar DOWN-tilt, deg (0 = looking horizontal)

    def h_cm(y, z):                          # world height above floor, cm (tilt-corrected)
        return (mount_m + z * math.cos(th) - y * math.sin(th)) * 100.0

    pts = ([[round(float(p[0]), 3), round(float(p[1]), 3), round(float(p[2]), 3)]
            for p in pts_raw] if pts_raw is not None and len(pts_raw) else [])
    poses = sc.get("poses") or {}            # {tid: Pose} from firmware TLV 321
    tg = [{"tid": int(t.tid), "x": round(t.x, 3), "y": round(t.y, 3),
           "z": round(t.z, 3), "speed": round(t.speed, 2)} for t in tgts]
    for d in tg:                             # attach per-track fall legs (TLV 321)
        p = poses.get(d["tid"])
        if p is not None and p.valid:
            d["pose"] = p.label
            d["falling_prob"] = round(p.falling_prob, 3)
        else:
            d["pose"] = None
        if p is not None and p.win_valid:    # window leg (sustained down-state)
            d["down"] = bool(p.down)
            d["h_s_cm"] = p.h_s_cm
        else:
            d["down"] = None
    if tg:
        zsm = sc.get("z_smooth")
        zval = zsm if zsm is not None else tg[0]["z"]
        z_cm = h_cm(sc.get("y0") if sc.get("y0") is not None else tg[0]["y"], zval)
        src = f"tid{tg[0]['tid']}"
    elif pts:
        z_cm = float(_np.percentile([h_cm(p[1], p[2]) for p in pts], 90)); src = "pts"
    else:
        z_cm = None; src = None
    # raw diagnostics: is the TRACK Z floating (per fall-design ~1m blur), or sane?
    diag = None
    if tg:
        diag = {"raw_y": tg[0]["y"], "raw_z": tg[0]["z"]}   # radar-frame (untilted)
    if pts:
        pzs = [p[2] for p in pts]                            # point-cloud radar-frame Z
        phs = [h_cm(p[1], p[2]) for p in pts]                # per-point world height cm
        diag = (diag or {})
        diag.update(pz_min=round(min(pzs), 2), pz_max=round(max(pzs), 2),
                    ph_min=round(min(phs)), ph_max=round(max(phs)), npts=len(pts))
    # BLOCK-PERSON: voxelize the accumulated 3001 minor point cloud in the ROOM frame
    # (20 cm, SNR-weighted). world Z=height, Y=ground range, X=lateral.
    # 3001 point cloud, PER TRACK: assign points near each track, box them, tag by
    # track index (for per-track colour). Points not near ANY track (stray/background
    # discrete points) are DROPPED — not shown, not boxed.
    boxes = []; pc_pts = []; clusters = []      # per-cluster fall selection (multi-person)
    cloud_wz_med = None          # robust whole-cloud median world height (recovery / up signal)
    cloud_below_frac = 0.0       # fraction of the cloud below the floor band (energy density)
    pcx = sc.get("pc_xyz")
    # NOTE: no "and tg" here -- the block MUST run even with zero GTRACK tracks, because a
    # fallen still body is exactly when GTRACK drops every track yet the 3001 cloud persists.
    # Gating on tg was why the scene went blank after a fall (no cloud, no id near the floor).
    if pcx is not None and len(pcx):
        px, py, pz = pcx[:, 0], pcx[:, 1], pcx[:, 2]
        wz = mount_m + pz * math.cos(th) - py * math.sin(th)   # world height
        wy = py * math.cos(th) + pz * math.sin(th)             # world ground range
        from scipy.spatial import cKDTree
        from scipy.sparse import coo_matrix
        from scipy.sparse.csgraph import connected_components
        P = _np.stack([px, wy, wz], axis=1)
        # discrete-point removal: drop points whose nearest-neighbour > 5x the mean.
        if len(P) >= 4:
            nn = cKDTree(P).query(P, k=2)[0][:, 1]
            keep = nn <= 5.0 * float(nn.mean())
            px, py, wy, wz, P = px[keep], py[keep], wy[keep], wz[keep], P[keep]
        if len(wz):
            cloud_wz_med = float(_np.median(wz))       # robust up/down signal (all points)
            cloud_wz_cog = float(_np.mean(wz))         # ⭐ ENERGY CENTROID (point-weighted mean world-z;
                                                       # no per-point power so density ≈ energy). "有没有
                                                       # 倒地看能量重心>40" (user 0722j): a person's body is
                                                       # many points -> dominates; furniture legs (few points)
                                                       # can't pull it -> furniture-immune "is the mass UP".
            cloud_below_frac = float((wz < FLOOR_Z).mean())   # below-floor energy density
        # CLUSTER by connectivity: points closer than EPS chain into one cluster; a gap
        # bigger than EPS (e.g. two people ~1 m apart) splits them into separate boxes.
        EPS = 0.4
        n = len(P)
        if n >= 1:
            pr = cKDTree(P).query_pairs(EPS, output_type='ndarray')
            if len(pr):
                r = _np.concatenate([pr[:, 0], pr[:, 1]]); c = _np.concatenate([pr[:, 1], pr[:, 0]])
                _, labels = connected_components(
                    coo_matrix((_np.ones(len(r)), (r, c)), shape=(n, n)), directed=False)
            else:
                labels = _np.arange(n)
        else:
            labels = _np.zeros(0, int)
        q = lambda a: (round(float(_np.percentile(a, 5)), 2),
                       round(float(_np.percentile(a, 95)), 2))
        tx = _np.array([t["x"] for t in tg]); ty = _np.array([t["y"] for t in tg])
        bytid = {}                                        # ti -> ONE merged box per track
        orphans = []                                       # clusters near NO GTRACK track (any height)
        # `clusters` (init'd above the block) = EVERY connected-component (radar frame):
        # {cx,cy radar centroid, wy_med ground range, n, floor_frac, med_wz}
        # for multi-person per-cluster fall selection.
        for lab in _np.unique(labels):
            m = labels == lab
            if int(m.sum()) < 4:                          # drop tiny clusters (noise)
                continue
            cx = float(px[m].mean()); cy = float(py[m].mean())   # radar-frame centroid
            _cbelow = int((wz[m] < FLOOR_Z).sum()); _ctot = int(m.sum())
            _cwy = float(_np.median(wy[m])); _cff = _cbelow / max(_ctot, 1)
            _cz90 = float(_np.percentile(wz[m], 90))              # cluster top-of-body (robust) == hi2
            _cyspan = float(wy[m].max() - wy[m].min())            # ground-range extent (lie=elongated)
            # ⭐ TIER-2 ExtraMLP ON-DEMAND (user 2026-07-22f): "ExtraMLP 平时不调用" -- the trained
            # 3001-height logistic (lie-vs-stand) is EXPENSIVE-tier and runs ONLY after the tier-1
            # track_filter passed (i.e. `down` persisted FALL_PERSIST_S -> an episode is open, prev-frame
            # `_cube_episode_t0 > 0`). Idle / transient-alarm frames use the FREE geometric fallback and
            # never call p_lie. (1-frame lag on episode onset is negligible.)
            _clying_geom = bool(_cff >= LYING_FFRAC and _ctot >= LYING_MIN_N
                                and _cz90 < LYING_TOP_Z + LYING_TOP_RANGE_K * max(0.0, _cwy - 2.0))
            if _cube_episode_t0[0] > 0.0 and _extramlp.ok and _EXTRAMLP_ON:
                _cplie = _extramlp.p_lie({"hi2": _cz90, "floorfrac": _cff, "yspan": _cyspan,
                                          "n3001": _ctot, "micro": 0.0})
                _clying = bool(_ctot >= LYING_MIN_N and _cplie is not None and _cplie >= EXTRAMLP_LIE_THR)
            else:
                _cplie = None                         # ExtraMLP not called this frame -> free geom fallback
                _clying = _clying_geom
            clusters.append({"cx": cx, "cy": cy, "wy_med": _cwy, "n": _ctot,
                             "floor_frac": _cff, "med_wz": float(_np.median(wz[m])),
                             "z90": round(_cz90, 3), "lying": _clying,
                             "lying_geom": _clying_geom, "p_lie": round(_cplie or 0.0, 3)})
            d2 = (tx - cx) ** 2 + (ty - cy) ** 2
            ti = int(d2.argmin()) if len(tx) else -1
            if ti < 0 or float(d2[ti]) > 0.8 ** 2:        # cluster not near any GTRACK track
                # Orphan blob GTRACK left unassigned. Keep it (ANY height, not just low):
                # a STILL person -- sitting OR fallen -- gets dropped by GTRACK's motion-based
                # allocator, and the FloorTracker below re-attaches the person's identity
                # (inherited from the flickering track). Low-only would miss a sitting body
                # (world-z ~0.65) and make it flicker in/out of the scene.
                x0o, x1o = q(px[m]); y0o, y1o = q(wy[m]); z0o, z1o = q(wz[m])
                io = _np.where(m)[0]
                orphans.append({
                    "cx": cx, "cy": cy, "n": int(m.sum()),
                    "x0": x0o, "x1": x1o, "y0": y0o, "y1": y1o, "z0": z0o, "z1": z1o,
                    "below": int((wz[m] < FLOOR_Z).sum()),
                    "pts": [[round(float(px[i]), 2), round(float(wy[i]), 2),
                             round(float(wz[i]), 2)] for i in io[::max(1, len(io) // 60)]]})
                continue
            x0, x1 = q(px[m]); y0, y1 = q(wy[m]); z0, z1 = q(wz[m])
            below = int((wz[m] < FLOOR_Z).sum()); tot = int(m.sum())    # cloud floor-band count
            b = bytid.get(ti)
            if b is None:
                bytid[ti] = {"tid": int(tg[ti]["tid"]), "ti": ti, "x0": x0, "x1": x1,
                             "y0": y0, "y1": y1, "z0": z0, "z1": z1, "n": tot, "_below": below,
                             "_zsum": float(wz[m].sum())}   # sum world-z -> energy-center height
            else:                                         # MERGE same-track fragments: a body
                b["x0"] = min(b["x0"], x0); b["x1"] = max(b["x1"], x1)   # split by an EPS gap
                b["y0"] = min(b["y0"], y0); b["y1"] = max(b["y1"], y1)   # (torso vs legs) is
                b["z0"] = min(b["z0"], z0); b["z1"] = max(b["z1"], z1)   # rejoined -> pose sees
                b["n"] += tot; b["_below"] += below                     # the WHOLE person
                b["_zsum"] += float(wz[m].sum())
            idx = _np.where(m)[0]
            for i in idx[::max(1, len(idx) // 150)]:
                pc_pts.append([round(float(px[i]), 2), round(float(wy[i]), 2),
                               round(float(wz[i]), 2), ti])
        _clusters_now[0] = clusters                       # THIS frame's clusters -> _cube_target_bin (0722j)
        for b in bytid.values():                          # per TRACK (merged whole body)
            b["pose"] = _pose_of(b["x0"], b["x1"], b["y0"], b["y1"], b["z0"], b["z1"],
                                 ez=b.pop("_zsum") / max(b["n"], 1))   # energy-center world-z
            b["floor_frac"] = round(b.pop("_below") / max(b["n"], 1), 2)   # 3001 cloud floor-band fraction
            boxes.append(b)

        # ---- FloorTracker: a fallen body GTRACK dropped keeps a CONTINUOUS id ----------
        # The low orphan blobs above are the fallen person's cloud that GTRACK abandoned
        # (it allocates from motion; a still body reads as furniture). Give each a track_id:
        # inherited from the just-lost GTRACK tid ("track lost + floor blob here" = fell), or
        # a fresh negative id. DISPLAY is decoupled from the alert: the floor cloud is ALWAYS
        # drawn (so it never vanishes after a fall); the person-vs-furniture call (inherited
        # real tid OR cube RR) only gates whether it gets a fall BOX/alert.
        def _rr_at(cx, cy):
            if _cube_result["rr"] in (None, 0) or _cube_result.get("bin") is None:
                return False
            return abs(math.hypot(cx, cy) / RANGE_STEP - _cube_result["bin"]) <= 4
        gtracks = {int(t["tid"]): (float(t["x"]), float(t["y"])) for t in tg}
        # GTRACK-death memory for the floor-fall leg: a tid present last frame, gone now, went
        # DOWN here (a still body GTRACK drops), unless it walked off (its cloud goes with it).
        global _gtrack_prev, _fall_deaths
        _fdnow = time.time()
        for _tid, _xy in _gtrack_prev.items():
            if _tid not in gtracks:
                # ⑥ WALKED-OFF exclusion (user 0722i): a death with an UPRIGHT cloud cluster near it
                # (med_wz > 0.4, n >= 8, within 1.6 m) is a person who STOOD UP and walked off -- their
                # cloud trails the track -- NOT a body that dropped. Do NOT poison the floor-fall death
                # table with it (that was the "walked away but still query" chain). Collapsed / low /
                # no-cloud deaths enter normally (the real fall path is untouched).
                _upright_near = any((c["cx"] - _xy[0]) ** 2 + (c["cy"] - _xy[1]) ** 2 < 1.6 ** 2
                                    and c.get("med_wz", -1.0) > 0.4 and c["n"] >= 8 for c in clusters)
                if not _upright_near:
                    _fall_deaths.append((_fdnow, _xy[0], _xy[1]))
        _gtrack_prev = dict(gtracks)
        _fall_deaths = [(t, x, y) for (t, x, y) in _fall_deaths if _fdnow - t < FALL_DEATH_S]
        ftracks = _floor_tracker.update(time.time(), gtracks,
                                        [(o["cx"], o["cy"], o["n"]) for o in orphans],
                                        rr_at=_rr_at)
        ft_by_xy = {(round(t.x, 3), round(t.y, 3)): t for t in ftracks if t.age == 0}
        boxed = {b["tid"] for b in boxes}
        for o in orphans:
            ftk = ft_by_xy.get((round(o["cx"], 3), round(o["cy"], 3)))
            fid = int(ftk.id) if ftk else -999            # -999 = unassigned floor cloud
            for p in o["pts"]:                            # ALWAYS draw the floor cloud
                pc_pts.append([p[0], p[1], p[2], fid])
            if ftk and ftk.person and ftk.id not in boxed:   # person -> add a fall box/alert
                fb = {"tid": int(ftk.id), "ti": int(ftk.id), "x0": o["x0"], "x1": o["x1"],
                      "y0": o["y0"], "y1": o["y1"], "z0": o["z0"], "z1": o["z1"], "n": o["n"],
                      "floor_frac": round(o["below"] / max(o["n"], 1), 2), "floor_src": ftk.source}
                fb["pose"] = _pose_of(fb["x0"], fb["x1"], fb["y0"], fb["y1"], fb["z0"], fb["z1"])
                boxes.append(fb)
                boxed.add(ftk.id)

        # ---- lost-track RR probe: a GTRACK-dropped person (inherited) that has stayed lost
        # past the flicker window (LOST_WAIT_S) gets an active cubeQuery at its range, to
        # read RR (confirm living body + show it). GTRACK re-acquiring clears the timer, so a
        # brief flicker never triggers a burst.
        _now = time.time()
        alive_ids = set()
        for ftk in ftracks:
            alive_ids.add(ftk.id)
            # a LIVE GTRACK track near this floor-track = the person is TRACKED, not lost.
            # FloorTracker fragments a spread body into churning orphan bits that briefly
            # inherit an old tid, so a NORMALLY-tracked (even standing) person spawns phantom
            # "inherited" floor-tracks. lost-probing those floods 320 forever -- and because
            # the person breathes, the RR reset above keeps the dry-cap from ever stopping it
            # -> re-wedge. So NEVER lost-probe while GTRACK still has a track nearby.
            near_live = any((gx - ftk.x) ** 2 + (gy - ftk.y) ** 2 < 1.5 ** 2
                            for gx, gy in gtracks.values())
            if ftk.source == "gtrack" or near_live:
                _lost_since.pop(ftk.id, None)                # GTRACK has it -> not lost
            elif ftk.person and ftk.source == "inherited":
                _lost_since.setdefault(ftk.id, _now)         # mark when it became lost
                if _cube_episode_t0[0] == 0.0:               # start the 3001-first clock on this trigger
                    _cube_episode_t0[0] = _now
                # NO reset-on-RR here: a breathing body returns RR every burst, and resetting
                # the counter kept it probing forever -> 320 flood -> WEDGE. Rate-limited instead.
                # GATED: 3001-first DELAY elapsed (18 s of 3001 before the cube) AND sensor FRESH
                # (never flood a wedged firmware) AND CUBE_RETRY_S spacing AND the MAX_CUBE_BURSTS
                # alarm-done cap (shared across both probes) -- <=3 queries/fall, then the alarm is done.
                if (_now - _lost_since[ftk.id] >= LOST_WAIT_S
                        and (_now - _cube_episode_t0[0]) >= CUBE_DELAY_S
                        and not _cube_busy[0]
                        and _cube_episode[0] < MAX_CUBE_BURSTS      # alarm-done: <=3 queries/fall
                        and (_now - sc.get("t", _now)) < STALE_GATE_S
                        and (_now - _last_cube_burst_t[0]) > CUBE_RETRY_S):
                    # WHERE to query (Q: lost坐标记录了吗/查的是它吗/考虑地板能量吗):
                    # 1) PREFER the floor-band ENERGY cluster (`_fall_anchor`, max floor_frac) --
                    #    a still/fallen body IS the floor energy; the FloorTracker position
                    #    `ftk.x,ftk.y` DRIFTS onto whatever orphan cloud is densest (190500: the
                    #    seated 2nd person, bin10) and wasted every burst there.
                    # 2) else the DEATH coordinate near this lost track (where a person went DOWN,
                    #    from `_fall_deaths`) -- not the drifted floor position.
                    # 3) else fall back to the floor-track position.
                    # ⭐ below-floor cloud FIRST (user 0722): a lost far track's death/anchor coord
                    # scatters onto empty bins; the PERSISTENT below-floor mass is the real body.
                    # Death coordinate is only the fallback when the cloud is fully gone.
                    _dxy = [(dx, dy) for (dt, dx, dy) in _fall_deaths
                            if (dx - ftk.x) ** 2 + (dy - ftk.y) ** 2 < FALL_REGION_M ** 2]
                    _ax, _ay = _dxy[-1] if _dxy else (ftk.x, ftk.y)
                    # ⭐ TRIGGER ANCHOR (user 0722k): the LOST track's location (death coord / floor pos) is
                    # where the faller went down -> that is the cube target (below).
                    _fall_trigger_anchor[0] = (_ax, _ay)
                    rb = _cube_target_bin(sc, fallback=int(round(math.hypot(_ax, _ay) / RANGE_STEP)))
                    _cube_busy[0] = True
                    _lost_query_t[ftk.id] = _now
                    _last_cube_burst_t[0] = _now
                    _cube_query_epoch[0] += 1        # (B) new query -> 作废 prior result until this lands
                    # ⭐ quota (_cube_episode) is charged at EVALUATION on a VALID Y/N verdict, NOT here at
                    # launch (user 0722i, ④): a mis-targeted/empty first burst (作废) must not burn a query
                    # (222000 starvation). The launch gate below still holds < MAX_CUBE_BURSTS; the hard
                    # duty ceiling is the 60 s cadence + firmware cubeGuard.
                    # 60 frames (~6s): a single LONG cubeQuery (150/~15s) WEDGES the firmware
                    # -- the sustained 320 flood over DATA UART kills it ([NO-Done] + no frames).
                    # Long integration for a still body must come from stacking SHORT bursts
                    # into a server-side sliding buffer (option A), not one long burst.
                    threading.Thread(target=_fetch_cube_bg, args=(rb, 1.0, 60, _cube_query_epoch[0]),
                                     daemon=True).start()
        for d in [i for i in _lost_since if i not in alive_ids]:   # forget gone tracks
            _lost_since.pop(d, None); _lost_query_t.pop(d, None)

        # ⛔ FAR-RANGE FORCE REMOVED (user 2026-07-22): it fired a cubeQuery on a far death REGARDLESS
        # of the 3001 gate and with NO 18 s wait -- violating the core "3001 FILTERS first, THEN the
        # cube CONFIRMS" flow. Cube queries are expensive and easily wedge the firmware, so every query
        # must pass the 3001 filter (floor_fall / below-floor cloud / sustained-down) AND the 18 s
        # 3001-first delay. A far fall is now carried by the below-floor cloud (floor_fall leg) which
        # persists far, then the down-/lost-probe fires the cube AFTER 18 s -- no immediate far bypass.

        # ---- floor-fall leg: pick the FALLEN cluster by PER-CLUSTER floor-band ratio --------
        # Multi-person: a dense SEATED 2nd person has MORE points but a LOW floor_frac; the
        # faller's cluster is floor-DOMINATED (frac ~1.0) even far/sparse. Selecting the
        # max-floor_frac cluster -- NOT the aggregate below-floor median, which mixes two people
        # and lands between them (190500: seated (2.1,1.3) bin15 + faller (-0.2,2.5) bin29 ->
        # aggregate scattered the anchor bin 27-43 and starved the far faller) -- anchors the
        # arm + the cube query on the RIGHT body. Falls back to the aggregate when no cluster
        # qualifies (a body fragmented below the 4-pt component floor).
        below = wz < FLOOR_Z
        _cand = [c for c in clusters if c["n"] >= FALL_CLUSTER_MIN_N
                 and c["floor_frac"] >= FALL_LEG_FRAC and c["med_wz"] < FALL_LEG_ZMED]
        fall_cl = max(_cand, key=lambda c: (c["floor_frac"], c["n"])) if _cand else None
        _fall_anchor[0] = fall_cl["wy_med"] if fall_cl is not None else None   # per-frame (deep fallback only)
        if fall_cl is not None:
            bx, by = fall_cl["cx"], fall_cl["cy"]                 # the faller cluster centroid
        elif int(below.sum()) >= FALL_LEG_MIN_PTS:
            bx = float(_np.median(px[below])); by = float(_np.median(py[below]))  # aggregate fallback
        else:
            bx = by = None
        if bx is not None:
            _last_low_xy[0] = (bx, by)           # where the body is while low (30 s-cancel anchor)
            near = ((px - bx) ** 2 + (py - by) ** 2) < FALL_REGION_M ** 2
            reg_below = int((near & below).sum()); reg_tot = int(near.sum())
            reg_med_z = float(_np.median(wz[near])) if reg_tot else 1.0   # local mass height
            _reg_min = FALL_CLUSTER_MIN_N if fall_cl is not None else FALL_LEG_MIN_PTS
            # VETO: a LIVE GTRACK track in the region that is UPRIGHT (world height above the
            # fall band) = a person standing / getting up here, not a fallen body.
            veto_up = any(((t["x"] - bx) ** 2 + (t["y"] - by) ** 2 < FALL_REGION_M ** 2)
                          and (mount_m + t["z"] * math.cos(th) - t["y"] * math.sin(th))
                          > FALL_UPRIGHT_M for t in tg)
            if veto_up:
                _fall_region["since"] = 0.0        # someone is upright here -> drop it now
            elif (reg_below >= _reg_min and reg_below >= FALL_LEG_FRAC * reg_tot
                    and reg_med_z < FALL_LEG_ZMED):
                # a substantial, floor-DOMINATED blob. Arm ONLY if a person went DOWN here: a
                # GTRACK track died nearby recently (a still body drops off GTRACK). NOT on a
                # live track nearby -- a standing/sitting tracked person must never arm it, and
                # when GTRACK holds the track the normal window/real pipeline handles it; this
                # leg exists for exactly the prim=None case. `armed_here` = sticky sustain.
                near_death = any(_now - dt < FALL_DEATH_S
                                 and (dx - bx) ** 2 + (dy - by) ** 2 < FALL_REGION_M ** 2
                                 for (dt, dx, dy) in _fall_deaths)
                armed_here = (_fall_region["since"] > 0.0 and
                              (bx - _fall_region["x"]) ** 2 + (by - _fall_region["y"]) ** 2
                              < FALL_REGION_M ** 2)
                if near_death or armed_here:
                    if _fall_region["since"] == 0.0:
                        _fall_region["since"] = _now
                    _fall_region["last"] = _now
                    if near_death:                 # (re)anchor ONLY on a fresh fall here;
                        _fall_region["x"] = bx     # sticky sustain keeps the ORIGINAL anchor
                        _fall_region["y"] = by     # so the region can't WALK with a person who
                                                   # got up and moved off (the 2.5->4.6m drift).
    # ---- fall via the falldet pipeline (Module 1 window + Module 3 clean) --------------
    # Server-side reference for the on-chip Phase-3 window trigger; also a fallback until
    # the firmware window ships. MLP leg = firmware Phase 2 (absent here). Red Fall needs
    # the cube second-check (RR + floor energy); without it the best verdict is Suspected.
    global _floor_pts
    prim = next((b for b in boxes if b["ti"] == 0), None) or \
        (max(boxes, key=lambda b: b["n"]) if boxes else None)
    primary_pose = prim["pose"] if prim else None
    prim_ffrac = float(prim.get("floor_frac", 0.0)) if prim else 0.0
    # ⭐ ExtraMLP STATE classifier verdict: is ANY real in-room CLUSTER lying on the floor? Uses the
    # per-CLUSTER aggregate (robust to the far-fall no-box blind spot the per-box version had).
    lying_cluster = next((c for c in clusters if c.get("lying")
                          and abs(c["cx"]) < 3.5 and 0.0 <= c["wy_med"] < 6.5), None)  # range->6.5m (front wall 6.2m)
    lying_state = lying_cluster is not None

    # rolling floor calibration H_g(x,y) from the scene cloud (world x, wy, wz)
    if pc_pts:
        _floor_pts.extend((p[0], p[1], p[2]) for p in pc_pts)
        _floor_pts = _floor_pts[-4000:]
        if len(_floor_pts) >= 200 and len(_floor.hg) == 0:
            _floor.fit(_floor_pts)

    # Module 1: sustained max-height window on the primary track's points.
    # Prefer the FIRMWARE window leg (TLV 321, true 10 fps sustain) when it's
    # emitting; else fall back to this server-side reference (~3 scene-calls/s).
    prim_pts = [(p[0], p[1], p[2]) for p in pc_pts if prim and p[3] == prim["ti"]]
    wout = _window.update(prim_pts)                    # keep running (floor calib + fallback)
    fw = poses.get(prim["tid"]) if prim else None      # firmware legs for the primary track
    if fw is not None and fw.win_valid:
        w_down, w_hs = bool(fw.down), fw.h_s_cm / 100.0
        w_src = "fw"
    else:
        w_down, w_hs = bool(wout["down"]), wout["h_s"]
        w_src = "srv"

    now = time.time()
    # REAL-PERSON gate: reject ghost tracks (jumpy, out-of-room, few points) so a low
    # ghost / stray floor clutter never triggers a fall or spams cubeQuery. DEBOUNCED:
    # the 3001-cloud point count near a STILL/lying track oscillates a lot (measured
    # 0..36 frame-to-frame), so an instantaneous n>=12 gate flickers and keeps resetting
    # the fall trigger + `run` counter -> the red Fall only ever caught a 1-frame sliver
    # the dashboard missed. The firmware winDown is ALREADY sustained (sustain=5), so we
    # only need real-person to ARM; hold it for REAL_GRACE_S through the n dips.
    real_inst = bool(prim and prim.get("n", 0) >= 12
                     and abs((prim["x0"] + prim["x1"]) / 2.0) < 3.5
                     and prim["y1"] < 6.5)                   # range->6.5m (front wall 6.2m)
    if real_inst:
        _real_since[0] = now
    real_person = (now - _real_since[0]) < REAL_GRACE_S     # debounced
    # floor-fall leg (armed above): disarm when the below-floor blob is gone (person got up /
    # moved off). It is track-INDEPENDENT, so it triggers `down` even when prim=None (GTRACK
    # dropped a still body and FloorTracker fragmented its identity) -> catches the clean floor
    # falls the prim pipeline misses, and sustains far falls through the cloud collapse.
    if _fall_region["since"] and (now - _fall_region["last"]) > FALL_EXIT_GRACE_S:
        _fall_region["since"] = 0.0
    floor_fall = _fall_region["since"] > 0.0
    down = bool((w_down and real_person) or floor_fall)     # gated trigger (+ track-free leg)
    # sustained-down clock (bridges brief DOWN_GAP_S flicker gaps) -- computed HERE so the tier-1 6 s
    # free filter can gate the 3001/cube episode on it. A transient misfire clears < 6 s -> no episode.
    if down:
        if _down_since[0] == 0.0:
            _down_since[0] = now
        _down_last[0] = now
    elif _down_since[0] and (now - _down_last[0]) > DOWN_GAP_S:
        _down_since[0] = 0.0                       # down truly gone -> reset the sustain timer
    down_dur = (now - _down_since[0]) if _down_since[0] else 0.0
    # ⭐ TIER-1 TWO-STATE ARMING (user 0722k/4): lock the anchor + a 6 s clock on the FIRST down/lost. The
    # ONLY early exit is ID-backed RECOVERY evidence sustained ARM_CANCEL_S (an UPRIGHT + TI-silent track,
    # tid-continuous OR at the anchor R<=FALL_ANCHOR_R). EVERYTHING ELSE (down present/flicker/gone, single-
    # frame evidence) is ignored -> at 6 s the episode opens BY DEFAULT and 3001/cube resolves it. Bias =
    # never miss a collapse: an empty / far-sparse anchor still opens the episode (the cube asks the spot).
    if _cube_episode_t0[0] == 0.0:                              # ARMING (episode not yet open)
        if _fall_trigger_anchor[0] is None and down:           # FIRST down/lost -> lock anchor + start clock
            if w_down and prim is not None:
                _fall_trigger_anchor[0] = ((prim["x0"] + prim["x1"]) / 2.0, (prim["y0"] + prim["y1"]) / 2.0)
                _fall_trigger_tid[0] = prim["tid"]
            else:                                              # track-free (floor_fall) -> anchor the floor body
                _fcl = [c for c in _clusters_now[0]
                        if c.get("floor_frac", 0) >= 0.7 and c.get("n", 0) >= FALL_CLUSTER_MIN_N]
                if _fcl:
                    _bc = max(_fcl, key=lambda c: c["n"]); _fall_trigger_anchor[0] = (_bc["cx"], _bc["wy_med"])
            if _fall_trigger_anchor[0] is not None:
                _fall_persist_since[0] = now
        if _fall_trigger_anchor[0] is not None:
            _a = _fall_trigger_anchor[0]
            _ti_silent = (not w_down) and not (fw is not None and fw.valid and fw.falling_prob >= 0.5)
            _rec = _ti_silent and any(                          # ID-backed recovery: upright + silent
                (mount_m + t["z"] * math.cos(th) - t["y"] * math.sin(th)) >= FALL_UPRIGHT_M
                and ((t["x"] - _a[0]) ** 2 + (t["y"] - _a[1]) ** 2 <= FALL_ANCHOR_R ** 2
                     or t["tid"] == _fall_trigger_tid[0]) for t in tg)
            if _rec:
                if _arm_recover_since[0] == 0.0:
                    _arm_recover_since[0] = now
                if now - _arm_recover_since[0] >= ARM_CANCEL_S:  # sustained -> CANCEL -> standby (only exit)
                    _fall_trigger_anchor[0] = None; _fall_trigger_tid[0] = None
                    _fall_persist_since[0] = 0.0; _arm_recover_since[0] = 0.0
            else:
                _arm_recover_since[0] = 0.0                     # evidence flickered -> reset the debounce
            if _fall_trigger_anchor[0] is not None and (now - _fall_persist_since[0]) >= FALL_PERSIST_S:
                _cube_episode_t0[0] = now                      # 6 s default -> open episode, 3001-first起钟
    elif w_down and prim is not None:                          # episode OPEN: refresh anchor for cube target
        _cxy = ((prim["x0"] + prim["x1"]) / 2.0, (prim["y0"] + prim["y1"]) / 2.0)
        _a = _fall_trigger_anchor[0]
        if _a is None or (_cxy[0] - _a[0]) ** 2 + (_cxy[1] - _a[1]) ** 2 <= FALL_ANCHOR_R ** 2:
            _fall_trigger_anchor[0] = _cxy

    # server-triggered cube fetch: HELD CUBE_DELAY_S after the trigger (3001-first tiering) then
    # rate-limited to one burst / CUBE_RETRY_S (TODO#3: firmware 10% duty) — not every scene call.
    # Range from the cloud GROUND wy (_fall_range_bin, per-cluster fall anchor). GATED on: sensor is
    # FRESH (never flood a wedged firmware) AND the global CUBE_RETRY_S rate limit. During 0-18 s the
    # living gate is the 3001 below-floor cloud (floor_fall) + sustained-down -> red needs no cube.
    fresh = (now - sc.get("t", now)) < STALE_GATE_S
    cube_delay_ok = _cube_episode_t0[0] > 0.0 and (now - _cube_episode_t0[0]) >= CUBE_DELAY_S
    # ⭐ VETO model (user 2026-07-20): on a TI alarm, the 3001 ExtraMLP posture can VETO -- but ONLY
    # when it says WALK/STAND (primary_pose=="STAND"; _pose_of has no WALK, walking = a moving STAND)
    # AND nothing is lying. If 3001 does NOT veto -- lying, OR SIT (ambiguous), OR a collapsed/absent
    # far body, OR empty -- the cubeQuery proceeds normally. This preserves the far-lying-collapse
    # case (no STAND box -> no veto -> cube STILL queries -> RR/MUSIC can confirm).
    veto = bool((not lying_state) and primary_pose == "STAND")
    # CUBE fires BROADLY: on a TI alarm, unless 3001 vetoes a STAND. Keeps the far-lying-collapse
    # rescue (no STAND box -> cube still queries). This is SEPARATE from the suspected declaration
    # below -- querying a cube is cheap-to-be-wrong; declaring "suspect fall" to the user is not.
    cube_gate = bool(cube_delay_ok and not veto)
    # ⭐ ALARM-DONE (user 0722e): at most MAX_CUBE_BURSTS queries per fall (confirm @ +18s, then +60s,
    # +60s), then STOP -- the alarm has fired, the red HOLDS on the confirmation, and RECOVERY clears it.
    # The cap makes total cube 3x6s=18s/fall (can't wedge) and self-terminating (no query-forever).
    if (cube_gate and not _cube_busy[0] and fresh
            and _cube_episode[0] < MAX_CUBE_BURSTS
            and (now - _last_cube_burst_t[0]) > CUBE_RETRY_S):
        rb = _fall_range_bin(sc)
        if rb is not None:
            _cube_busy[0] = True
            _last_query_t[0] = now
            _last_cube_burst_t[0] = now
            _cube_query_epoch[0] += 1               # (B) new query -> 作废 prior result until this lands
            # quota charged at EVALUATION on a valid Y/N (not here) -- 作废 must not burn a query (④)
            threading.Thread(target=_fetch_cube_bg, args=(rb, prim_ffrac, 60, _cube_query_epoch[0]),
                             daemon=True).start()

    # cube second-check evidence = RR + floor energy computed FROM the fetched 320 burst
    # (self-contained; a living body on the floor breathes -> RR -> red Fall; a dropped
    # object does not -> no red). A confirmed RR is HELD 12 s (the person stays down and
    # breathing; a single 3 s burst is < 1 breath cycle, so bridge the gaps) — the ~4 s
    # re-query keeps it refreshed while down.
    # ⭐ PRESENCE verdict (FROZEN 2026-07-22): cube_ff PRIMARY / z40 FALLBACK -- the cube fire's
    # `lying` dimension. cube_ff >= 0.5 (near/breathing body's MUSIC floor-band motion) trusts itself;
    # weak/0/uncomputable/多簇 -> z40 (差值/基值 vs empty) arbitrates; both absent -> 作废 (None) -> no
    # cube confirmation this frame and NO veto (an unassessed cube must not assert OR deny a body).
    # This REPLACES the A+B z40-primary override (z40 wrongly won whenever present -- BACKWARDS).
    cube_ev = None
    cube_lying = None            # cube presence verdict: True=body / False=empty / None=作废
    _cube_void = ""              # DIAG only: why a fresh cube verdict was 作废 (loc / epoch)
    cube_living_state = None     # "Living" (rr|micro) / "?" (body but unmeasurable) / None (no fresh cube)
    if now - _cube_result["t"] < 12.0:
        cube_lying = _cube_lying_verdict(_cube_result)
        # ⭐ (A) LOCATION + (B) PROVENANCE gate (FROZEN 0722, fixed 10-bin CUBE_LOC_MAX_BIN): the held
        # cube must be THIS fall's OWN query AT THIS location, else it is a stale/foreign result reused
        # as a false confirm (Q2 live 154000: an upright-time bin-35 cube reused 10 s later for a window
        # blip -> 假红). (A) the queried bin is within 10 bins of the CURRENT fall/lost below-floor
        # location; (B) the result's stamped query-epoch == the CURRENT epoch (it is THIS active query's
        # own answer, not a leftover reused across a time gap -- bin distance can't prove return timing;
        # the epoch can). Fail either -> discard -> 作废 (cube_lying=None, no confirm, no veto) -> retry.
        if cube_lying is not None:
            _qbin = _cube_result.get("bin")
            _curbin = _cube_target_bin(sc)               # current fall/lost location (None -> no mass)
            _loc_ok = (_curbin is not None and _qbin is not None
                       and abs(_qbin - _curbin) <= CUBE_LOC_MAX_BIN)     # (A) location
            _epoch_ok = (_cube_result.get("epoch") == _cube_query_epoch[0])   # (B) provenance: this query's own
            if not (_loc_ok and _epoch_ok):
                cube_lying = None                        # not THIS active query's answer here -> 作废
                _cube_void = (f"void(loc={int(bool(_loc_ok))} qbin={_qbin} curbin={_curbin},"
                              f"epoch={int(bool(_epoch_ok))} {_cube_result.get('epoch')}vs{_cube_query_epoch[0]})")
        if cube_lying is not None:                       # a real assessment (Y or N) -> feed the cleaner
            # micro = living micro-motion (confirms a person when RR can't lock: back-to-radar/occluded)
            cube_ev = {"rr": _cube_result["rr"],
                       "floor_frac": 1.0 if cube_lying else 0.0,   # cleaner: >=0.7 = body present
                       "micro": _cube_result.get("micro")}
        # Living_state LABEL (NOT a gate; "?" must NOT escalate to 💔 -- it is unmeasurable, not apnea):
        if cube_lying:
            _alive = (_cube_result["rr"] not in (None, 0)) or bool(_cube_result.get("micro"))
            cube_living_state = "Living" if _alive else "?"
    # ⭐ RED STATE MACHINE (user 2026-07-22g): count THIS query's verdict ONCE (dedup on _cube_result["t"]).
    # Y -> 升红 + 阴性清零; N -> +1, two consecutive -> 撤红; None(作废) -> skip (neither raise nor count).
    if _cube_result["t"] > 0.0 and _cube_result["t"] != _cube_eval_t[0]:
        _cube_eval_t[0] = _cube_result["t"]
        if cube_lying is True:                       # Y -> raise red, reset the negative run
            _cube_confirmed_episode[0] = True
            _cube_neg_run[0] = 0
            _cube_episode[0] += 1                     # ④ quota charged on a VALID verdict (Y/N), not 作废
        elif cube_lying is False:                    # N -> count; 2 consecutive -> clear red
            _cube_neg_run[0] += 1
            _cube_episode[0] += 1                     # ④ quota charged on a VALID verdict (Y/N), not 作废
            if _cube_neg_run[0] >= CUBE_CANCEL_NEG:
                _cube_confirmed_episode[0] = False
                _fall_latch_until[0] = 0.0
                _fall_region["since"] = 0.0          # ⑤ 撤红 -> stop the region re-arming on residual clutter
                _fall_deaths[:] = [(dt, dx, dy) for (dt, dx, dy) in _fall_deaths
                                   if _last_low_xy[0] is None
                                   or (dx - _last_low_xy[0][0]) ** 2 + (dy - _last_low_xy[0][1]) ** 2 > 1.0]
        # None (作废) -> neither: leave the hold and the negative run untouched

    # MLP leg from the firmware (Phase 2): falling motion + pose. None until its
    # 8-frame window fills. OR-fused with the window leg inside the cleaner.
    mlp_out = ({"pose": fw.label, "falling_p": fw.falling_prob}
               if (fw is not None and fw.valid) else None)
    dec = _cleaner.decide({"down": down, "h_s": w_hs}, mlp_out, cube=cube_ev, geom=None)
    # ⭐ Suspected DECLARATION is TIGHTER than the cube gate: it needs an ACTUAL ExtraMLP lying
    # candidate (lying_state), NOT merely "not a STAND" (user 2026-07-20: web 频报 suspect). Without
    # this, empty-room floor_fall noise or a sitting/moving person that trips a TI trigger reads as
    # Suspected every 18 s. The cube still queries broadly (cube_gate) to confirm/rescue; only a real
    # ExtraMLP lying candidate (surviving the 18 s gate) goes orange + beeps.
    susp_gate = bool(cube_delay_ok and lying_state)
    fall_state = "fall" if dec["fall"] else ("suspected" if susp_gate else "none")

    # SUSTAINED-DOWN escalation: track how long the person has continuously been down
    # (bridging brief DOWN_GAP_S flicker gaps). If down that long AND a real person, call it
    # a red Fall even if the cube never found breathing -- catches a kid / weak-breathing
    # body. (down_dur / _down_since are computed ABOVE, at the tier-1 6 s gate.)
    # ffrac guard: a real fall puts the cloud BELOW the floor line (high floor_frac). A
    # ~0.45 m furniture cluster (floor_frac~0.02) reads "down" via the window leg but is NOT
    # on the floor -> must never latch a permanent fall (that was the 22-minute false latch).
    # floor_fall already IS a below-floor body, so it satisfies both the real-person and the
    # ffrac guard on its own (that is the whole point -- it carries the falls where n<12 / the
    # prim box is absent). OR it in for the sustained -> red escalation.
    sustained_fall = bool(_down_since[0] and down_dur >= FALL_SUSTAIN_S
                          and (real_person or floor_fall)
                          and (prim_ffrac >= FALL_FFRAC_MIN or floor_fall))
    # ⭐ ExtraMLP lying sustain -> STANDALONE red (user 2026-07-20: ExtraMLP is the primary verdict).
    # Per-frame lying_state flickers on DYNAMIC recordings [fragmentation], so we require it SUSTAINED
    # (LYING_SUSTAIN_S, gap-bridged) AND a real_person -- that persistence is the flicker mitigation.
    # cube RR is no longer a hard gate (user), but real_person still rejects ghosts/furniture.
    if lying_state:
        if _lying_since[0] == 0.0:
            _lying_since[0] = now
        _lying_last[0] = now
    elif _lying_since[0] and (now - _lying_last[0]) > LYING_GAP_S:
        _lying_since[0] = 0.0
    lying_confirmed = bool(_lying_since[0] and (now - _lying_since[0]) >= LYING_SUSTAIN_S)
    # ExtraMLP (Tier-1, 3001) NO LONGER reds on its own: the per-frame lying flicker re-armed the
    # onset and blew up the event count ~15-19x (A/B 2026-07-20). It only produces the orange
    # candidate above; RED requires Tier-2 cube confirmation (MUSIC floor-energy + RR) via dec["fall"],
    # or the track-free sustained-down leg below. lying_confirmed stays computed as a STATE output.
    # CUBE-FREE red CLOSED by default (see _CUBEFREE_FALL): red requires the cube confirm
    # (dec["fall"]); the sustained-down leg alone was the walk-by false-fire. A real fall's
    # collapsed cloud is rescued by the lost-probe FIRING the cube (energy confirm), not by
    # declaring red without one.
    if _CUBEFREE_FALL and sustained_fall and not dec["fall"]:
        fall_state = "fall"
        dec["reason"] = list(dec.get("reason") or []) + [f"sustained{int(down_dur)}s"]

    # ⭐ 中途-UP RECOVERY (user 2026-07-22h): got up mid-round -> 撤警 + FULL CLEAR -> standby. LEG 1 cloud_up
    # (mass risen, sustained RECOVER_S) OR LEG 2 walk-away 3-gate (got up AND walked). See the RECOVER_* block.
    leg1 = leg2 = False
    _leg2_why = ""                                                # DIAG only (which track/disp fired LEG2)
    # ⭐ IDLE GUARD (2026-07-23): only run the recovery when there is something to RECOVER FROM.
    # A walking person's leg cloud dips below the floor line -> _last_low_xy gets set -> LEG1 then
    # sees the (correctly) high whole-cloud median and "撤警" over and over with no red on: live
    # 0723 logged 9 RECOVER firings in 38 s at down=0, wz_med 0.62-0.85, cands=0. Each firing wiped
    # episode state and ticked _cube_query_epoch, which is what voids in-flight cube answers.
    _episode_live = bool(_cube_confirmed_episode[0] or now < _fall_latch_until[0] or _down_since[0])
    _llxy = _last_low_xy[0] if _episode_live else None
    if _llxy is not None:
        _lx, _ly = _llxy
        # ⛔ ground_clear DELETED (user 2026-07-22j): counting raw below-floor points near the anchor is
        # DEADLOCKED by static furniture -- a table/chair leg permanently sits below the floor line, so _gm
        # never drops to <=3 -> ground_clear never True -> recovery IMPOSSIBLE, red STUCK forever. Furniture
        # immunity must come from the cube's z40 (差值/基值 vs the empty-room baseline), NOT a raw point count.
        # LEG 1 cloud_up: whole-cloud median risen, sustained RECOVER_S, real_inst.
        cloud_up = bool(cloud_wz_med is not None and cloud_wz_med > RECOVER_ZMED and real_inst)
        _recover_since[0] = _recover_since[0] or (now if cloud_up else 0.0)
        if not cloud_up:
            _recover_since[0] = 0.0
        leg1 = cloud_up and (now - _recover_since[0] >= RECOVER_S)
        # LEG 2 walk-away 3-gate (per-track): (1) origin, (2) displacement, (3) speed limit. No vetoes.
        _alive_ids = set()
        for t in tg:
            _tid = t["tid"]; _alive_ids.add(_tid)
            _c = _recover_cand.get(_tid)
            if _c is None:                                         # gate (1) ORIGIN within 0.8 m of fall spot
                if (t["x"] - _lx) ** 2 + (t["y"] - _ly) ** 2 <= RECOVER_ORIGIN_M ** 2:
                    _recover_cand[_tid] = {"ox": t["x"], "oy": t["y"], "lx": t["x"], "ly": t["y"],
                                           "lt": now, "t0": now}
                continue
            _step = math.hypot(t["x"] - _c["lx"], t["y"] - _c["ly"]); _dt = max(now - _c["lt"], 0.05)
            if _step > RECOVER_STEP_NOISE and _step / _dt > RECOVER_SPEED_MAX:   # gate (3) teleport -> re-queue
                _recover_cand.pop(_tid, None); continue           # cancel qualification, may re-register via (1)
            _c["lx"], _c["ly"], _c["lt"] = t["x"], t["y"], now
            _disp = math.hypot(t["x"] - _c["ox"], t["y"] - _c["oy"])                 # gate (2) displacement
            if _disp >= RECOVER_DISP_M:                           # (1)(2)(3) all pass -> walked away
                leg2 = True
                _el = max(now - _c["t0"], 0.05)
                _leg2_why = (f"tid{_tid} o=({_c['ox']:.2f},{_c['oy']:.2f}) "
                             f"now=({t['x']:.2f},{t['y']:.2f}) disp={_disp:.2f} "
                             f"el={_el:.2f}s avg={_disp/_el:.2f}m/s step={_step:.2f}/{_dt:.2f}s")   # DIAG only
        for _d in [k for k in _recover_cand if k not in _alive_ids]:
            _recover_cand.pop(_d, None)
    else:
        _recover_cand.clear()
    if leg1 or leg2:                                              # 撤警 + 全清 -> 待机 (round 2 re-triggers)
        # DIAG (2026-07-23): WHICH leg 撤了红, with the numbers that decided it. A red that dies while
        # the person is still down (live 005619 / 010902: down=1, hs<0.3) must be pinnable to leg1's
        # cloud median vs leg2's walked-off displacement. Logging only -- no decision uses this.
        _dbg = (f"[fall] RECOVER leg1={int(leg1)} leg2={int(leg2)} "
                f"wz_med={cloud_wz_med if cloud_wz_med is None else round(cloud_wz_med, 2)} "
                f"real_inst={int(real_inst)} rec_s={(now - _recover_since[0]) if _recover_since[0] else 0:.1f} "
                f"down={int(down)} w_down={int(w_down)} spot=({_lx:.2f},{_ly:.2f}) "
                f"cands={len(_recover_cand)}" + (f" | LEG2 {_leg2_why}" if _leg2_why else ""))
        print(_dbg, flush=True)
        try:
            with open(os.path.join(os.path.dirname(__file__), "..", "record", "fall_debug.log"), "a") as _fh:
                _fh.write(_dbg + "\n")
        except Exception:
            pass
        _cube_confirmed_episode[0] = False; _fall_latch_until[0] = 0.0
        _cube_episode[0] = 0; _cube_episode_t0[0] = 0.0; _cube_neg_run[0] = 0
        _cube_query_epoch[0] += 1        # ③ invalidate any held cube result so it can't revive red via latch
        _fall_region["since"] = 0.0      # ⑤ stop the region re-arming on residual clutter after 撤警
        _fall_deaths[:] = [(dt, dx, dy) for (dt, dx, dy) in _fall_deaths
                           if (dx - _lx) ** 2 + (dy - _ly) ** 2 > 1.0]   # ⑤ clear deaths within 1 m of spot
        _last_low_xy[0] = None; _fall_anchor[0] = None; _recover_cand.clear()
        _fall_trigger_anchor[0] = None; _fall_trigger_tid[0] = None
        _fall_persist_since[0] = 0.0; _arm_recover_since[0] = 0.0
        _recover_since[0] = 0.0
        _fall_onset_armed[0] = True                               # round over -> ready to count round 2

    # ⭐ PERIODIC WHOLE-ROOM SWEEP trigger. INTERLOCKED: never while a cube query is in flight, never
    # while ANY fall state is live (a confirm must never lose UART or guard budget to housekeeping),
    # and never before the previous sweep finished. Runs on its own thread so /api/scene never waits.
    if (SWEEP_PERIOD_S > 0 and not _sweep_busy[0] and not _cube_busy[0]
            and not _cube_confirmed_episode[0] and now >= _fall_latch_until[0]
            and not down and not _down_since[0]
            and now - _sweep_last[0] >= SWEEP_PERIOD_S
            and hasattr(_src, "request_cube")):
        _occ_r = []                                   # ground range of every occupant right now
        for _t in tg:
            _occ_r.append(math.hypot(_t["x"], _t["y"]))
        if cloud_wz_med is not None and _clusters_now[0]:
            for _c in _clusters_now[0]:
                if _c.get("wy_med") is not None:
                    _occ_r.append(float(_c["wy_med"]))
        _sweep_busy[0] = True
        threading.Thread(target=_sweep_bg, daemon=True,
                         args=(_occ_r, [(t["x"], t["y"]) for t in tg])).start()

    # ⭐ RED = the cube-verdict hold (升红 on Y, 撤红 on 2N) OR the short bridging latch. The hold clears on 2
    # consecutive cube N, on 中途-up RECOVERY (above), OR on `down` staying clear (episode reset below).
    if dec["fall"] or (_CUBEFREE_FALL and sustained_fall):
        _fall_latch_until[0] = now + FALL_HOLD_S      # live confirmation -> (re)extend the bridging latch
    if _cube_confirmed_episode[0] or now < _fall_latch_until[0]:
        fall_state = "fall"
    # fused fall probability P (dashboard '跌倒概率 P 融合'): TI MLP + the 5 scene features.
    _mlp_p = (fw.falling_prob if (fw is not None and fw.valid) else 0.0)
    _rr_ok = bool(cube_ev and cube_ev.get("rr"))
    _geom_flat = _flatness(prim["x0"], prim["x1"], prim["y0"], prim["y1"],
                           prim["z0"], prim["z1"]) if prim else 0.0
    fall_p = _fall_fuse(_mlp_p, w_down, cloud_wz_med, cloud_below_frac,
                        _rr_ok, _geom_flat, floor_fall)
    # ---- ⭐ Living-body confirm + cardiac/collapse-suspect + fall-onset counter --------
    # episode-level VITALS memory (sticks until CUBE_RESET_S quiet resets it at the top of
    # _scene). living = RR locked OR cube micro-motion present -> a PERSON is here and ALIVE, so
    # this is NOT a cardiac collapse (breathing rehearsals carry micro-motion the RR lock misses).
    # measured = the cube actually returned usable data (to tell measured-silence from unassessed).
    _cube_fresh = now - _cube_result["t"] < 12.0
    _micro = bool(_cube_fresh and _cube_result.get("micro"))
    if _rr_ok:
        _fall_had_rr[0] = True
    if _rr_ok or _micro:
        _fall_living[0] = True
    if _cube_fresh and _cube_result.get("measured"):
        _fall_measured[0] = True
    # fallen geometry = on the floor (floor_fall), lying flat, a folded/half-kneel body (mid
    # geom_flat), OR a below-floor mass. This is what separates a collapse from a crouch.
    fallen_geom = bool(floor_fall or primary_pose == "LIE"
                       or cloud_below_frac >= 0.5 or _geom_flat >= 0.55)
    # collapse-suspect: a SUSTAINED red Fall, fallen geometry, and NO living vital sign confirmed
    # this episode -- neither RR nor micro-motion. Per the design (no-RR is acceptable, a fall is
    # still a fall) this is the escalation, NOT a downgrade: we can't confirm the person is alive.
    # Gated on `_fall_living` (not just RR) so a breathing person the RR lock missed does NOT
    # falsely read as a cardiac collapse -- that was the 4-rehearsal 💔 over-fire.
    collapse_suspect = bool(fall_state == "fall" and down_dur >= COLLAPSE_SUSTAIN_S
                            and fallen_geom and not _fall_living[0])
    if collapse_suspect and _collapse_since[0] == 0.0:
        _collapse_since[0] = now
    # confidence: STRONG only when the cube MEASURED the chest and found it silent (no RR, no
    # micro-motion -- genuine apnea signature); WEAK when the cube never returned data (far fall /
    # cloud collapse -- an unassessed fall we still alarm on but can't call apnea).
    collapse_conf = (None if not collapse_suspect
                     else ("strong" if _fall_measured[0] else "weak"))

    # fall-onset event count (PRE-latch, so latch-merged falls still separate). Count +1 on the
    # first red trigger of an episode, then DISARM. Re-arm only after a genuine RECOVERY between
    # falls -- the person actually got UP (whole-cloud centroid risen, cloud_up) for
    # FALL_EVENT_GAP_S. Gating on cloud_up (not raw `down` clearing) is what makes it robust: a
    # far-fall CLOUD COLLAPSE drops `down` while the person is still on the floor (would falsely
    # re-arm -> overcount 231500/000000), but the mass never rises, so cloud_up stays false and
    # the onset does NOT re-arm; two DISTINCT falls have a real stand-up between them (cloud_up).
    # ⭐ C (user 0722): gate the cube-free sustained-down red by _CUBEFREE_FALL, matching the state
    # (L1103) and onset (L1114) legs. This was the LEAK: red_trigger/beep fired on sustained_fall
    # even with cube-free CLOSED -> the "closed" cube-free red still 3-beeped. Now RED = cube-confirmed
    # dec["fall"] only (unless CUBEFREE_FALL is explicitly on). Far falls red via the cube (z40 XY).
    red_trigger = bool(dec["fall"] or (_CUBEFREE_FALL and sustained_fall))
    if red_trigger and _fall_onset_armed[0]:
        _fall_event_n[0] += 1
        _fall_onset_armed[0] = False
        if now - _confirm_beep_t[0] >= CONFIRM_BEEP_MIN_S:   # debounce: onset can re-arm on flicker
            _beep(3, gap=0.18, overlap=True, snd=_BEEP_SND_CONFIRM)   # ⭐ CONFIRMED fall -> ALARM burst
            _confirm_beep_t[0] = now
    # SUSPECTED (pre-confirm) -> a single LIGHT click (distinct timbre from the CONFIRMED alarm), on
    # the rising edge. No debounce needed: susp_gate already fires once per episode (the 18 s 3001
    # noise gate subsumes it), so a bare edge-detect gives exactly one click when ExtraMLP declares.
    if (fall_state == "suspected" and _beep_last_state[0] != "suspected"
            and now - _suspect_beep_t[0] >= SUSPECT_BEEP_MIN_S):
        _beep(1, snd=_BEEP_SND_SUSPECT)
        _suspect_beep_t[0] = now
    _beep_last_state[0] = fall_state
    if down:
        _down_clear_since[0] = 0.0
    else:                                         # ⭐ ROUND MODEL: raw `down` clear -> round over grace ->
        if _down_clear_since[0] == 0.0:           # re-arm for the NEXT round (was gated on cloud_up; the
            _down_clear_since[0] = now             # round model ends a round on down-clear, not on a risen
        elif now - _down_clear_since[0] >= FALL_EVENT_GAP_S:   # cloud -- a still-down body that re-asserts
            _fall_onset_armed[0] = True           # `down` just re-detects as a new round, which is fine)
    # DIAG: whenever a fall trigger is active, log the full gate breakdown so a missed
    # red-Fall can be pinned to the exact failing gate. Goes to stdout AND
    # record/fall_debug.log (so it can be read back without copy-paste). Remove once tuned.
    if down or dec["trigger"] or cube_ev is not None:
        cr = _cube_result
        _line = (f"[fall] {fall_state:9s} down={int(down)}(w={int(w_down)}/{w_src},"
                 f"real_inst={int(real_inst)},real={int(real_person)},floor={int(floor_fall)}) "
                 f"hs={w_hs if w_hs is None else round(w_hs,2)} prim=tid{prim['tid'] if prim else '-'} "
                 f"n={prim.get('n') if prim else '-'} pffrac={prim_ffrac:.2f} busy={int(_cube_busy[0])} "
                 f"cube_res(rr={cr['rr']},str={cr['strength']},ffrac={cr['floor_frac']},"
                 f"z40={cr.get('z40')},bin={cr.get('bin')},age={now-cr['t']:.1f}s) "
                 f"run={_cleaner.run} P={fall_p} conf={dec['confidence']} reason={dec['reason']} cleaned={dec['cleaned']}"
                 + (f" {_cube_void}" if _cube_void else ""))
        print(_line, flush=True)
        try:
            with open(os.path.join(os.path.dirname(__file__), "..", "record", "fall_debug.log"), "a") as _fh:
                _fh.write(_line + "\n")
        except Exception:
            pass
    # mark the cube episode active while RAW down/floor-fall is in play, so the episode resets (and the
    # round re-arms) CUBE_RESET_S after `down` clears -- NOT gated on the FALL_HOLD_S red latch (the round
    # ends on down-clear even while the red stays visible for its dashboard hold).
    if down or floor_fall:
        _cube_last_active[0] = now
    return {"live": True, "points": pts, "targets": tg,
            "height_cm": None if z_cm is None else round(z_cm),
            "src": src, "cube_entries": int(sc.get("n_cube", 0)),
            "mount_cm": round(mount_cm), "tilt_deg": TILT, "diag": diag,
            "boxes": boxes, "pc": pc_pts, "fall_state": fall_state,
            "fall_p": fall_p, "primary_pose": primary_pose,
            "lying_state": lying_state,          # ⭐ ExtraMLP state classifier: person lying on floor?
            # ⭐ CUBE fire 2D contract (FROZEN 2026-07-22): lying (Y/N/None=作废, cube_ff-primary/
            # z40-fallback presence) + living_state ("Living"=rr|micro / "?"=body but unmeasurable /
            # None=no fresh cube). "?" is a LABEL, NOT collapse -- must never escalate to 💔.
            "cube_lying": cube_lying, "cube_living_state": cube_living_state,
            # ⭐ cardiac/collapse-suspect (fallen + immobile + no confirmed breathing) and the
            # distinct-event count (latch-blind, separates falls the 30 s hold merges).
            "collapse_suspect": collapse_suspect, "collapse_conf": collapse_conf,
            "living_confirmed": _fall_living[0], "fall_event": _fall_event_n[0],
            "fall_ev": {"window": bool(w_down), "real": real_person, "win_src": w_src,
                        "h_s": (round(w_hs, 2) if w_hs is not None else None),
                        "reason": dec["reason"], "cleaned": dec["cleaned"],
                        "cube": (cube_ev is not None),
                        "rr": (cube_ev.get("rr") if cube_ev else None),
                        "floor_fall": floor_fall,
                        # fused-P feature breakdown (for the dashboard '跌倒概率 P 融合' tooltip)
                        "P": fall_p, "f_mlp": round(_mlp_p, 2), "f_win": int(bool(w_down)),
                        "f_height": (None if cloud_wz_med is None else round(cloud_wz_med, 2)),
                        "f_energy": round(cloud_below_frac, 2), "f_geom": _geom_flat,
                        "f_rr": int(_rr_ok),
                        "floor_frac": prim_ffrac, "floor_cells": len(_floor.hg),
                        # ⭐ collapse + full feature vector (for scene_features.py training set)
                        "collapse": collapse_suspect, "collapse_conf": collapse_conf,
                        "had_rr": _fall_had_rr[0], "living": _fall_living[0],
                        "micro": _micro, "measured": _fall_measured[0],
                        "event": _fall_event_n[0],
                        "down": bool(down), "down_dur": round(down_dur, 1),
                        "sustained": bool(sustained_fall), "pose": primary_pose,
                        "prim_n": (int(prim.get("n", 0)) if prim else 0),
                        "cube_str": _cube_result["strength"],
                        "cube_bursts": _cube_episode[0]},
            "elev_acc_deg": ELEV_ACC_DEG,
            # cube RR (breathing from the fall cube second-check, SAME estimator as the
            # vitals RR: bcg_vitals demod_channels + estimate_rr). Surfaced top-level so the
            # dashboard shows it even when the vitals /api/state RR is idle (no still track).
            # ⭐ gate the displayed cube RR on freshness (12 s, == cube_ev/_cube_fresh): once the
            # person is up and the 18 s burst is spent, the stale RR must CLEAR, not linger (user
            # 2026-07-20: showed "23.7 s ago" after the person got up).
            "cube_rr": (_cube_result["rr"] if _cube_fresh else None),
            "cube_rr_str": (_cube_result["strength"] if _cube_fresh else 0.0),
            "cube_rr_age": (round(time.time() - _cube_result["t"], 1) if (_cube_fresh and _cube_result["t"]) else None),
            "age_s": round(time.time() - sc.get("t", 0), 1)}


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, body, ctype="application/json"):
        try:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body if isinstance(body, bytes) else body.encode())
        except (BrokenPipeError, ConnectionResetError):
            pass   # client (browser poll) closed mid-response — harmless

    def log_message(self, *a):
        pass       # quiet the per-request access log

    def do_GET(self):
        u = urlparse(self.path)
        if u.path in ("/", "/index.html"):
            with open(os.path.join(HERE, "dashboard.html"), "rb") as f:
                return self._send(200, f.read(), "text/html; charset=utf-8")
        if u.path == "/api/meta":
            return self._send(200, json.dumps(_meta))
        if u.path == "/api/fall/reset":              # clear a latched red Fall
            _fall_latch_until[0] = 0.0
            _cube_confirmed_episode[0] = False
            _cube_neg_run[0] = 0
            return self._send(200, json.dumps({"fall_reset": True}))
        if u.path == "/api/scene":
            _last_client_scene[0] = time.time()
            try:
                return self._send(200, json.dumps(_scene()))
            except Exception as e:
                return self._send(200, json.dumps({"live": False, "error": str(e)}))
        if u.path == "/api/cube":                    # server-triggered cube fetch (fall 2nd-check)
            q = parse_qs(u.query)
            rb = int(q.get("bin", [36])[0]); n = int(q.get("n", [30])[0]); hw = int(q.get("hw", [3])[0])
            if not hasattr(_src, "request_cube"):
                return self._send(200, json.dumps({"error": "source has no request_cube (not live)"}))
            ents = _src.request_cube(rb, n_frames=n, half_win=hw)
            bins = sorted({int(e.range_bin) for e in ents})
            return self._send(200, json.dumps({"bin": rb, "half_win": hw, "n_frames": n,
                                                "entries": len(ents), "range_bins": bins,
                                                "n_ant": (len(ents[0].vec) if ents else 0)}))
        if u.path == "/api/state":
            q = parse_qs(u.query)
            bl = int(q["bin_lo"][0]) if "bin_lo" in q else None
            bh = int(q["bin_hi"][0]) if "bin_hi" in q else None
            try:
                return self._send(200, json.dumps(_state(bl, bh)))
            except Exception as e:
                return self._send(200, json.dumps({"error": str(e)}))
        if u.path in ("/api/rec/on", "/api/rec/off"):
            # SAVE switch: the stream/serve keep running; only disk persistence
            # toggles. ON => 5-min rolling files (0-4,5-9,...) unchanged. OFF =>
            # stop immediately, flushing the current partial (last record = now).
            on = u.path.endswith("/on")
            r = _src.rec_set(on) if hasattr(_src, "rec_set") else {"error": "not a live source"}
            return self._send(200, json.dumps(r))
        if u.path == "/api/rec/status":
            r = _src.rec_status() if hasattr(_src, "rec_status") else {"saving": False}
            return self._send(200, json.dumps(r))
        if u.path == "/api/replay/list":
            # npz recordings available to replay (case/ + record/), newest first.
            import glob
            root = os.path.join(HERE, "..")
            files = []
            for sub in ("case", "record"):
                for p in glob.glob(os.path.join(root, sub, "*.npz")):
                    try:
                        st = os.stat(p)
                    except OSError:
                        continue
                    files.append({"path": f"{sub}/{os.path.basename(p)}",
                                  "label": f"{sub}/{os.path.basename(p)}",
                                  "size": st.st_size, "mtime": st.st_mtime})
            files.sort(key=lambda f: f["mtime"], reverse=True)
            return self._send(200, json.dumps({"files": files}))
        if u.path == "/api/replay":
            # Run the npz back through the REAL fall pipeline in a SUBPROCESS (fall_replay.py
            # monkeypatches module globals -> must NOT run in this live process). Returns the
            # code-of-record fall verdict + timeline. file must sit under case/ or record/.
            import subprocess
            q = parse_qs(u.query)
            rel = (q.get("file", [""])[0]).replace("\\", "/")
            mnt = q.get("mount", [str(MOUNT if MOUNT is not None else 2.0)])[0]
            tlt = q.get("tilt", [str(TILT or 25.0)])[0]
            if (not rel or ".." in rel or rel.split("/")[0] not in ("case", "record")
                    or not rel.endswith(".npz")):
                return self._send(400, json.dumps({"error": "bad file (want case/*.npz or record/*.npz)"}))
            root = os.path.abspath(os.path.join(HERE, ".."))
            fp = os.path.abspath(os.path.join(root, rel))
            if not fp.startswith(root + os.sep) or not os.path.exists(fp):
                return self._send(404, json.dumps({"error": "file not found"}))
            try:
                out = subprocess.run(
                    [sys.executable, os.path.join(HERE, "fall_replay.py"), fp,
                     "--mount", str(mnt), "--tilt", str(tlt), "--json"],
                    capture_output=True, text=True, timeout=180, cwd=root)
                if out.returncode != 0:
                    return self._send(200, json.dumps({"error": "replay failed",
                                                       "stderr": out.stderr[-2000:]}))
                return self._send(200, out.stdout.strip() or "{}")
            except subprocess.TimeoutExpired:
                return self._send(200, json.dumps({"error": "replay timed out"}))
            except Exception as e:
                return self._send(200, json.dumps({"error": str(e)}))
        if u.path == "/api/quit":
            # graceful programmatic shutdown (== Ctrl+C): flush + release the
            # read-only serial attach without stopping the sensor.
            self._send(200, json.dumps({"quitting": True}))
            if _shutdown:
                threading.Thread(target=_shutdown, daemon=True).start()
            return
        self._send(404, "{}")

    def log_message(self, *a):
        pass                                    # quiet


def main():
    global _src, _meta, TILT, MOUNT
    argv = sys.argv[1:]
    for flag, setter in (("--tilt", "TILT"), ("--mount", "MOUNT")):
        if flag in argv:
            i = argv.index(flag)
            globals()[setter] = float(argv[i + 1]); del argv[i:i + 2]
    # Recording is available as part of the live service but the WRITE switch starts
    # OFF (see --save-on below). Custom file name via --record NAME; --no-record
    # disables the recorder entirely (WRITE btn shows unavailable).
    rec_name = "live"
    if "--record" in argv:
        i = argv.index("--record"); rec_name = argv[i + 1]; del argv[i:i + 2]
    no_record = "--no-record" in argv
    if no_record:
        argv.remove("--no-record")
    # Recording (WRITE) is OFF by default — stream/serve only until the user turns
    # it on (dashboard WRITE btn or /api/rec/on). Use --save-on to start recording
    # immediately. (--save-off still accepted as a no-op = the default.)
    save_on = "--save-on" in argv
    if save_on:
        argv.remove("--save-on")
    if "--save-off" in argv:
        argv.remove("--save-off")
    spec = argv[0] if argv else "live"
    port = int(argv[1]) if len(argv) > 1 else 8765
    record_prefix = None
    if spec == "live" and not no_record:
        # recordings ALWAYS land in pc/record/ (raw, gitignored) as
        # pc/record/<name>_<5min-stamp>.npz; cp what validation needs into pc/case/.
        rec_dir = os.path.join(os.path.dirname(HERE), "record")
        os.makedirs(rec_dir, exist_ok=True)
        record_prefix = rec_name if os.path.sep in rec_name else os.path.join(rec_dir, rec_name)
    # ⭐ SELF-HEAL, BEFORE the source grabs the ports (2026-07-23). On the 6844 the flash holds the
    # FIRMWARE but not the CONFIG, so a power-cycled EVM boots to `mmwDemo:/>` and streams nothing
    # until a host pushes a cfg -- ports enumerate, `version` answers normally, DATA stays at
    # exactly zero bytes. (That was this morning's "dead radar"; it was NOT a wedge, and NOT a
    # leftover sensorStop -- this server closes with stop_sensor=False.) Normally the TI visualiser
    # pushes the cfg; with the Mac driving the sensor nobody did.
    # It MUST happen here, on a private connection: pushing through the live RadarSession fails
    # silently -- its drain thread eats the CLI replies, so sensorStart never takes (observed:
    # cubeQuery still answered [OK] while the frame loop stayed dead).
    if spec == "live" and os.path.exists(AUTO_CFG):
        try:
            import serial as _ser
            _d = _ser.Serial(LiveSource.DATA, 1250000, timeout=0.4)
            _t0, _n = time.time(), 0
            while time.time() - _t0 < 2.0:
                _n += len(_d.read(8192))
            _d.close()
            if _n < 500:
                print(f"DATA silent ({_n} B/2s) -> pushing {os.path.basename(AUTO_CFG)}", flush=True)
                _c = _ser.Serial(LiveSource.CLI, 115200, timeout=1.2)
                _c.reset_input_buffer()
                _fatal = []
                for _raw in open(AUTO_CFG):
                    _l = _raw.strip()
                    if not _l or _l.startswith("%"):
                        continue
                    _c.write(_l.encode() + b"\r\n"); _c.flush()
                    time.sleep(4.0 if _l.startswith("sensorStart") else 1.0)
                    _rsp = _c.read(4096).decode("ascii", "replace")
                    if "already defined" in _rsp or "mmWave open failed" in _rsp:
                        _fatal.append(_l.split()[0])
                _c.close()
                if _fatal:
                    # ⭐ The cfg is ONE-SHOT PER BOOT (measured 2026-07-23): `antGeometryBoard`
                    # answers "Antenna geometry is already defined" on a second parse, and once the
                    # sensor has been stopped `sensorStart` fails with "mmWave open failed
                    # [-203227134]" -- the RF front end cannot be reopened. So this self-heal can
                    # rescue a FRESHLY BOOTED EVM (its intended case: flash keeps the firmware, not
                    # the config, so a power-cycled board idles at the prompt) but nothing can
                    # rescue a configured-then-stopped sensor. Say so instead of idling silently.
                    print("*" * 78 + "\n"
                          f"  CFG REJECTED ({', '.join(sorted(set(_fatal)))}) — the EVM was already\n"
                          "  configured this boot and the RF front end will not reopen.\n"
                          "  >>> POWER-CYCLE THE EVM. <<<  No host-side command can recover this;\n"
                          "  the cfg parses only once per boot. (Never send sensorStop to this\n"
                          "  firmware -- there is no way back from it.)\n" + "*" * 78, flush=True)
                time.sleep(1.0)
        except Exception as _e:
            print(f"cfg self-heal skipped: {type(_e).__name__}: {_e}", flush=True)
    _src = make_source(spec, record_prefix=record_prefix, save_on=save_on)
    _src.start()
    # live source needs a moment for the reader thread to see the first frames
    # (bins are empty until then); wait so measurable_range has a range to report.
    for _ in range(150):
        m = _src.meta()
        if m["bins"]:
            break
        time.sleep(0.1)
    m = _src.meta()
    # NOTE: m["bins"] fills only from cubeQuery 320 data, so it is EMPTY on a healthy point-cloud
    # stream -- do NOT use it as a liveness signal (it produced a spurious "no radar frames" every
    # startup). Real liveness is the frame reader: LiveSource logs "no radar frames for Ns — WEDGED"
    # from its own thread when the DATA line goes silent, which is the signal the monitor watches.
    _meta = dict(source=m, win_s=WIN_S, mount_calibrated=(TILT is not None and MOUNT is not None),
                 tilt_deg=TILT, h_mount=MOUNT,
                 range=pipe.measurable_range(m["bins"], m["dr"]) if m["bins"] else None)
    rec = f"  recording 5-min files -> {record_prefix}_*.npz" if record_prefix else ""
    print(f"source={m['kind']} ({m['name']})  bins {min(m['bins']) if m['bins'] else '?'}-"
          f"{max(m['bins']) if m['bins'] else '?'}  serving http://127.0.0.1:{port}{rec}", flush=True)
    # ⭐ SELF-TICK (2026-07-23): _scene() only ran when something GET /api/scene -- the server was
    # PURELY passive. Close the dashboard and fall detection, 5-min recording and the hourly
    # baseline sweep all silently stop, which is exactly wrong for an unattended run (verified live:
    # no sweep fired for 90 s until a curl arrived). This drives the pipeline itself, but ONLY while
    # no client is polling, so an open dashboard keeps its own cadence and _scene() is not called
    # from two threads at once in the normal case.
    def _self_tick():
        while True:
            try:
                time.sleep(0.5)
                if time.time() - _last_client_scene[0] > 2.0:
                    _scene()
            except Exception:
                pass
    threading.Thread(target=_self_tick, daemon=True).start()
    srv = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    # graceful stop on Ctrl-C AND SIGTERM (plain `pkill`) so the read-only serial
    # attach is released cleanly and never wedges the firmware. NOTE: `kill -9`
    # (SIGKILL) is uncatchable and CAN wedge the UART -> then power-cycle the radar.
    import signal
    global _shutdown
    def _graceful(*_a):
        # ⭐ MUST run off the main thread (fixed 2026-07-23). Python delivers signals to the MAIN
        # thread, which is exactly where serve_forever() is blocked; ServerBase.shutdown() then
        # waits for that loop to finish -- and the loop cannot advance because the main thread is
        # sitting inside this handler. Self-deadlock: SIGTERM and SIGINT were both ignored, so every
        # stop ended in `kill -9`, the one thing this handler exists to avoid (SIGKILL is uncatchable
        # and can wedge the UART -- the "ports up, firmware alive, zero bytes" state found this
        # morning). Handing the shutdown to a helper thread lets the main loop unwind normally.
        threading.Thread(target=lambda: (_src.stop(), srv.shutdown()), daemon=True).start()
    _shutdown = _graceful                                 # exposed via /api/quit
    signal.signal(signal.SIGINT, _graceful)
    signal.signal(signal.SIGTERM, _graceful)
    try:
        srv.serve_forever()
    finally:
        _src.stop()


if __name__ == "__main__":
    main()
