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
from radar_source import make_source
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
_cube_busy = [False]         # a request_cube fetch is in flight (1-elem list = mutable flag)
_last_query_t = [0.0]        # last cubeQuery wall time (rate-limit: 1 per fall episode + refresh)
QUERY_REFRESH_S = 12.0       # min seconds between cubeQuery bursts while down. A 60-frame
                             # burst is ~6 s -> 6 s on + 6 s idle = 50% DATA-UART duty. The
                             # firmware wedges when 320 floods the DATA UART for MINUTES; a
                             # single 6 s burst is safe, but back-to-back 6 s bursts at ~60%
                             # duty accumulate and wedge it. The fix is a BIGGER idle gap
                             # (>=burst), NOT a tiny 0.4 s micro-gap (that raises duty ->
                             # worse). See [[fall-detection-design]] / the wedge in fall log.
STALE_GATE_S = 3.0           # NEVER cubeQuery when the scene is this stale: the sensor is
                             # wedged/stalled, so 320 bursts hit a DEAD firmware -- useless,
                             # and they can keep it from recovering. Gate every probe on this.
MAX_CUBE_BURSTS = 3          # HARD cap on cubeQuery bursts PER fall/lost episode -- NOT reset by
                             # finding RR. A fall grabs cube a FEW times to verify person-vs-
                             # object (RR + the fused MLP), then STOPS. The old "reset on RR"
                             # was the wedge cause: a breathing body returns RR every burst ->
                             # counter never advances -> 320 floods the DATA UART for the whole
                             # (100 s+) fall -> firmware WEDGES. Strictly bounded now: <=3 bursts
                             # (~18 s of cube) per episode, then silent; the fall stays latched
                             # via the window / floor-fall / sustained legs, no more cube needed.
FALL_FFRAC_MIN = 0.15        # sustained-down -> red Fall ONLY if the cloud is really below the
                             # floor line. A ~0.45 m furniture cluster (floor_frac~0.02) must
                             # NOT latch a permanent fall (that was the 22-minute false latch).
_cube_episode = [0]          # cube bursts fired in the CURRENT physical fall episode, UNIFIED
                             # across down-probe + lost-probe. NOT per floor-track id -- those
                             # CHURN with fragmentation, so a per-id cap leaked (each new id got
                             # a fresh 3-burst budget -> dozens of bursts -> re-wedge). Hard cap.
_cube_last_active = [0.0]    # last wall time a fall was active (down / floor-fall / latched)
CUBE_RESET_S = 5.0           # after this long with NO active fall (person up & gone), the cube
                             # episode budget resets -> the next distinct fall gets its own bursts
# ⭐ 3001-FIRST tiering emulated by DELAY (user 2026-07-18b): the deployed product bursts 18 s of
# 3001 (cheap micro-motion/living confirm) BEFORE the bandwidth-heavy cube. In the dev prototype
# 3001 streams continuously, so we get the SAME data ordering for free by just holding the cube
# query CUBE_DELAY_S after the episode's trigger: 0-18 s the living gate = the 3001 below-floor
# cloud (floor_fall, inherently micro-motion-keyed) + sustained-down -> red WITHOUT cube; at 18 s
# the cube adds RR/floor-energy (tier 2) and gives the server ExtraMLP its cube features.
CUBE_DELAY_S = 3.0           # hold the episode's FIRST cubeQuery this long after the trigger. Was 18s
                             # (the "3001-first" tier: cube-free red covered 0-18s). Now that the
                             # cube-free red is CLOSED, red REQUIRES the cube -> an 18s delay is an
                             # 18s RED-BLIND window at the START of every fall: a fall down <18s (or a
                             # short/flickering far fall) never fetches a cube -> never reds -> MISSED
                             # (live 040500: down 13.6s at 5.2m, reported only ~18s later at the record
                             # boundary). 3s = enough to debounce the trigger, fast enough to catch short
                             # falls and alarm in ~3-9s (the burst). Budget still capped by MAX_CUBE_BURSTS.
_cube_episode_t0 = [0.0]     # wall time the current cube episode's trigger fired (0 = no episode)
_real_since = [0.0]          # last time the real-person gate was instantaneously true
REAL_GRACE_S = 2.0           # hold real-person through brief point-count dips (see below)
_fall_latch_until = [0.0]    # a confirmed red Fall LATCHES the display until this wall time
FALL_HOLD_S = 30.0           # keep showing red Fall this long after the last confirmation
                             # (a caregiver must SEE it; it must not clear when the person
                             # stirs/gets up). Cleared by /api/fall/reset or a clear recovery.
_recover_since = [0.0]       # wall time the person has been continuously upright & recovered
RECOVER_S = 2.0              # a CLEAR recovery this long CLEARS the latch early: a tracked real
                             # person whose WHOLE-cloud median height is up (RECOVER_ZMED). Uses
                             # the robust cloud centroid, NOT the per-box pose/ffrac -- those
                             # thrash as the walking-away body FRAGMENTS (000000: ffrac 0<->1,
                             # pose SIT/STAND/LIE flicker, a stray fragment even re-latched the
                             # red). The centroid stays ~+0.7 m once up. Only clears when up.
RECOVER_ZMED = 0.4           # whole-cloud median world height above this = the mass is UP (a
                             # lying body is -0.2..-0.5; standing/sitting is +0.5..+0.9).
CANCEL_R = 0.5               # 30 s monitor: if a GTRACK track stands UP within this radius of
                             # where the body fell (the person got up unaided at the fall spot),
                             # DISCARD the fall -- a stumble that self-recovers is not an
                             # emergency. Faster + more specific than the cloud-centroid clear.
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
# Lost-track RR probe: when GTRACK drops a still person's track (FloorTracker inherits it),
# actively cubeQuery that spot to get RR -- confirms a living body (vs furniture) and shows
# the RR for a sitting/fallen still person. WAIT 2 s first: most track losses are brief
# flickers that re-acquire, and we must not spend a ~6 s cube burst on those.
LOST_WAIT_S = 2.0            # a track must stay lost this long (not a flicker) before probing
LOST_QUERY_REFRESH_S = 12.0 # min seconds between lost-probe cubeQuery bursts per track (50% duty)
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


def _fetch_cube_bg(range_bin, floor_frac, n_frames=60):
    """Background: burst 320 at range_bin, then compute RR from it (the cube second-check).
    n_frames sets the integration window: 60 (~6s, ~2 breaths) for a quick fall confirm;
    the lost-track/still-person probe uses a LONGER window (~15s) -- a still body is not
    time-limited, and longer coherent integration lifts weak breathing above the ~1um
    noise floor (SNR ~ sqrt(T)) AND sharpens RR resolution. Non-blocking for /api/scene."""
    try:
        if hasattr(_src, "request_cube"):
            ents = _src.request_cube(range_bin, n_frames=n_frames, half_win=3,
                                     timeout=n_frames / 10.0 + 3.0)
            rr, strength, micro, measured = _rr_from_cube(ents)
            cube_ff = _cube_floor_energy(ents)          # MUSIC power-weighted floor band (or None)
            z40 = None
            if _Z40_PRESENCE and ents:                  # 思路B presence vs the fixed background
                try:
                    import numpy as _np2
                    from collections import defaultdict as _dd
                    from spatial3d.range_music import DR_M as _DRM
                    from spatial3d.occupancy_ratio import z40_present_from_cov
                    byb = _dd(list)
                    for e in ents:
                        byb[int(e.range_bin)].append(_np2.asarray(e.vec, complex))
                    lbins = sorted(b for b in byb if len(byb[b]) >= 8)
                    if lbins:
                        lcov = [((_np2.stack(byb[b]).conj().T @ _np2.stack(byb[b])) / len(byb[b]))
                                for b in lbins]
                        drr = float((_meta or {}).get("source", {}).get("dr") or _DRM)
                        z40 = round(z40_present_from_cov(lcov, lbins, drr,
                                                         fall_range_m=range_bin * drr), 2)
                except Exception:
                    z40 = None
            _cube_result.update(rr=rr, strength=strength, t=time.time(),
                                floor_frac=round(float(floor_frac), 2), bin=int(range_bin),
                                cube_ff=(None if cube_ff is None else round(cube_ff, 2)),
                                micro=micro, measured=measured, z40=z40)
    except Exception:
        pass
    finally:
        _cube_busy[0] = False


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
    if _fall_anchor[0] is not None:
        return int(round(float(_fall_anchor[0]) / RANGE_STEP))
    pcx = sc.get("pc_xyz")
    if pcx is None or not len(pcx):
        return None
    th = _m.radians(TILT or 0.0)
    py, pz = pcx[:, 1], pcx[:, 2]
    wy = py * _m.cos(th) + pz * _m.sin(th)                # world GROUND range (matches 320)
    wz = MOUNT + pz * _m.cos(th) - py * _m.sin(th)        # world height -> pick the fallen body
    low = wz < (_floor.default + 0.5)
    sel = low if int(low.sum()) >= 4 else _np.ones(len(pcx), bool)
    return int(round(float(_np.median(wy[sel])) / RANGE_STEP))


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
    # UNIFIED cube budget: reset the per-episode burst count once the scene has been quiet
    # (no active fall) for CUBE_RESET_S -> the next distinct fall gets its own MAX_CUBE_BURSTS.
    if time.time() - _cube_last_active[0] > CUBE_RESET_S:
        _cube_episode[0] = 0
        _cube_episode_t0[0] = 0.0     # episode ended -> restart the 18 s cube-delay clock next fall
        _fall_had_rr[0] = False       # new physical fall -> re-assess vitals from scratch
        _fall_living[0] = False; _fall_measured[0] = False
        _collapse_since[0] = 0.0
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
            # ⭐ ExtraMLP lying verdict: the trained 3001-height logistic (hi2/floorfrac/yspan/n3001)
            # OWNS lie-vs-stand. micro imputes to 0 (per-cluster temporal micro not tracked at runtime
            # yet; weight is small). A body-size floor gate (LYING_MIN_N) still guards against furniture
            # fragments; RR person-vs-furniture is applied later on the fetched cube.
            _cplie = _extramlp.p_lie({"hi2": _cz90, "floorfrac": _cff, "yspan": _cyspan,
                                      "n3001": _ctot, "micro": 0.0})
            _clying_geom = bool(_cff >= LYING_FFRAC and _ctot >= LYING_MIN_N
                                and _cz90 < LYING_TOP_Z + LYING_TOP_RANGE_K * max(0.0, _cwy - 2.0))
            _clying = (bool(_ctot >= LYING_MIN_N and _cplie is not None and _cplie >= EXTRAMLP_LIE_THR)
                       if (_extramlp.ok and _EXTRAMLP_ON) else _clying_geom)
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
                # the counter kept it probing forever -> 320 flood -> WEDGE. Hard cap instead.
                # GATED: 3001-first DELAY elapsed (18 s of 3001 before the cube) AND sensor FRESH
                # (never flood a wedged firmware) AND under the per-episode HARD burst cap.
                if (_now - _lost_since[ftk.id] >= LOST_WAIT_S
                        and (_now - _cube_episode_t0[0]) >= CUBE_DELAY_S
                        and _now - _lost_query_t.get(ftk.id, 0) > LOST_QUERY_REFRESH_S
                        and not _cube_busy[0]
                        and (_now - sc.get("t", _now)) < STALE_GATE_S
                        and _cube_episode[0] < MAX_CUBE_BURSTS):
                    # WHERE to query (Q: lost坐标记录了吗/查的是它吗/考虑地板能量吗):
                    # 1) PREFER the floor-band ENERGY cluster (`_fall_anchor`, max floor_frac) --
                    #    a still/fallen body IS the floor energy; the FloorTracker position
                    #    `ftk.x,ftk.y` DRIFTS onto whatever orphan cloud is densest (190500: the
                    #    seated 2nd person, bin10) and wasted every burst there.
                    # 2) else the DEATH coordinate near this lost track (where a person went DOWN,
                    #    from `_fall_deaths`) -- not the drifted floor position.
                    # 3) else fall back to the floor-track position.
                    if _fall_anchor[0] is not None:
                        rb = int(round(float(_fall_anchor[0]) / RANGE_STEP))
                    else:
                        _dxy = [(dx, dy) for (dt, dx, dy) in _fall_deaths
                                if (dx - ftk.x) ** 2 + (dy - ftk.y) ** 2 < FALL_REGION_M ** 2]
                        _ax, _ay = _dxy[-1] if _dxy else (ftk.x, ftk.y)
                        rb = int(round(math.hypot(_ax, _ay) / RANGE_STEP))
                    _cube_busy[0] = True
                    _lost_query_t[ftk.id] = _now
                    _cube_episode[0] += 1
                    # 60 frames (~6s): a single LONG cubeQuery (150/~15s) WEDGES the firmware
                    # -- the sustained 320 flood over DATA UART kills it ([NO-Done] + no frames).
                    # Long integration for a still body must come from stacking SHORT bursts
                    # into a server-side sliding buffer (option A), not one long burst.
                    threading.Thread(target=_fetch_cube_bg, args=(rb, 1.0, 60),
                                     daemon=True).start()
        for d in [i for i in _lost_since if i not in alive_ids]:   # forget gone tracks
            _lost_since.pop(d, None); _lost_query_t.pop(d, None)

        # ⭐ FAR-RANGE FORCE (user 2026-07-20): a GTRACK death (lost trigger) beyond FAR_FORCE_M gets a
        # cubeQuery REGARDLESS of the 3001 person/veto gate. Past ~4.5 m the cloud collapses so 3001
        # can't confirm a person, and the lost-probe's person-gate (needs inherited tid + RR) blocks
        # exactly the far lie we must catch. NO 18 s 3001-first wait: 3001 is useless at far range and
        # the death memory only lives FALL_DEATH_S (8 s). RR/micro on the returned cube then confirm.
        _far_death = next(((dt, dx, dy) for (dt, dx, dy) in reversed(_fall_deaths)
                           if math.hypot(dx, dy) > FAR_FORCE_M and _now - dt >= LOST_WAIT_S), None)
        if (_far_death is not None and not _cube_busy[0]
                and (_now - sc.get("t", _now)) < STALE_GATE_S
                and _now - _last_query_t[0] > QUERY_REFRESH_S
                and _cube_episode[0] < MAX_CUBE_BURSTS):
            if _cube_episode_t0[0] == 0.0:
                _cube_episode_t0[0] = _now
            rb = int(round(math.hypot(_far_death[1], _far_death[2]) / RANGE_STEP))
            _cube_busy[0] = True; _last_query_t[0] = _now; _cube_episode[0] += 1
            threading.Thread(target=_fetch_cube_bg, args=(rb, 1.0, 60), daemon=True).start()

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
        _fall_anchor[0] = fall_cl["wy_med"] if fall_cl is not None else None
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
    if down and _cube_episode_t0[0] == 0.0:                 # mark the episode trigger (3001-first clock)
        _cube_episode_t0[0] = now

    # server-triggered cube fetch: HELD CUBE_DELAY_S after the trigger (3001-first tiering) then
    # rate-limited (50% duty refresh) — not every scene call. Range from the cloud GROUND wy
    # (_fall_range_bin, now the per-cluster fall anchor). GATED on: sensor is FRESH (never flood a
    # wedged firmware) AND under the per-episode HARD burst cap (MAX_CUBE_BURSTS). During 0-18 s
    # the living gate is the 3001 below-floor cloud (floor_fall) + sustained-down -> red needs no cube.
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
    if (cube_gate and not _cube_busy[0] and fresh
            and (now - _last_query_t[0]) > QUERY_REFRESH_S
            and _cube_episode[0] < MAX_CUBE_BURSTS):
        rb = _fall_range_bin(sc)
        if rb is not None:
            _cube_busy[0] = True
            _last_query_t[0] = now
            _cube_episode[0] += 1
            threading.Thread(target=_fetch_cube_bg, args=(rb, prim_ffrac), daemon=True).start()

    # cube second-check evidence = RR + floor energy computed FROM the fetched 320 burst
    # (self-contained; a living body on the floor breathes -> RR -> red Fall; a dropped
    # object does not -> no red). A confirmed RR is HELD 12 s (the person stays down and
    # breathing; a single 3 s burst is < 1 breath cycle, so bridge the gaps) — the ~4 s
    # re-query keeps it refreshed while down.
    cube_ev = None
    if now - _cube_result["t"] < 12.0:
        # Tier-2 floor evidence = the BETTER of the sparse 3001 floor_frac and the cube's OWN
        # MUSIC power-weighted band (the latter sees a far/specular lying body where 3001 collapsed).
        # PRESENCE = the CUBE's OWN floor energy (its MUSIC band), NOT the sparse 3001 floor_frac:
        # the 3001 floor_frac reads a standing-behind-chair low cluster as floor-dominated (=1.0) and
        # an UNASSESSED cube (0 ents) then falsely asserts a body -> cube-collapse red. cube_ff is None
        # when the cube returned nothing -> 0 -> not present -> not a fall (unassessed, correctly).
        _ff_use = float(_cube_result.get("cube_ff") or 0.0)
        if _Z40_PRESENCE and _cube_result.get("z40") is not None:   # 思路B (furniture/ghost rejected) wins
            _ff_use = 1.0 if float(_cube_result["z40"]) >= 0.4 else 0.0
        # micro = living micro-motion (confirms a person when RR can't lock: back-to-radar/occluded)
        cube_ev = {"rr": _cube_result["rr"], "floor_frac": _ff_use, "micro": _cube_result.get("micro")}

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
    # body. The cube-RR path (above) still confirms faster for a clear adult.
    if down:
        if _down_since[0] == 0.0:
            _down_since[0] = now
        _down_last[0] = now
    elif _down_since[0] and (now - _down_last[0]) > DOWN_GAP_S:
        _down_since[0] = 0.0                       # down truly gone -> reset the sustain timer
    down_dur = (now - _down_since[0]) if _down_since[0] else 0.0
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

    # LATCH a confirmed red Fall so it stays visible on the dashboard for FALL_HOLD_S even
    # after the person stirs/gets up (a ~6 s red that clears the instant they move is easy
    # to miss). Cleared by GET /api/fall/reset or a sustained clear recovery below.
    # `cloud_up` = the WHOLE-cloud centroid is up (robust to the per-box fragmentation that
    # thrashes pose/ffrac as a body walks away). Don't RE-LATCH on a stray low fragment while
    # the mass is clearly up -- that was extending the red long after the person got up.
    cloud_up = cloud_wz_med is not None and cloud_wz_med > RECOVER_ZMED
    if (dec["fall"] or (_CUBEFREE_FALL and sustained_fall)) and not cloud_up:
        _fall_latch_until[0] = now + FALL_HOLD_S
    # RECOVERY / 30 s-CANCEL clears the latch early: the person got up and moved on -- a stumble
    # that self-recovers is not an emergency. Signal = the CLOUD/energy centroid at the fall spot
    # has RISEN (cloud_up). We do NOT use the GTRACK track's Z here: it floats ~1 m on a still
    # body (fall-design), so a fallen person's coasting track reads "up" and would falsely cancel
    # a real fall (it wrecked fall B in 000000). The cloud centroid is the reliable "up". A track
    # near the fall spot must ALSO show the cloud up -- which cloud_up already requires. Held
    # RECOVER_S to debounce, and only when a real person is present (real_inst).
    if real_inst and cloud_up:
        if _recover_since[0] == 0.0:
            _recover_since[0] = now
        if now - _recover_since[0] >= RECOVER_S:
            _fall_latch_until[0] = 0.0
            _last_low_xy[0] = None                 # episode discarded -> forget the fall spot
    else:
        _recover_since[0] = 0.0
    if now < _fall_latch_until[0]:
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
    red_trigger = bool(dec["fall"] or sustained_fall)
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
    elif cloud_up:                                # person genuinely upright (mass risen)
        if _down_clear_since[0] == 0.0:
            _down_clear_since[0] = now
        elif now - _down_clear_since[0] >= FALL_EVENT_GAP_S:
            _fall_onset_armed[0] = True           # recovered -> ready for the next DISTINCT fall
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
                 f"run={_cleaner.run} P={fall_p} conf={dec['confidence']} reason={dec['reason']} cleaned={dec['cleaned']}")
        print(_line, flush=True)
        try:
            with open(os.path.join(os.path.dirname(__file__), "..", "record", "fall_debug.log"), "a") as _fh:
                _fh.write(_line + "\n")
        except Exception:
            pass
    # mark the cube episode active while a fall is in play, so the budget only resets after
    # CUBE_RESET_S of genuine quiet (person up & gone) -- see the reset at the top of _scene.
    if fall_state == "fall" or down or floor_fall:
        _cube_last_active[0] = now
    return {"live": True, "points": pts, "targets": tg,
            "height_cm": None if z_cm is None else round(z_cm),
            "src": src, "cube_entries": int(sc.get("n_cube", 0)),
            "mount_cm": round(mount_cm), "tilt_deg": TILT, "diag": diag,
            "boxes": boxes, "pc": pc_pts, "fall_state": fall_state,
            "fall_p": fall_p, "primary_pose": primary_pose,
            "lying_state": lying_state,          # ⭐ ExtraMLP state classifier: person lying on floor?
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
            return self._send(200, json.dumps({"fall_reset": True}))
        if u.path == "/api/scene":
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
    if not m["bins"]:
        print("WARN: no radar frames yet (sensor streaming? ports free?). Serving anyway.",
              flush=True)
    _meta = dict(source=m, win_s=WIN_S, mount_calibrated=(TILT is not None and MOUNT is not None),
                 tilt_deg=TILT, h_mount=MOUNT,
                 range=pipe.measurable_range(m["bins"], m["dr"]) if m["bins"] else None)
    rec = f"  recording 5-min files -> {record_prefix}_*.npz" if record_prefix else ""
    print(f"source={m['kind']} ({m['name']})  bins {min(m['bins']) if m['bins'] else '?'}-"
          f"{max(m['bins']) if m['bins'] else '?'}  serving http://127.0.0.1:{port}{rec}", flush=True)
    srv = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    # graceful stop on Ctrl-C AND SIGTERM (plain `pkill`) so the read-only serial
    # attach is released cleanly and never wedges the firmware. NOTE: `kill -9`
    # (SIGKILL) is uncatchable and CAN wedge the UART -> then power-cycle the radar.
    import signal
    global _shutdown
    def _graceful(*_a):
        _src.stop(); srv.shutdown()
    _shutdown = _graceful                                 # exposed via /api/quit
    signal.signal(signal.SIGINT, _graceful)
    signal.signal(signal.SIGTERM, _graceful)
    try:
        srv.serve_forever()
    finally:
        _src.stop()


if __name__ == "__main__":
    main()
