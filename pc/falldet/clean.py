"""Module 3 — CLEANING / decision. Fuses the two independent triggers and removes
false positives with server-side evidence. This is where the raw, LIBERAL on-chip
triggers get cleaned into a confident fall decision.

Design (per data 20260716):
  liberal trigger  = WindowDetector.down  OR  MLP falling/lying prob high
     (recall high, precision low — intentional; either firing bursts the cube)
  clean it with, in order:
    1. cube second-check  — the retrieved 320 cube must show a LIVING body on the floor:
       RR present (person, not a dropped object) AND floor-band energy fraction high.
       This is the strong, validated filter (radar_pipeline). Fails -> reject.
    2. geometry prior     — lying at a known rest spot (bed/sofa) -> downgrade, not a fall.
    3. persistence        — the person stays DOWN (can't get up) for a while.
The two triggers are COMPLEMENTARY: window=sustained down-state (robust to track freeze),
MLP=falling motion + free pose. OR them for recall; the cube/geom/persist gates give
precision.
"""


class Cleaner:
    def __init__(self, mlp_trig=0.5, persist=10, floor_frac_min=0.7,
                 extra_confirm_min=0.45, require_cube=False):
        self.mlp_trig = mlp_trig          # MLP falling/lying prob that counts as a trigger
        self.persist = persist            # frames the trigger must hold for a confirmed fall
        self.floor_frac_min = floor_frac_min
        # server-side 3001/floor/geom evidence strong enough (summed extra_score) to CONFIRM a fall
        # when NO cube was fetched. Needs ~3 independent signals to reach this, so a single transient
        # signal can't escalate. Never overrides an explicit cube rejection.
        self.extra_confirm_min = extra_confirm_min
        # ⛔ require_cube: CLOSE the cube-free confirm. When True, a red Fall REQUIRES a fetched cube
        # that confirmed a living body on the floor (cube_ok True) -- the extra-evidence "confirm
        # without a cube" path is disabled. There is no cube-free fall (a real fall's 3001 cloud
        # collapses; only the cube's energy sees it). See radar_server _CUBEFREE_FALL.
        self.require_cube = require_cube
        self.run = 0

    def decide(self, window_out, mlp_out, cube=None, geom=None, extra=None):
        """window_out : {down, h_s}         from WindowDetector (may be None)
        mlp_out    : {pose, falling_p}    from MLPDetector    (may be None)
        cube       : {rr, floor_frac}     server 320 second-check (None if not fetched)
        geom       : {at_rest_spot: bool} location prior (None if unknown)
        extra      : server-side extra features (3001 cloud / floor energy / geometry)
        Returns {fall, suspected, collapse_suspect, confidence, trigger, reason, cleaned}.
        cube evidence is split: floor-energy (floor_frac) = PRESENCE gate (body-shaped floor
        reflector, breathing-independent); rr/micro = LIVENESS classifier (red vs 💔 collapse)."""
        w_down = bool(window_out and window_out.get("down"))
        m_fall = float(mlp_out.get("falling_p", 0.0)) if mlp_out else 0.0
        m_trig = m_fall >= self.mlp_trig
        trigger = w_down or m_trig
        self.run = self.run + 1 if trigger else 0

        conf, reason = 0.0, []
        if w_down: conf += 0.5; reason.append("window")            # sustained down-state
        if m_trig: conf += 0.3; reason.append(f"mlp:{mlp_out.get('pose')}")  # learned motion

        cleaned = None
        # 1) server-side 3001/floor/geometry features can boost the fusion confidence, and when
        #    strong enough (extra_confirm_min) CONFIRM a fall with no cube fetched (below).
        if extra is not None:
            extra_score = 0.0
            if extra.get("floor_fall"):
                extra_score += 0.25; reason.append("floor_fall")
            if extra.get("lying_state"):
                extra_score += 0.15; reason.append("lying_state")
            if float(extra.get("cloud_below_frac", 0.0)) >= self.floor_frac_min:
                extra_score += 0.15; reason.append("cloud_floor")
            if extra.get("cloud_z_med") is not None and float(extra["cloud_z_med"]) < 0.4:
                extra_score += 0.1; reason.append("low_cloud")
            if float(extra.get("geom_flat", 0.0)) >= 0.55:
                extra_score += 0.1; reason.append("geom_flat")
            if extra.get("w_hs") is not None and float(extra["w_hs"]) < 0.4:
                extra_score += 0.05; reason.append("low_hs")
            if float(extra.get("prim_ffrac", 0.0)) >= self.floor_frac_min:
                extra_score += 0.05; reason.append("track_floor")
            conf = min(0.98, conf + extra_score)
        else:
            extra_score = 0.0
        extra_strong = extra_score >= self.extra_confirm_min   # enough to confirm without a cube

        # 2) cube second-check (the strong filter) — a red Fall normally REQUIRES it (rejects a
        #    dropped object: energy on the floor but no LIVING signal). Without a cube fetched,
        #    strong extra evidence can confirm instead (below); else the best we say is 'suspected'.
        #    ⭐ LIVING signal = RR *OR* micro-motion (user 2026-07-20): RR needs a visible chest and
        #    FAILS when the person is back-to-radar or occluded, but the floor ENERGY band + body
        #    micro-motion are still measurable. So energy-on-floor + (RR or micro) confirms a lying
        #    person; a dropped object has energy but NEITHER RR nor micro.
        cube_ok = None
        cube_no_vital = False
        if cube is not None:
            rr_ok = cube.get("rr") not in (None, 0)
            micro_ok = bool(cube.get("micro"))
            # ⭐ PRESENCE gate (Z≤40 / MUSIC floor-energy fraction, 思路B): is there a body-shaped
            # floor reflector? Breathing-INDEPENDENT + ghost/wall-multipath rejected upstream. This
            # REPLACES the old "energy AND vital" reject: NO floor body -> reject (ghost/empty), but a
            # floor body that ISN'T breathing is NOT rejected -- that IS the collapse emergency.
            body_present = float(cube.get("floor_frac", 0.0)) >= self.floor_frac_min
            if not body_present:
                return {"fall": False, "suspected": False, "collapse_suspect": False,
                        "confidence": 0.0, "trigger": trigger, "reason": reason,
                        "cleaned": "cube: no floor body (ghost/empty)"}
            cube_ok = True                                  # a real floor body IS present
            # LIVENESS now CLASSIFIES (red vs collapse), it does NOT gate. alive -> red fall; no vital
            # -> COLLAPSE-suspect: a tracked person went DOWN here (the trigger), so a no-vital floor
            # body is that person collapsed, NOT an inert object (the trigger context resolves it).
            cube_no_vital = not (rr_ok or micro_ok)
            conf = min(0.98, conf + 0.4)
            reason.append("cube-collapse" if cube_no_vital else ("cube" if rr_ok else "cube-micro"))
        # 2) geometry prior
        if geom is not None and geom.get("at_rest_spot"):
            conf *= 0.4; cleaned = "geom:rest-spot"
        # 3) persistence + confirmation -> confirmed fall (else at most suspected). Confirmation
        #    comes from EITHER the cube second-check OR, when no cube was fetched, strong server-side
        #    extra evidence. An explicit cube REJECTION (cube_ok is False) already returned above, so
        #    extra can never override the cube -- it only fills the gap when the cube is absent.
        # cube_ok True = fetched cube confirmed a living floor body. The cube-free path
        # (no cube fetched + strong extra evidence) is DISABLED when require_cube is set.
        confirmed = (cube_ok is True) or (not self.require_cube and cube_ok is None and extra_strong)
        if confirmed and cube_ok is None:
            reason.append("extra-confirm")
        fall = bool(trigger and self.run >= self.persist and confirmed and conf >= 0.5)
        # 💔 collapse-suspect = a CONFIRMED floor body with NO vital sign (fallen + not breathing =
        # the emergency). It IS a fall (red), not a reject; the server sustains/escalates it.
        collapse_suspect = bool(fall and cube_no_vital)
        suspected = bool(trigger and not fall)
        return {"fall": fall, "suspected": suspected, "collapse_suspect": collapse_suspect,
                "confidence": round(conf, 2), "trigger": trigger, "reason": reason, "cleaned": cleaned}
