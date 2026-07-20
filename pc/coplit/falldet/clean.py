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
    def __init__(self, mlp_trig=0.5, persist=10, floor_frac_min=0.7):
        self.mlp_trig = mlp_trig          # MLP falling/lying prob that counts as a trigger
        self.persist = persist            # frames the trigger must hold for a confirmed fall
        self.floor_frac_min = floor_frac_min
        self.run = 0

    def decide(self, window_out, mlp_out, cube=None, geom=None, extra=None):
        """window_out : {down, h_s}         from WindowDetector (may be None)
        mlp_out    : {pose, falling_p}    from MLPDetector    (may be None)
        cube       : {rr, floor_frac}     server 320 second-check (None if not fetched)
        geom       : {at_rest_spot: bool} location prior (None if unknown)
        extra      : server-side extra features (3001 cloud / floor energy / geometry)
        Returns {fall, suspected, confidence, trigger, reason, cleaned}."""
        w_down = bool(window_out and window_out.get("down"))
        m_fall = float(mlp_out.get("falling_p", 0.0)) if mlp_out else 0.0
        m_trig = m_fall >= self.mlp_trig
        trigger = w_down or m_trig
        self.run = self.run + 1 if trigger else 0

        conf, reason = 0.0, []
        if w_down: conf += 0.5; reason.append("window")            # sustained down-state
        if m_trig: conf += 0.3; reason.append(f"mlp:{mlp_out.get('pose')}")  # learned motion

        cleaned = None
        # 1) server-side 3001/floor/geometry features can boost the fusion confidence.
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

        # 2) cube second-check (the strong filter) — a red Fall REQUIRES it (rejects a
        #    dropped object: energy on the floor but NO breathing). Without a cube fetched,
        #    the best we can say is 'suspected'.
        cube_ok = None
        if cube is not None:
            rr_ok = cube.get("rr") not in (None, 0)
            energy_ok = float(cube.get("floor_frac", 0.0)) >= self.floor_frac_min
            cube_ok = bool(rr_ok and energy_ok)
            if not cube_ok:
                return {"fall": False, "suspected": False, "confidence": 0.0,
                        "trigger": trigger, "reason": reason,
                        "cleaned": "cube: no living body on floor"}
            conf = min(0.98, conf + 0.4); reason.append("cube")
        # 2) geometry prior
        if geom is not None and geom.get("at_rest_spot"):
            conf *= 0.4; cleaned = "geom:rest-spot"
        # 3) persistence + cube confirmation -> confirmed fall (else at most suspected)
        fall = bool(trigger and self.run >= self.persist and cube_ok is True and conf >= 0.5)
        suspected = bool(trigger and not fall)
        return {"fall": fall, "suspected": suspected, "confidence": round(conf, 2),
                "trigger": trigger, "reason": reason, "cleaned": cleaned}
