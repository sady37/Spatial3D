"""Module 4 — FLOOR-CLUSTER TRACKER: give a fallen body a track_id when GTRACK won't.

GTRACK allocates tracks from MOVING points; a person who has fallen and lies STILL is a
zero-velocity blob that its allocator treats like furniture, so it either coasts the old
track a while or drops it and never re-allocates. That leaves the fallen person's 3001
point cloud with no track_id — the dropped-orphan blobs measured at 23–68% of clusters.

This is a light server-side tracker over the FLOOR-near clusters (world-z below the floor
band) that supplies the missing identity, keyed so the fall stays the SAME person:

  * GTRACK track nearby (widened gate, since a lying body spreads ~1.5 m)  -> that tid.
  * else an existing floor-track nearby                                     -> keep its id.
  * else a GTRACK track DIED near here recently                            -> INHERIT its
        tid ("track lost + floor blob at the same spot" = the person fell there — the
        handoff that keeps standing-person-2 == fallen-person-2, and that by construction
        does NOT fire when a track dies with no floor blob = the person walked away).
  * else                                                                    -> a fresh
        negative id (own namespace, never collides with GTRACK's positive tids).

Person vs furniture (both are low static blobs) is decided by TWO complementary signals,
OR'd — either alone is enough, together they cover the gaps:
  * inherited/associated a real GTRACK tid  (walked in, then went down), AND/OR
  * shows RR / breathing from the cube       (a living body on the floor breathes; catches
        someone already lying at start-up, whom no prior track covers).
A blob with neither — never tracked, not breathing — is furniture/clutter: shown dimmed,
never alerted. (RR is the same cube second-check that confirms the red Fall, doing double
duty.) See fall-detection-design / fall-modular-pipeline.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class FloorTrack:
    id: int                 # inherited GTRACK tid (>=0) or a fresh floor id (<0)
    x: float
    y: float
    n: int
    source: str             # 'gtrack' | 'inherited' | 'floor'
    seen: int = 1           # frames alive
    age: int = 0            # frames since last matched
    has_rr: bool = False    # breathing seen at this location

    @property
    def person(self) -> bool:
        # a real identity (from/inherited a GTRACK track) OR a breathing body
        return self.source in ("gtrack", "inherited") or self.has_rr


def _d2(ax, ay, bx, by):
    dx, dy = ax - bx, ay - by
    return dx * dx + dy * dy


class FloorTracker:
    """Frame-to-frame tracker for floor-near clusters. Pure logic; no server deps."""

    def __init__(self, assoc_gate_m=1.2, grace_frames=8, death_grace_s=5.0):
        self.gate2 = assoc_gate_m * assoc_gate_m
        self.grace_frames = grace_frames
        self.death_grace_s = death_grace_s
        self._tracks: list[FloorTrack] = []
        self._prev_gtids: dict[int, tuple[float, float]] = {}   # last-seen live GTRACK xy
        self._dead: dict[int, tuple[float, float, float]] = {}  # tid -> (x, y, death_time)
        self._next_id = -1

    def _fresh_id(self) -> int:
        i = self._next_id
        self._next_id -= 1
        return i

    def update(self, now, gtracks, clusters, rr_at=None):
        """Advance one frame.

        gtracks  : {tid: (x, y)}  live GTRACK targets this frame (tid >= 0)
        clusters : [(x, y, n)]    FLOOR-near clusters this frame (caller pre-filters by z)
        rr_at    : optional callable (x, y) -> bool, True if the cube shows RR near here
        Returns the current list of FloorTrack (person and furniture; filter on .person).
        """
        # --- 1. GTRACK death bookkeeping: tids present last frame, gone now ---------
        gset = set(gtracks)
        for tid, xy in self._prev_gtids.items():
            if tid not in gset and xy is not None:
                self._dead[tid] = (xy[0], xy[1], now)
        self._prev_gtids = {t: (x, y) for t, (x, y) in gtracks.items()}
        self._dead = {t: v for t, v in self._dead.items()
                      if now - v[2] < self.death_grace_s and t not in gset}

        # --- 2. age existing floor-tracks -------------------------------------------
        for tr in self._tracks:
            tr.age += 1
        used = set()

        for (cx, cy, cn) in clusters:
            chosen = None
            # a) a LIVE GTRACK track within the (widened) gate -> that person
            btid, bd = None, self.gate2
            for tid, (gx, gy) in gtracks.items():
                d = _d2(cx, cy, gx, gy)
                if d < bd:
                    bd, btid = d, tid
            if btid is not None:
                chosen = self._claim(btid, cx, cy, cn, "gtrack", used)
            else:
                # b) an existing floor-track nearby -> keep its identity
                bt, bd = None, self.gate2
                for i, tr in enumerate(self._tracks):
                    if i in used:
                        continue
                    d = _d2(cx, cy, tr.x, tr.y)
                    if d < bd:
                        bd, bt = d, i
                if bt is not None:
                    tr = self._tracks[bt]
                    tr.x, tr.y, tr.n, tr.age = cx, cy, cn, 0
                    tr.seen += 1
                    # not matched to a LIVE gtrack this frame (branch a missed): a real
                    # tid here means the person is coasting after GTRACK stopped = inherited.
                    tr.source = "inherited" if tr.id >= 0 else "floor"
                    used.add(bt)
                    chosen = tr
                else:
                    # c) a recently-DEAD GTRACK track nearby -> inherit its tid (handoff)
                    dtid, bd = None, self.gate2
                    for tid, (dx, dy, _t) in self._dead.items():
                        d = _d2(cx, cy, dx, dy)
                        if d < bd:
                            bd, dtid = d, tid
                    if dtid is not None:
                        self._dead.pop(dtid, None)
                        chosen = self._claim(dtid, cx, cy, cn, "inherited", used)
                    else:
                        # d) brand-new floor blob -> fresh negative id
                        chosen = FloorTrack(self._fresh_id(), cx, cy, cn, "floor")
                        self._tracks.append(chosen)
                        used.add(len(self._tracks) - 1)
            if rr_at is not None and rr_at(cx, cy):
                chosen.has_rr = True

        # --- 3. retire stale floor-tracks -------------------------------------------
        self._tracks = [t for t in self._tracks if t.age <= self.grace_frames]
        return list(self._tracks)

    def _claim(self, tid, cx, cy, cn, source, used):
        """Find/create the floor-track carrying `tid`, update it, mark its slot used."""
        for i, tr in enumerate(self._tracks):
            if tr.id == tid:
                tr.x, tr.y, tr.n, tr.age, tr.source = cx, cy, cn, 0, source
                tr.seen += 1
                used.add(i)
                return tr
        tr = FloorTrack(tid, cx, cy, cn, source)
        self._tracks.append(tr)
        used.add(len(self._tracks) - 1)
        return tr

    def persons(self):
        """Current floor-tracks judged to be people (identity or breathing)."""
        return [t for t in self._tracks if t.person]
