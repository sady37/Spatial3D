"""Unit tests for FloorTracker (pure logic, no hardware/recording needed)."""

from falldet.floor_track import FloorTracker


def test_gtrack_association_and_continuity():
    ft = FloorTracker()
    # a live GTRACK track at (0,3); a floor cluster at the same spot -> that tid
    trs = ft.update(0.0, {5: (0.0, 3.0)}, [(0.05, 3.0, 30)])
    t = next(t for t in trs if t.person)
    assert t.id == 5 and t.source == "gtrack"
    # next frame, still there -> same id, seen grows
    trs = ft.update(0.1, {5: (0.0, 3.0)}, [(0.0, 3.0, 28)])
    assert [t.id for t in trs] == [5] and trs[0].seen == 2


def test_handoff_inherits_tid_when_track_dies_over_a_blob():
    ft = FloorTracker()
    ft.update(0.0, {2: (0.0, 4.0)}, [(0.0, 4.0, 40)])          # person standing, tracked
    # GTRACK drops the track (fall), but the floor blob is still there at the same spot
    trs = ft.update(0.1, {}, [(0.05, 4.0, 35)])
    t = trs[0]
    assert t.id == 2 and t.source == "inherited" and t.person   # SAME person, handed off


def test_walk_away_does_not_hand_off():
    ft = FloorTracker()
    ft.update(0.0, {2: (0.0, 4.0)}, [])                          # tracked, no floor blob
    # track dies AND there is no floor blob near it -> nothing to inherit
    trs = ft.update(0.1, {}, [])
    assert trs == []


def test_furniture_blob_is_not_a_person():
    ft = FloorTracker()
    # a low blob that never had a track and shows no RR -> furniture, not a person
    trs = ft.update(0.0, {}, [(2.0, 2.0, 20)], rr_at=lambda x, y: False)
    assert len(trs) == 1 and trs[0].id < 0 and not trs[0].person


def test_rr_promotes_a_trackless_blob_to_person():
    ft = FloorTracker()
    # someone already lying at start-up: no prior track, but the cube shows breathing
    trs = ft.update(0.0, {}, [(2.0, 2.0, 20)], rr_at=lambda x, y: True)
    assert trs[0].person and trs[0].source == "floor"


def test_dead_memory_expires():
    ft = FloorTracker(death_grace_s=1.0)
    ft.update(0.0, {2: (0.0, 4.0)}, [])
    ft.update(0.1, {}, [])                                       # track dies at t=0.1
    # a floor blob appears 2 s later -> too late to inherit, gets a fresh floor id
    trs = ft.update(2.2, {}, [(0.0, 4.0, 30)], rr_at=lambda x, y: False)
    assert trs[0].id < 0 and not trs[0].person
