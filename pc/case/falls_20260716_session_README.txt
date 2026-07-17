Fall test recordings — 20260716/17 night session (cubeQuery firmware live + falldet server).
All single-person real falls; new firmware (people_tracking_6844_CUBEQUERY, boundaryBox -2.3,
sensorPosition 0 0 2 0 -25 = mount 2m/tilt 25 down). Geometry: world wz = 2 + z*cos25 - y*sin25.

fall_320_trackcube_survived_2315.npz   — fall, track SURVIVES the floor (boundaryBox -2.3),
    trackcube 320 fires during the lie (bins 20-30). First clean track-survives-fall.
fall_fallcfg_2940cube_2330.npz          — fall cfg (enable 1): 2940 320-entries, heavy burst
    through the lie. 320 breathing verified at bins 29-44 (median 35).
fall2x_0cube_deadlock_2350.npz          — 2 real falls (6-18s, 27-45s) but 0 320: proves the
    recent-320-bin SEED DEADLOCK (no prior 320 -> no cubeQuery -> never any 320). Negative.
fall_wyfix_bootstrap320_0000.npz        — after the wy fix: cubeQuery bootstraps from the cloud
    GROUND range (wy), 1260 320-entries even with no native 320. Deadlock broken.
fall_endtoend_red_0015.npz              — FULL pipeline: window down -> real-person gate ->
    cubeQuery @wy bin 35-37 (stable) -> 6s cube -> RR (~13-22 bpm) -> red Fall; clears on stand.

KEY CALIBRATION (verified on these): 320 fires at the WORLD GROUND range wy (= py*cos+pz*sin),
NOT the slant (~1m farther) and NOT the track. Cube RR from _rr_from_cube (per-bin ant0 slow-
time FFT 0.15-0.5Hz). See [[fall-detection-design]] [[fall-modular-pipeline]].
