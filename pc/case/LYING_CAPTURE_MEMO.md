# Full-room LYING capture — to validate lying_detect (2026-07-21)

Goal: a full-room (63-bin, 0.1–6.7m) capture of a person **lying in the open floor**,
same room/setup as `empty_20260721.npz` (which is the baseline). This closes the
range-intensity lying-detection loop (all existing lie/fall captures are narrow
3.5–4.5m vital windows — body can't spread over range there).

## Prep
1. Firmware **pose65s must be streaming** — if ports are up but 0 bytes (dead
   firmware), **power-cycle** the radar and reload the healthy demo.
2. **Close the live display** so the DATA port (`...RA444`) is free — room_capture
   ATTACHES to the stream, it does NOT resend cfg.

## Capture (person lies still ~1 min each)
Command (venv, from `pc/`):
```
.venv/bin/python3 -m spatial3d.room_capture lie_floor
```
→ writes `case/lie_floor_20260721.npz` (30s track/point-cloud phase + cube sweep, ~2 min).

Lie in the **OPEN FLOOR** (room centre, NOT behind furniture). Orientation matters:
- **RADIAL** (head→toe pointing toward/away from the radar): body spreads over
  **range bins** → this is the case that tests the range-span discriminator. **Do this one.**
- optional TANGENTIAL (lying across): spreads over azimuth instead.

Optional same-spot references for contrast (same command, new label):
```
.venv/bin/python3 -m spatial3d.room_capture stand_floor   # standing at that spot
.venv/bin/python3 -m spatial3d.room_capture sit_floor      # sitting at that spot
```

Notes:
- A lying person may not get a firmware TRACK (person_xyz → nan) — that's fine, the
  CUBE still records and that's what lying_detect uses. For ground truth you can
  stand briefly (gets tracked) then lie.
- Output name is hardcoded `_20260721` in room_capture.py line 39 — rename the file
  if captured on another day.

## Analyze
```
.venv/bin/python3 -m spatial3d.lying_detect empty_20260721 lie_floor_20260721 stand_floor_20260721 sit_floor_20260721
```
Expect: **lying** shows a long contiguous run of elevated range bins (high
`lying_score = span_m × excess`), clearly above sit/stand (compact in range).
Reference on 0721: standing raised its bin ~+9 dB, seated ~+3 dB.
