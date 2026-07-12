"""Empirically test: does a mid-stream `rangeAntennaOutput <start> 64 1` move the
range-antenna window on the CURRENT firmware? (Prior notes claim it's ignored; today's
code read of mod/ shows no guard + fresh per-frame read -> settle it on hardware.)

Steps: boot profile_vitals_win64.cfg (start_bin 77) -> read start_bins (expect 77) ->
send `rangeAntennaOutput 100 64 1` mid-stream -> read start_bins (expect 100 if live).

    python verify_dynamic_window.py
"""
import os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spatial3d.uart_reader import RadarSession

CLI = "/dev/cu.usbmodem0000RA441"; DATA = "/dev/cu.usbmodem0000RA444"
# PROVEN-STABLE 44-bin cfg (tachy2 streamed 120s on it), booted with the window on the
# 3.3 m chair position via layer= so a wedge can't be blamed on bin count.
#   startBin 119 -> bins 119..162 = 2.79-3.80 m ; user chest at 3.3m (bin 141) is centered.
CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "profile_fall_20fps_near.cfg")
START0, START1, NBINS = 119, 100, 44        # boot at 3.3m, move-test to 100, then back


def read_start_bins(s, n, timeout=1.0, max_wait=8.0):
    """Collect up to n range-antenna start_bin values."""
    out = []; t0 = time.time()
    while len(out) < n and time.time() - t0 < max_wait:
        f = s.get_frame(timeout=timeout)
        if f is None:
            continue
        ra = f.range_antenna()
        if ra is not None:
            out.append((ra.start_bin, ra.num_bins))
    return out


def main():
    s = RadarSession(CLI, DATA); s.start_drain()

    # already streaming?
    live = read_start_bins(s, 5, max_wait=6.0)
    if len(live) < 3:
        print(f"not streaming ({len(live)} RA frames) -> sending cfg {os.path.basename(CFG)} "
              f"window@bin{START0} (3.3m)", flush=True)
        s.send_cfg(CFG, echo=False, layer=(START0, NBINS))
        # poll for RF warmup + a SUSTAINED stream (cold unit can need ~120s). Require a
        # solid run of frames, not just a couple, so a brief warmup blip isn't mistaken
        # for a live stream.
        print("waiting for sustained RA stream (up to 150s)...", flush=True)
        t0 = time.time(); total = 0; last = None
        while time.time() - t0 < 150:
            g = read_start_bins(s, 10, max_wait=5.0)
            total += len(g)
            if g:
                last = g[-1][0]
                print(f"  t={time.time()-t0:.0f}s +{len(g)} frames (total {total}) start_bin={last}", flush=True)
            if total >= 15:                          # sustained -> proceed
                break
        if total < 15:
            print(f"NO sustained stream ({total} frames in 150s) — likely wedged. Abort."); s.close(); return
    else:
        print(f"already streaming, start_bin={live[0]}", flush=True)

    # --- BEFORE: expect start_bin 77 ---
    before = read_start_bins(s, 15, max_wait=10.0)
    sb_before = [b for b, _ in before]
    print(f"\nBEFORE move: {len(before)} frames, start_bins={sb_before}")

    # --- MID-STREAM MOVE ---
    print(f"\n>>> sending mid-stream: rangeAntennaOutput {START1} {NBINS} 1", flush=True)
    reply = s.send_cli(f"rangeAntennaOutput {START1} {NBINS} 1", wait=2.0, echo=True)
    print(f"    CLI reply: {reply!r}")
    time.sleep(0.5)

    # --- AFTER: expect start_bin 100 if dynamic works ---
    after = read_start_bins(s, 20, max_wait=12.0)
    sb_after = [b for b, _ in after]
    print(f"\nAFTER move: {len(after)} frames, start_bins={sb_after}")

    moved = sb_after and all(b == START1 for b in sb_after[-5:])
    print("\n==================== VERDICT ====================")
    if moved:
        print(f"DYNAMIC WINDOW WORKS: start_bin {START0} -> {START1} live, no reboot.")
    elif sb_after and sb_after[-1] == START0:
        print(f"IGNORED mid-stream: start_bin stayed {START0} (needs firmware fix).")
    else:
        print(f"INCONCLUSIVE: before={set(sb_before)} after={set(sb_after)}")

    # leave the window back on the 3.3m chair position for a capture
    print(f"\nmoving window back to bin {START0} (3.3m) for capture ...", flush=True)
    s.send_cli(f"rangeAntennaOutput {START0} {NBINS} 1", wait=2.0, echo=True)
    back = read_start_bins(s, 8, max_wait=8.0)
    print(f"  now start_bins={[b for b, _ in back]} (sensor left running)")
    s.close(stop_sensor=False)


if __name__ == "__main__":
    main()
