"""Live posture view — a PURE DISPLAY CONSUMER of radar_server's /api/scene.

The single serial owner is radar_server.py (the hub); this window just polls its
HTTP /api/scene, so it coexists with the browser dashboard and any number of
other viewers. It does NOT touch the serial port.

    # 1) start the hub (owns the serial, records + serves):
    python web/radar_server.py live --mount 2.5
    # 2) this matplotlib view (as many as you like, alongside the browser):
    python live_view.py
    python live_view.py --url http://127.0.0.1:8765/api/scene

Left  : top-down radar plane, radar on TOP centre, range increasing downward.
Right : height bar (cm): <=40 Fall/SitGround (red), 40..79 Sit (amber), >=80 Stand.
"""
import argparse, json, time
from urllib.request import urlopen
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation

ap = argparse.ArgumentParser()
ap.add_argument("--url", default="http://127.0.0.1:8765/api/scene")
ap.add_argument("--xlim", type=float, default=3.0, help="plane +/- X limit, m")
ap.add_argument("--ymax", type=float, default=5.0, help="plane Y (range) max, m")
a = ap.parse_args()

ZONES = [(0, 40, "Fall/Ground", "#e03131"),
         (40, 80, "Sit", "#f08c00"),
         (80, 200, "Stand", "#2f9e44")]

def pose_col(p):
    return {"fall": "#e03131", "sit": "#f08c00", "stand": "#2f9e44"}.get(p, "#868e96")

def fetch():
    try:
        with urlopen(a.url, timeout=1.0) as r:
            return json.load(r)
    except Exception:
        return None

# ---- figure ----
fig = plt.figure(figsize=(10, 6))
gs = fig.add_gridspec(1, 2, width_ratios=[4, 1], wspace=0.3)
axp = fig.add_subplot(gs[0]); axh = fig.add_subplot(gs[1])

axp.set_title("radar plane (radar on top, range down)")
axp.set_xlabel("X  left/right (m)")
axp.set_xlim(a.xlim, -a.xlim); axp.set_ylim(a.ymax, 0)   # radar (0) at TOP; X flipped L/R
axp.set_aspect("equal", adjustable="box"); axp.grid(True, ls=":", alpha=0.4)
axp.plot(0, 0, marker="v", ms=14, color="#1971c2", zorder=5)
axp.annotate("RADAR", (0, 0), textcoords="offset points", xytext=(8, 6),
             color="#1971c2", fontsize=8)
scat = axp.scatter([], [], s=30, c="#2f9e44", alpha=0.55, zorder=3)
trk = axp.scatter([], [], s=320, marker="x", c="#e03131", linewidths=3, zorder=6)

axh.set_title("height", fontsize=10)
axh.set_xlim(0, 1); axh.set_ylim(0, 200); axh.set_xticks([]); axh.set_ylabel("cm")
for lo, hi, lbl, col in ZONES:
    axh.add_patch(Rectangle((0, lo), 1, hi - lo, color=col, alpha=0.28))
    axh.text(0.5, (lo + min(hi, 200)) / 2, lbl, ha="center", va="center", fontsize=8)
hline = axh.axhline(-10, color="k", lw=2.5)
htext = axh.text(0.5, 190, "", ha="center", fontsize=9, weight="bold")
status = fig.suptitle("connecting to hub...", fontsize=11)

def update(_):
    sc = fetch()
    if not sc or not sc.get("live"):
        status.set_text(f"no hub at {a.url}  (start: radar_server.py live)")
        scat.set_offsets(np.empty((0, 2))); trk.set_offsets(np.empty((0, 2)))
        hline.set_ydata([-10, -10]); htext.set_text("")
        return scat, trk, hline, htext, status
    pts = sc.get("points") or []
    tg = sc.get("targets") or []
    col = pose_col(sc.get("posture"))
    scat.set_offsets(np.array([[p[0], p[1]] for p in pts]) if pts else np.empty((0, 2)))
    trk.set_offsets(np.array([[t["x"], t["y"]] for t in tg]) if tg else np.empty((0, 2)))
    trk.set_color(col)
    h = sc.get("height_cm")
    if h is not None:
        lbl = sc.get("posture", "").upper()
        hline.set_ydata([h, h]); hline.set_color(col)
        htext.set_text(f"{h:.0f}cm\n{lbl}"); htext.set_color(col)
    else:
        hline.set_ydata([-10, -10]); htext.set_text("")
    status.set_text(f"points:{len(pts)}  tracks:{len(tg)}  "
                    f"height:{h if h is not None else '—'}cm  {sc.get('posture') or '(no target)'}"
                    f"  (mount {sc.get('mount_cm')}cm, age {sc.get('age_s')}s)")
    return scat, trk, hline, htext, status

ani = FuncAnimation(fig, update, interval=300, blit=False, cache_frame_data=False)
plt.show()
