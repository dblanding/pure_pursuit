#!/usr/bin/env python3
# vfh_viz.py
"""
Real-time VFH polar histogram visualizer.

Subscribes to lidar scan, localization pose, and nav goal, then displays
a live two-panel figure showing:

  LEFT  — Polar plot: obstacle density radiating outward from robot centre,
           coloured by free (green) / blocked (red), with goal direction
           and chosen valley overlaid.

  RIGHT — Linear plot: histogram density vs angle, with obstacle threshold,
           valley regions highlighted, goal direction and chosen valley marked.

Useful for understanding why VFH steers the way it does in tight spaces.

Usage:
    python vfh_viz.py
    python vfh_viz.py --broker 192.168.1.85
"""

import sys
import json
import math
import threading
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
import paho.mqtt.client as mqtt

sys.path.insert(0, '../raspibot/robot')
from topics import Topics

# ---------------------------------------------------------------------------
# Configuration — must match obstacle_avoidance.py VFH settings
# ---------------------------------------------------------------------------
DEFAULT_BROKER     = "raspibot.local"
BROKER_PORT        = 1883
MQTT_USERNAME      = "robot"
MQTT_PASSWORD      = "robot"

N_HISTOGRAM_BINS   = 72          # 5° per bin
OBSTACLE_THRESHOLD = 3.0         # density above this = blocked
MIN_VALLEY_WIDTH   = 3           # bins
FORWARD_ARC_DEG    = 90.0        # ±45° detection arc
MAX_SCAN_RANGE     = 3.0
WIDTH_WEIGHT       = 0.02

UPDATE_MS          = 200         # plot refresh interval (ms)

# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------
latest_scan = None
latest_pose = None
latest_goal = None
lock        = threading.Lock()


# ---------------------------------------------------------------------------
# VFH logic (mirrors obstacle_avoidance.py)
# ---------------------------------------------------------------------------

def scan_to_robot_frame(scan):
    pts = []
    for entry in scan:
        a = entry["a"]
        d = entry["d"]
        if 0.05 < d < MAX_SCAN_RANGE:
            pts.append((a, d))
    return pts


def build_histogram(pts):
    hist     = np.zeros(N_HISTOGRAM_BINS)
    bin_size = 2 * math.pi / N_HISTOGRAM_BINS
    for bearing, dist in pts:
        b   = bearing % (2 * math.pi)
        idx = int(b / bin_size) % N_HISTOGRAM_BINS
        hist[idx] += 1.0 / (dist * dist)
    return hist


def find_valleys(hist):
    n     = len(hist)
    free  = hist < OBSTACLE_THRESHOLD
    free2 = np.concatenate([free, free])
    valleys = []
    i = 0
    while i < n:
        if free2[i]:
            j = i
            while j < i + n and free2[j]:
                j += 1
            width = j - i
            if width >= MIN_VALLEY_WIDTH:
                centre_bin   = (i + width // 2) % n
                centre_angle = (centre_bin + 0.5) * (2 * math.pi / n)
                if centre_angle > math.pi:
                    centre_angle -= 2 * math.pi
                # Also record start/end bin for shading
                valleys.append({
                    "angle": centre_angle,
                    "width": width,
                    "start_bin": i % n,
                    "end_bin":   (i + width - 1) % n,
                })
            i = j
        else:
            i += 1
    return valleys


def best_valley(valleys, preferred):
    if not valleys:
        return None
    def score(v):
        diff = abs(math.atan2(math.sin(v["angle"] - preferred),
                              math.cos(v["angle"] - preferred)))
        return diff - WIDTH_WEIGHT * v["width"]
    return min(valleys, key=score)


def goal_bearing_robot_frame(pose, goal):
    dx = goal["x"] - pose["x"]
    dy = goal["y"] - pose["y"]
    map_bearing   = math.atan2(dy, dx)
    robot_bearing = map_bearing - pose["h"]
    return math.atan2(math.sin(robot_bearing), math.cos(robot_bearing))


# ---------------------------------------------------------------------------
# MQTT
# ---------------------------------------------------------------------------

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        client.subscribe(Topics.LIDAR_SCAN)
        client.subscribe(Topics.POSE)
        client.subscribe(Topics.NAV_GOAL)
        print(f"Connected. Listening on scan / pose / goal topics.")
    else:
        print(f"Connection failed rc={rc}")


def on_message(client, userdata, msg):
    global latest_scan, latest_pose, latest_goal
    try:
        payload = json.loads(msg.payload.decode())
        with lock:
            if msg.topic == Topics.LIDAR_SCAN:
                latest_scan = payload
            elif msg.topic == Topics.POSE:
                latest_pose = payload
            elif msg.topic == Topics.NAV_GOAL:
                latest_goal = payload
    except Exception as e:
        print(f"Parse error: {e}")


# ---------------------------------------------------------------------------
# Plot setup
# ---------------------------------------------------------------------------

def make_figure():
    fig = plt.figure(figsize=(14, 6))
    fig.patch.set_facecolor("#1a1a2e")

    ax_polar  = fig.add_subplot(121, projection="polar")
    ax_linear = fig.add_subplot(122)

    for ax in [ax_polar, ax_linear]:
        ax.set_facecolor("#16213e")

    fig.suptitle("VFH Histogram Visualizer", color="white", fontsize=13)
    return fig, ax_polar, ax_linear


def update(frame, ax_polar, ax_linear, client_ref):
    with lock:
        scan = latest_scan
        pose = latest_pose
        goal = latest_goal

    ax_polar.cla()
    ax_linear.cla()

    # Style
    ax_polar.set_facecolor("#16213e")
    ax_linear.set_facecolor("#16213e")

    bin_size_rad = 2 * math.pi / N_HISTOGRAM_BINS
    bin_angles   = np.array([(i + 0.5) * bin_size_rad
                              for i in range(N_HISTOGRAM_BINS)])
    # Convert to [-π, π]
    bin_angles_pm = np.where(bin_angles > math.pi,
                             bin_angles - 2 * math.pi, bin_angles)
    lin_angles_deg = np.degrees(bin_angles_pm)

    if scan is None:
        ax_linear.text(0.5, 0.5, "Waiting for scan...",
                       transform=ax_linear.transAxes,
                       ha="center", va="center", color="white", fontsize=12)
        ax_polar.set_title("Waiting for scan...", color="white", pad=10)
        return

    pts  = scan_to_robot_frame(scan)
    hist = build_histogram(pts)

    # Compute valleys and best valley
    valleys = find_valleys(hist)
    preferred = 0.0
    if pose and goal:
        preferred = goal_bearing_robot_frame(pose, goal)
    chosen = best_valley(valleys, preferred)

    # ------------------------------------------------------------------
    # LEFT: Polar plot — math convention, CCW positive, 0° = forward (E)
    # Matches the linear histogram so left/right are consistent in both.
    # ------------------------------------------------------------------
    ax_polar.set_theta_zero_location("E")   # 0° = forward = East
    ax_polar.set_theta_direction(1)          # CCW positive = left = consistent with linear
    ax_polar.set_title("Polar Histogram  (0°=fwd, +CCW=left, −CW=right)",
                        color="white", pad=10, fontsize=9)
    ax_polar.tick_params(colors="gray")

    # Colour each bin: red=blocked, green=free, blue=chosen valley
    for i in range(N_HISTOGRAM_BINS):
        angle     = bin_angles[i]           # already in math convention (CCW from 0=fwd)
        density   = hist[i]
        bar_height = min(density, OBSTACLE_THRESHOLD * 3)

        if chosen and _bin_in_valley(i, chosen):
            colour = "#00bfff"       # chosen valley — bright blue
        elif density >= OBSTACLE_THRESHOLD:
            colour = "#ff4444"       # blocked — red
        elif _bin_in_any_valley(i, valleys):
            colour = "#44ff88"       # navigable valley — green
        else:
            colour = "#888888"       # below threshold but not a wide valley

        ax_polar.bar(angle, bar_height, width=bin_size_rad * 0.9,
                     color=colour, alpha=0.75, bottom=0)

    # Threshold ring
    theta_ring = np.linspace(0, 2 * math.pi, 200)
    ax_polar.plot(theta_ring,
                  np.full_like(theta_ring, OBSTACLE_THRESHOLD),
                  "w--", linewidth=1, alpha=0.4)

    # Goal direction — no conversion needed, angle is already in math convention
    if pose and goal:
        ax_polar.annotate("", xy=(preferred, OBSTACLE_THRESHOLD * 2.5),
                          xytext=(0, 0),
                          arrowprops=dict(arrowstyle="->", color="#ff9900", lw=2))
        ax_polar.text(preferred, OBSTACLE_THRESHOLD * 2.9, "goal",
                      color="#ff9900", ha="center", fontsize=8)

    # Chosen valley direction
    if chosen:
        ax_polar.annotate("", xy=(chosen["angle"], OBSTACLE_THRESHOLD * 2.5),
                          xytext=(0, 0),
                          arrowprops=dict(arrowstyle="->", color="#00bfff", lw=2))

    # Forward arc boundary lines
    half = math.radians(FORWARD_ARC_DEG / 2)
    for ang in [-half, half]:
        ax_polar.plot([ang, ang], [0, OBSTACLE_THRESHOLD * 3.2],
                      color="yellow", linewidth=0.8, alpha=0.4, linestyle=":")

    # Label cardinal directions in robot frame
    for label, angle in [("fwd", 0), ("left", math.pi/2),
                          ("back", math.pi), ("right", -math.pi/2)]:
        ax_polar.text(angle, OBSTACLE_THRESHOLD * 3.4, label,
                      color="gray", ha="center", va="center", fontsize=7)

    ax_polar.set_ylim(0, OBSTACLE_THRESHOLD * 3.7)
    ax_polar.set_yticklabels([])

    # ------------------------------------------------------------------
    # RIGHT: Linear plot
    # ------------------------------------------------------------------
    ax_linear.set_facecolor("#16213e")
    ax_linear.tick_params(colors="gray")
    for spine in ax_linear.spines.values():
        spine.set_edgecolor("gray")

    # Shade valley regions
    for v in valleys:
        v_start = _bin_to_deg(v["start_bin"])
        v_end   = _bin_to_deg(v["end_bin"])
        colour  = "#00bfff" if (chosen and v is chosen) else "#44ff88"
        # Handle wrap-around
        if v_start <= v_end:
            ax_linear.axvspan(v_start, v_end, alpha=0.15, color=colour)
        else:
            ax_linear.axvspan(-180, v_end,   alpha=0.15, color=colour)
            ax_linear.axvspan(v_start, 180,  alpha=0.15, color=colour)

    # Histogram bars
    bar_colours = []
    for i, d in enumerate(hist):
        if chosen and _bin_in_valley(i, chosen):
            bar_colours.append("#00bfff")
        elif d >= OBSTACLE_THRESHOLD:
            bar_colours.append("#ff4444")
        elif _bin_in_any_valley(i, valleys):
            bar_colours.append("#44ff88")
        else:
            bar_colours.append("#555577")

    bar_width = 360 / N_HISTOGRAM_BINS * 0.85
    ax_linear.bar(lin_angles_deg, hist, width=bar_width,
                  color=bar_colours, alpha=0.8)

    # Threshold line
    ax_linear.axhline(OBSTACLE_THRESHOLD, color="white", linewidth=1.2,
                      linestyle="--", alpha=0.6,
                      label=f"threshold = {OBSTACLE_THRESHOLD}")

    # Forward arc boundaries
    ax_linear.axvline(-FORWARD_ARC_DEG / 2, color="yellow",
                      linewidth=1, linestyle=":", alpha=0.5,
                      label=f"arc ±{FORWARD_ARC_DEG/2:.0f}°")
    ax_linear.axvline( FORWARD_ARC_DEG / 2, color="yellow",
                       linewidth=1, linestyle=":", alpha=0.5)

    # Goal direction
    if pose and goal:
        ax_linear.axvline(math.degrees(preferred), color="#ff9900",
                          linewidth=2, label="goal direction")

    # Chosen valley
    if chosen:
        ax_linear.axvline(math.degrees(chosen["angle"]), color="#00bfff",
                          linewidth=2, linestyle="-.",
                          label=f"chosen valley ({math.degrees(chosen['angle']):.1f}°, "
                                f"w={chosen['width']}bins)")

    ax_linear.set_xlabel("Angle (°, 0=forward)", color="gray")
    ax_linear.set_ylabel("Obstacle density", color="gray")
    ax_linear.set_title("Linear Histogram", color="white")
    ax_linear.set_xlim(-180, 180)
    ax_linear.set_ylim(0, max(hist.max() * 1.1, OBSTACLE_THRESHOLD * 1.5))
    ax_linear.set_xticks(range(-180, 181, 30))
    ax_linear.tick_params(colors="gray")

    legend = ax_linear.legend(loc="upper right", fontsize=8,
                               facecolor="#1a1a2e", edgecolor="gray",
                               labelcolor="white")

    # Status text
    n_valleys = len(valleys)
    status = (f"Valleys: {n_valleys}  |  "
              f"Goal: {math.degrees(preferred):+.1f}°  |  "
              + (f"Best: {math.degrees(chosen['angle']):+.1f}°  w={chosen['width']}bins"
                 if chosen else "No navigable valley"))
    ax_linear.text(0.01, 0.97, status,
                   transform=ax_linear.transAxes,
                   color="white", fontsize=9, va="top",
                   bbox=dict(boxstyle="round", facecolor="#1a1a2e",
                             edgecolor="gray", alpha=0.8))

    plt.tight_layout()


# ---------------------------------------------------------------------------
# Bin helper functions
# ---------------------------------------------------------------------------

def _bin_to_deg(bin_idx):
    angle = (bin_idx + 0.5) * 360 / N_HISTOGRAM_BINS
    if angle > 180:
        angle -= 360
    return angle


def _bin_in_valley(bin_idx, valley):
    """Check if bin_idx falls within the given valley."""
    n     = N_HISTOGRAM_BINS
    start = valley["start_bin"]
    end   = valley["end_bin"]
    if start <= end:
        return start <= bin_idx <= end
    else:  # wraps around
        return bin_idx >= start or bin_idx <= end


def _bin_in_any_valley(bin_idx, valleys):
    return any(_bin_in_valley(bin_idx, v) for v in valleys)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="VFH histogram visualizer")
    parser.add_argument("--broker", default=DEFAULT_BROKER,
                        help=f"MQTT broker (default: {DEFAULT_BROKER})")
    args = parser.parse_args()

    # Start MQTT
    client = mqtt.Client()
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(args.broker, BROKER_PORT, 60)
    client.loop_start()

    # Start live plot
    fig, ax_polar, ax_linear = make_figure()
    ani = FuncAnimation(fig, update,
                        fargs=(ax_polar, ax_linear, client),
                        interval=UPDATE_MS, cache_frame_data=False)
    plt.show()

    client.loop_stop()
    client.disconnect()
    print("VFH visualizer stopped.")
