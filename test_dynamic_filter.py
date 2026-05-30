# test_dynamic_filter.py
"""
Test dynamic obstacle filtering for ICP localization.

Compares two ICP runs against a single home-position scan:
  A) Unfiltered — all scan points used, matched against survey_planning_base.png
  B) Filtered   — scan points near moveable obstacles removed,
                  matched against survey_map.png (fixed features only)

The moveable-obstacle mask is derived automatically as the pixel difference
between survey_planning_base.png and survey_map.png — no extra input needed.

Usage:
    python test_dynamic_filter.py

Requires icp_localizer.py in the same directory (imports scan_to_cartesian,
icp, and related helpers from it).

Produces:
    dynamic_filter_test.png  — side-by-side scan overlays showing which
                               points were kept vs filtered
"""

import sys
import json
import math
import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from PIL import Image
from sklearn.neighbors import KDTree
import paho.mqtt.client as mqtt

sys.path.insert(0, '../raspibot/robot')
from topics import Topics

# ---------------------------------------------------------------------------
# Configuration — must match icp_localizer.py and map_metadata.json
# ---------------------------------------------------------------------------
FIXED_MAP_FILE    = "survey_map.png"             # fixed features only
COMBINED_MAP_FILE = "survey_planning_base.png"   # fixed + moveable

RESOLUTION = 0.05
WIDTH_PX   = HEIGHT_PX = 300
ORIGIN_X   = -3.0
ORIGIN_Y   = -7.0

BROKER        = "192.168.1.85"
BROKER_PORT   = 1883
MQTT_USERNAME = "robot"
MQTT_PASSWORD = "robot"

OCCUPIED_THRESHOLD      = 128    # pixels darker than this = occupied
DYNAMIC_FILTER_RADIUS_M = 0.20   # metres — scan points within this distance
                                  # of a moveable obstacle pixel are excluded
MAX_SCAN_RANGE          = 5.0

# ---------------------------------------------------------------------------
# Coordinate helpers
# ---------------------------------------------------------------------------

def m_to_px(x_m, y_m):
    col = int((x_m - ORIGIN_X) / RESOLUTION)
    row = HEIGHT_PX - 1 - int((y_m - ORIGIN_Y) / RESOLUTION)
    return col, row


def px_to_m(col, row):
    x = ORIGIN_X + (col + 0.5) * RESOLUTION
    y = ORIGIN_Y + (HEIGHT_PX - 1 - row + 0.5) * RESOLUTION
    return x, y


# ---------------------------------------------------------------------------
# Map loading
# ---------------------------------------------------------------------------

def load_map_kdtree(png_file):
    """Load PNG map → occupied cell centres as (N,2) array + KDTree."""
    img = Image.open(png_file).convert("L")
    arr = np.flipud(np.array(img))
    rows, cols = np.where(arr < OCCUPIED_THRESHOLD)
    x = ORIGIN_X + (cols + 0.5) * RESOLUTION
    y = ORIGIN_Y + (rows + 0.5) * RESOLUTION
    pts = np.column_stack([x, y])
    print(f"  {png_file}: {len(pts)} occupied cells")
    return pts, KDTree(pts)


def load_moveable_mask():
    """
    Derive moveable-only pixels as the difference between the combined map
    and the fixed map.  Returns (N,2) array of map-frame points + KDTree.
    """
    fixed    = np.array(Image.open(FIXED_MAP_FILE).convert("L"))
    combined = np.array(Image.open(COMBINED_MAP_FILE).convert("L"))

    # Moveable pixels: occupied in combined but NOT in fixed
    fixed_occ    = fixed    < OCCUPIED_THRESHOLD
    combined_occ = combined < OCCUPIED_THRESHOLD
    moveable_occ = combined_occ & ~fixed_occ

    # Flip vertically for map-frame y
    moveable_occ = np.flipud(moveable_occ)
    rows, cols   = np.where(moveable_occ)
    x = ORIGIN_X + (cols + 0.5) * RESOLUTION
    y = ORIGIN_Y + (rows + 0.5) * RESOLUTION
    pts = np.column_stack([x, y])
    print(f"  Moveable mask: {len(pts)} pixels")
    return pts, KDTree(pts) if len(pts) > 0 else None


# ---------------------------------------------------------------------------
# Scan conversion
# ---------------------------------------------------------------------------

def scan_to_cartesian(scan):
    """Scan → robot-frame (x, y) array.  a=0 points east (+x = forward)."""
    pts = []
    for entry in scan:
        a = entry["a"]
        d = entry["d"]
        if 0.05 < d < MAX_SCAN_RANGE:
            pts.append([d * math.cos(a), d * math.sin(a)])
    return np.array(pts) if pts else np.zeros((0, 2))


def scan_to_map_frame(scan_pts, pose=(0.0, 0.0, 0.0)):
    """Transform robot-frame scan points to map frame using pose (x,y,h)."""
    x, y, h = pose
    c, s = math.cos(h), math.sin(h)
    R = np.array([[c, -s], [s, c]])
    return (R @ scan_pts.T).T + np.array([x, y])


# ---------------------------------------------------------------------------
# ICP
# ---------------------------------------------------------------------------

def icp(source, target_kdtree, target_pts,
        max_iterations=30, convergence_threshold=0.0005,
        max_correspondence_dist=0.30, min_points=20):
    """Point-to-point ICP. Returns (dx, dy, dtheta_rad, mean_err, converged)."""
    pts    = source.copy()
    cum_R  = np.eye(2)
    cum_t  = np.zeros(2)

    for _ in range(max_iterations):
        dists, idx = target_kdtree.query(pts, k=1)
        dists = dists.flatten()
        idx   = idx.flatten()
        mask  = dists < max_correspondence_dist

        if mask.sum() < min_points:
            return 0.0, 0.0, 0.0, float("inf"), False

        src = pts[mask]
        tgt = target_pts[idx[mask]]

        sc = src.mean(axis=0);  tc = tgt.mean(axis=0)
        H  = (src - sc).T @ (tgt - tc)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1] *= -1;  R = Vt.T @ U.T
        t = tc - R @ sc

        pts_new = (R @ pts.T).T + t
        cum_t   = R @ cum_t + t
        cum_R   = R @ cum_R

        if np.mean(np.linalg.norm(pts_new - pts, axis=1)) < convergence_threshold:
            pts = pts_new
            break
        pts = pts_new

    mean_err  = float(np.mean(dists[mask])) if mask.sum() > 0 else float("inf")
    dx        = float(cum_t[0])
    dy        = float(cum_t[1])
    dtheta    = float(math.atan2(cum_R[1, 0], cum_R[0, 0]))
    converged = mean_err < max_correspondence_dist
    return dx, dy, dtheta, mean_err, converged


# ---------------------------------------------------------------------------
# Grab one scan from MQTT
# ---------------------------------------------------------------------------

def grab_scan():
    received = threading.Event()
    hold     = [None]

    def on_msg(client, userdata, msg):
        hold[0] = json.loads(msg.payload.decode())
        received.set()

    client = mqtt.Client()
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.message_callback_add(Topics.LIDAR_SCAN, on_msg)
    client.connect(BROKER, BROKER_PORT, 60)
    client.loop_start()
    client.subscribe(Topics.LIDAR_SCAN)
    print("Waiting for scan from home position...")
    received.wait(timeout=5.0)
    client.loop_stop()
    client.disconnect()
    return hold[0]


# ---------------------------------------------------------------------------
# Main test
# ---------------------------------------------------------------------------

def run_test():
    print("\n=== Dynamic Filter Test ===\n")

    # Load maps
    print("Loading maps:")
    fixed_pts,    fixed_kd    = load_map_kdtree(FIXED_MAP_FILE)
    combined_pts, combined_kd = load_map_kdtree(COMBINED_MAP_FILE)
    moveable_pts, moveable_kd = load_moveable_mask()

    # Grab scan
    scan_raw = grab_scan()
    if not scan_raw:
        print("No scan received — is the scanner running?")
        return

    # Convert to robot frame then map frame (robot at home = 0,0,0)
    scan_robot = scan_to_cartesian(scan_raw)
    scan_map   = scan_to_map_frame(scan_robot)
    print(f"\nScan: {len(scan_map)} points after range filter")

    # Filter out points near moveable obstacles
    if moveable_kd is not None and len(moveable_pts) > 0:
        dists_to_moveable, _ = moveable_kd.query(scan_map, k=1)
        dists_to_moveable    = dists_to_moveable.flatten()
        keep_mask   = dists_to_moveable > DYNAMIC_FILTER_RADIUS_M
        filtered_map = scan_map[keep_mask]
        removed_map  = scan_map[~keep_mask]
        print(f"Filter: {keep_mask.sum()} kept, "
              f"{(~keep_mask).sum()} removed "
              f"({100*(~keep_mask).sum()/len(keep_mask):.1f}% of scan)")
    else:
        filtered_map = scan_map
        removed_map  = np.zeros((0, 2))
        print("No moveable mask — filter has no effect")

    # ------------------------------------------------------------------
    # Run A: unfiltered ICP against combined map
    # ------------------------------------------------------------------
    print("\nRun A — Unfiltered ICP vs combined map (survey_planning_base.png):")
    dx_a, dy_a, dth_a, err_a, conv_a = icp(scan_map, combined_kd, combined_pts)
    print(f"  {'OK' if conv_a else 'no-converge'}  "
          f"dx={dx_a:+.4f}m  dy={dy_a:+.4f}m  "
          f"dθ={math.degrees(dth_a):+.3f}°  err={err_a:.4f}m")

    # ------------------------------------------------------------------
    # Run B: filtered ICP against fixed map only
    # ------------------------------------------------------------------
    print("\nRun B — Filtered ICP vs fixed map only (survey_map.png):")
    if len(filtered_map) < 20:
        print("  Too few points after filtering — cannot run ICP")
        dx_b = dy_b = dth_b = err_b = 0.0
        conv_b = False
    else:
        dx_b, dy_b, dth_b, err_b, conv_b = icp(filtered_map, fixed_kd, fixed_pts)
        print(f"  {'OK' if conv_b else 'no-converge'}  "
              f"dx={dx_b:+.4f}m  dy={dy_b:+.4f}m  "
              f"dθ={math.degrees(dth_b):+.3f}°  err={err_b:.4f}m")

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------
    print(f"\n{'='*50}")
    print(f"  err improvement:  {err_a*100:.1f}cm → {err_b*100:.1f}cm  "
          f"({'better' if err_b < err_a else 'worse or same'})")
    print(f"  correction shift: "
          f"Δdx={abs(dx_b-dx_a)*100:.1f}cm  "
          f"Δdy={abs(dy_b-dy_a)*100:.1f}cm  "
          f"Δdθ={abs(math.degrees(dth_b-dth_a)):.2f}°")
    print(f"{'='*50}\n")

    # ------------------------------------------------------------------
    # Visualisation
    # ------------------------------------------------------------------
    fig, axes = plt.subplots(1, 2, figsize=(14, 7))
    fig.suptitle("Dynamic Filter Test — Home Position Scan", fontsize=13)

    map_img_fixed    = np.array(Image.open(FIXED_MAP_FILE).convert("L"))
    map_img_combined = np.array(Image.open(COMBINED_MAP_FILE).convert("L"))

    for ax, map_img, title, scan_keep, scan_remove, dx, dy, dth, err, label in [
        (axes[0], map_img_combined,
         f"A: Unfiltered vs combined map\nerr={err_a*100:.1f}cm",
         scan_map, np.zeros((0,2)), dx_a, dy_a, dth_a, err_a, "all points"),
        (axes[1], map_img_fixed,
         f"B: Filtered vs fixed map\nerr={err_b*100:.1f}cm",
         filtered_map, removed_map, dx_b, dy_b, dth_b, err_b, "kept points"),
    ]:
        ax.imshow(map_img, cmap="gray", origin="upper",
                  extent=[ORIGIN_X, ORIGIN_X + WIDTH_PX * RESOLUTION,
                          ORIGIN_Y, ORIGIN_Y + HEIGHT_PX * RESOLUTION])

        def to_map(pts):
            return pts[:, 0], pts[:, 1]

        if len(scan_keep):
            xs, ys = to_map(scan_keep)
            ax.scatter(xs, ys, s=3, c="red",    alpha=0.6, label="kept")
        if len(scan_remove):
            xs, ys = to_map(scan_remove)
            ax.scatter(xs, ys, s=3, c="orange", alpha=0.6, label="filtered out")

        # Home position
        ax.plot(0, 0, "b+", markersize=12, markeredgewidth=2, label="home (0,0)")

        ax.set_title(title)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.legend(fontsize=8, loc="upper right")
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.2)

    plt.tight_layout()
    plt.savefig("dynamic_filter_test.png", dpi=150, bbox_inches="tight")
    print("Saved dynamic_filter_test.png")
    plt.show()


if __name__ == "__main__":
    run_test()
