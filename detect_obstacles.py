#!/usr/bin/env python3
# detect_obstacles.py
"""
Unknown obstacle detection node.

Subscribes to lidar scan and localization pose. For each scan it transforms
points into map coordinates and checks them against the survey map. Points
that land in empty map space are collected as "unmatched" — potential
unknown obstacles.

At shutdown (Ctrl+C), runs DBSCAN clustering on the accumulated unmatched
points, fits bounding boxes to significant clusters, and writes a report
with candidate draw_filled_rect() calls ready to paste into build_map.py.

Usage:
    python detect_obstacles.py
    python detect_obstacles.py --broker 192.168.1.85
    python detect_obstacles.py --map survey_map.png --min-points 15 --min-scans 4

Output:
    obstacle_candidates_YYYYMMDD_HHMMSS.txt
"""

import sys
import json
import math
import time
import argparse
import threading
from datetime import datetime
from pathlib import Path

import numpy as np
from PIL import Image
from sklearn.cluster import DBSCAN
import paho.mqtt.client as mqtt

sys.path.insert(0, '../raspibot/robot')
from topics import Topics

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
DEFAULT_BROKER   = "raspibot.local"
BROKER_PORT      = 1883
MQTT_USERNAME    = "robot"
MQTT_PASSWORD    = "robot"

# Map parameters — must match icp_localizer.py
MAP_FILE         = "survey_map.png"
RESOLUTION       = 0.05          # metres per pixel
ORIGIN_X         = -3.0          # map origin in world frame (metres)
ORIGIN_Y         = -7.0

# A scan point is "matched" if any map pixel within this many pixels is
# occupied.  3 pixels = 15cm — accounts for small ICP residual errors and
# slight map inaccuracies without swallowing real obstacles.
MATCH_RADIUS_PX  = 3

# Minimum scan range to consider (ignore very close returns — robot body)
MIN_RANGE_M      = 0.15
MAX_RANGE_M      = 4.0

# DBSCAN clustering parameters
# DBSCAN_EPS: max distance (metres) between points in the same cluster
# DBSCAN_MIN_SAMPLES: min points to form a cluster core
DBSCAN_EPS         = 0.15        # 15cm — tighter than a typical obstacle
DBSCAN_MIN_SAMPLES = 5

# Reporting thresholds
MIN_CLUSTER_POINTS = 15          # ignore clusters smaller than this
MIN_CLUSTER_SCANS  = 4           # ignore clusters seen in fewer scans
CONFIDENCE_HIGH    = 40          # points threshold for HIGH confidence
CONFIDENCE_MED     = 20          # points threshold for MED confidence

# Minimum bounding box size — smaller than this is likely noise
MIN_BOX_M          = 0.08        # 8cm

# ---------------------------------------------------------------------------
# Map loading and match lookup
# ---------------------------------------------------------------------------

def load_map(map_file):
    """
    Load the survey map and build a fast binary occupancy array.
    Returns (occupancy_array, map_width_px, map_height_px).
    occupancy_array[row, col] = True if that pixel is occupied.
    """
    img = Image.open(map_file).convert("L")
    arr = np.array(img)
    # Occupied pixels are dark (< 128 in greyscale)
    occupied = arr < 128
    print(f"Map loaded: {img.width}×{img.height}px, "
          f"{occupied.sum()} occupied cells from '{map_file}'")
    return occupied, img.width, img.height


def world_to_pixel(x, y, map_w, map_h):
    """Convert world coordinates (metres) to pixel (col, row)."""
    col = int((x - ORIGIN_X) / RESOLUTION)
    row = map_h - 1 - int((y - ORIGIN_Y) / RESOLUTION)
    return col, row


def build_match_lookup(occupied, radius_px):
    """
    Pre-compute a dilated occupancy mask: a pixel is "near occupied" if
    any occupied pixel exists within radius_px.  Used for fast match
    checking without per-point distance loops.
    """
    from scipy.ndimage import binary_dilation
    struct = np.ones((2 * radius_px + 1, 2 * radius_px + 1), dtype=bool)
    dilated = binary_dilation(occupied, structure=struct)
    print(f"Match lookup built  (radius={radius_px}px = "
          f"{radius_px * RESOLUTION * 100:.0f}cm)")
    return dilated


def is_matched(col, row, match_lookup, map_w, map_h):
    """Return True if (col, row) is within MATCH_RADIUS_PX of an occupied cell."""
    if col < 0 or row < 0 or col >= map_w or row >= map_h:
        return True   # out of bounds = treat as matched (map boundary)
    return bool(match_lookup[row, col])


# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------

latest_pose  = None
latest_scan  = None
new_scan_ready = False   # set True in callback, cleared after processing
lock         = threading.Lock()

# Accumulated unmatched points: list of (x, y, scan_id)
unmatched_points = []
scan_counter     = 0


# ---------------------------------------------------------------------------
# Scan processing
# ---------------------------------------------------------------------------

def process_scan(scan, pose, match_lookup, map_w, map_h):
    """
    Transform scan points to world frame, check each against the map,
    collect unmatched points.  Returns list of (x, y) unmatched world points.
    """
    rx, ry, rh = pose["x"], pose["y"], pose["h"]
    new_unmatched = []

    for entry in scan:
        a = entry["a"]
        d = entry["d"]
        if d < MIN_RANGE_M or d > MAX_RANGE_M:
            continue

        # Scan point in robot frame
        sx = d * math.cos(a)
        sy = d * math.sin(a)

        # Rotate into map frame using robot heading
        wx = rx + sx * math.cos(rh) - sy * math.sin(rh)
        wy = ry + sx * math.sin(rh) + sy * math.cos(rh)

        # Check against map
        col, row = world_to_pixel(wx, wy, map_w, map_h)
        if not is_matched(col, row, match_lookup, map_w, map_h):
            new_unmatched.append((wx, wy))

    return new_unmatched


# ---------------------------------------------------------------------------
# MQTT callbacks
# ---------------------------------------------------------------------------

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        client.subscribe(Topics.LIDAR_SCAN)
        client.subscribe(Topics.POSE)
        print(f"Connected to broker.")
        print(f"  Listening: {Topics.LIDAR_SCAN}")
        print(f"             {Topics.POSE}")
        print(f"Accumulating unmatched scan points — press Ctrl+C to finish.\n")
    else:
        print(f"Connection failed rc={rc}")


def on_message(client, userdata, msg):
    global latest_pose, latest_scan, unmatched_points, scan_counter, new_scan_ready
    try:
        payload = json.loads(msg.payload.decode())
        with lock:
            if msg.topic == Topics.POSE:
                latest_pose = payload
            elif msg.topic == Topics.LIDAR_SCAN:
                latest_scan = payload
                new_scan_ready = True
    except Exception as e:
        print(f"Parse error: {e}")


# ---------------------------------------------------------------------------
# Clustering and reporting
# ---------------------------------------------------------------------------

def cluster_and_report(points_with_scan, output_file):
    """
    Run DBSCAN on accumulated unmatched points, fit bounding boxes,
    write report.
    points_with_scan: list of (x, y, scan_id)
    """
    if len(points_with_scan) < DBSCAN_MIN_SAMPLES:
        print("Not enough unmatched points to cluster.")
        return

    coords = np.array([(p[0], p[1]) for p in points_with_scan])
    scan_ids = np.array([p[2] for p in points_with_scan])

    print(f"\nClustering {len(coords)} unmatched points...")
    db = DBSCAN(eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES).fit(coords)
    labels = db.labels_

    n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
    n_noise    = (labels == -1).sum()
    print(f"  {n_clusters} clusters found, {n_noise} noise points discarded")

    candidates = []
    for label in sorted(set(labels)):
        if label == -1:
            continue
        mask       = labels == label
        cluster_xy = coords[mask]
        cluster_sc = scan_ids[mask]
        n_points   = mask.sum()
        n_scans    = len(set(cluster_sc))

        if n_points < MIN_CLUSTER_POINTS:
            continue
        if n_scans < MIN_CLUSTER_SCANS:
            continue

        x_min, y_min = cluster_xy.min(axis=0)
        x_max, y_max = cluster_xy.max(axis=0)
        cx = (x_min + x_max) / 2
        cy = (y_min + y_max) / 2

        # Skip tiny clusters (noise)
        if (x_max - x_min) < MIN_BOX_M and (y_max - y_min) < MIN_BOX_M:
            continue

        # Add a small margin around the bounding box (half a resolution cell)
        margin = RESOLUTION
        x_min -= margin; y_min -= margin
        x_max += margin; y_max += margin

        if n_points >= CONFIDENCE_HIGH:
            confidence = "HIGH"
        elif n_points >= CONFIDENCE_MED:
            confidence = "MED"
        else:
            confidence = "LOW"

        # Shrink box by one pixel (0.05m) per side to compensate for
        # algorithm inflation — matches the build_map.py convention of
        # drawing lines 1 pixel inside the obstacle surface.
        shrink = RESOLUTION
        x_min_out = round(x_min + shrink, 3)
        y_min_out = round(y_min + shrink, 3)
        x_max_out = round(x_max - shrink, 3)
        y_max_out = round(y_max - shrink, 3)

        candidates.append({
            "n_points":   int(n_points),
            "n_scans":    int(n_scans),
            "cx":         round(cx, 3),
            "cy":         round(cy, 3),
            "x_min":      x_min_out,
            "y_min":      y_min_out,
            "x_max":      x_max_out,
            "y_max":      y_max_out,
            "confidence": confidence,
        })

    # Sort by confidence then point count
    order = {"HIGH": 0, "MED": 1, "LOW": 2}
    candidates.sort(key=lambda c: (order[c["confidence"]], -c["n_points"]))

    # Write report
    now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    lines = []
    lines.append(f"Unknown obstacle candidates — {now_str}")
    lines.append(f"Map: {MAP_FILE}  Resolution: {RESOLUTION}m  "
                 f"Match radius: {MATCH_RADIUS_PX}px  "
                 f"Min points: {MIN_CLUSTER_POINTS}  "
                 f"Min scans: {MIN_CLUSTER_SCANS}")
    lines.append(f"Total unmatched points: {len(coords)}  "
                 f"Clusters reported: {len(candidates)}")
    lines.append("")

    if not candidates:
        lines.append("No candidates met the reporting thresholds.")
    else:
        for i, c in enumerate(candidates, 1):
            lines.append(f"Candidate {i}  "
                         f"(confidence {c['confidence']} — "
                         f"{c['n_points']} points, {c['n_scans']} scans)")
            lines.append(f"  Centre: ({c['cx']:.3f}, {c['cy']:.3f})")
            lines.append(f"  draw_filled_rect(draw, "
                         f"{c['x_min']:.3f}, {c['y_min']:.3f}, "
                         f"{c['x_max']:.3f}, {c['y_max']:.3f})")
            lines.append("")

    report = "\n".join(lines)
    print("\n" + report)

    with open(output_file, "w") as f:
        f.write(report)
    print(f"Report saved to {output_file}")


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def run(broker, map_file, min_points, min_scans):
    global MIN_CLUSTER_POINTS, MIN_CLUSTER_SCANS, unmatched_points, scan_counter
    global new_scan_ready

    MIN_CLUSTER_POINTS = min_points
    MIN_CLUSTER_SCANS  = min_scans

    # Load map
    occupied, map_w, map_h = load_map(map_file)
    match_lookup = build_match_lookup(occupied, MATCH_RADIUS_PX)

    # MQTT
    client = mqtt.Client()
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker, BROKER_PORT, 60)
    client.loop_start()

    print(f"Connecting to {broker}...")
    last_scan_time = 0.0
    last_status_t  = time.monotonic()

    try:
        while True:
            time.sleep(0.05)
            now = time.monotonic()

            with lock:
                scan  = latest_scan
                pose  = latest_pose
                ready = new_scan_ready
                if ready:
                    new_scan_ready = False   # consume the flag

            if scan is None or pose is None or not ready:
                continue

            new_pts = process_scan(scan, pose, match_lookup, map_w, map_h)
            if new_pts:
                scan_counter += 1
                for pt in new_pts:
                    unmatched_points.append((pt[0], pt[1], scan_counter))

            # Status update every 10 seconds
            if now - last_status_t >= 10.0:
                last_status_t = now
                print(f"  t={now:.0f}s  scans={scan_counter}  "
                      f"unmatched points={len(unmatched_points)}")

    except KeyboardInterrupt:
        print(f"\nStopped. {scan_counter} scans, "
              f"{len(unmatched_points)} unmatched points accumulated.")
    finally:
        client.loop_stop()
        client.disconnect()

    # Generate report
    output_file = f"obstacle_candidates_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
    with lock:
        pts = list(unmatched_points)

    cluster_and_report(pts, output_file)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Detect unknown obstacles from lidar vs survey map")
    parser.add_argument("--broker",     default=DEFAULT_BROKER)
    parser.add_argument("--map",        default=MAP_FILE,
                        help=f"Survey map file (default: {MAP_FILE})")
    parser.add_argument("--min-points", type=int, default=MIN_CLUSTER_POINTS,
                        help=f"Min cluster points to report (default: {MIN_CLUSTER_POINTS})")
    parser.add_argument("--min-scans",  type=int, default=MIN_CLUSTER_SCANS,
                        help=f"Min scans per cluster to report (default: {MIN_CLUSTER_SCANS})")
    args = parser.parse_args()

    run(args.broker, args.map, args.min_points, args.min_scans)
