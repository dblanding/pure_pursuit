# icp_localizer.py
"""
ICP (Iterative Closest Point) scan-matching localizer.

Runs on the LAPTOP, connecting to the MQTT broker on the robot (192.168.1.85).
Subscribes to lidar scans and odometry published by the robot, periodically
runs ICP to correct odometry drift, and publishes the correction transform
and corrected map-frame pose back to the broker.

Architecture
------------
Subscribes to:
    Topics.LIDAR_SCAN   raw lidar scan  (list of {"a": rad CCW, "d": m, "t": sec})
    Topics.ODOM_POSE    odometry pose   {"x", "y", "h", "t", "xr", "yr", "hr"}

Publishes to:
    Topics.CORRECTION   map->odom correction  {"dx", "dy", "dtheta_deg"}
    Topics.POSE         corrected map-frame pose {"x", "y", "h"}

Note on scan angle convention
------------------------------
scanner.py negates the raw RPLidar angle before publishing, so "a" in the
MQTT message is already in the CCW-positive math convention. No further
negation is needed here.

Note on odometry heading
------------------------
odometer.py uses "h" (not "theta") for heading, in radians.
All internal pose dicts here use "h" to match.

Map parameters
--------------
Set MAP_FILE, MAP_RESOLUTION, MAP_ORIGIN_X, MAP_ORIGIN_Y to match your
map_metadata.json before running.
"""

import sys
import csv
import json
import math
import time
import threading
from datetime import datetime
import numpy as np
import paho.mqtt.client as mqtt
from sklearn.neighbors import KDTree

# Laptop needs topics.py from the robot source tree
sys.path.insert(0, '../raspibot/robot')
from topics import Topics

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

# Map — fill in from your map_metadata.json
MAP_FILE        = "survey_map.png"
MAP_RESOLUTION  = 0.05              # metres per pixel
MAP_ORIGIN_X    = -3.0              # map-frame x of image bottom-left corner
MAP_ORIGIN_Y    = -7.0              # map-frame y of image bottom-left corner

# ICP tuning
MAX_ITERATIONS          = 30
CONVERGENCE_THRESHOLD   = 0.0005    # metres; stop when mean point shift < this
MAX_CORRESPONDENCE_DIST = 0.30      # metres; discard point pairs further apart
MIN_POINTS_FOR_ICP      = 20        # skip ICP if scan yields fewer points
MAX_SCAN_RANGE          = 5.0       # metres; ignore long-range returns
MAX_CORRECTION_JUMP     = 0.50      # metres; reject implausibly large ICP results
MAX_CORRECTION_ROT_DEG  = 10.0      # degrees; same for rotation

# Correction blending — smooths out frame-to-frame jitter.
# 1.0 = accept each ICP result immediately (fastest but noisiest)
# 0.2 = heavily smoothed (slower to respond but stable)
CORRECTION_ALPHA        = 0.3

# How often to run ICP (seconds)
UPDATE_INTERVAL_SEC     = 3.0

# CSV log file — written alongside this script
# Each row: elapsed_s, odom_x, odom_y, odom_h, map_x, map_y, map_h,
#            corr_dx, corr_dy, corr_dtheta_deg, icp_err, cycle_ms
LOG_FILE = f"log_files/icp_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
LOG_HEADER = ["elapsed_s", "odom_x", "odom_y", "odom_h_deg",
               "map_x", "map_y", "map_h_deg",
               "corr_dx", "corr_dy", "corr_dtheta_deg",
               "icp_err", "cycle_ms"]

# MQTT — broker is on the robot
BROKER_ADDRESS  = "192.168.1.85"
BROKER_PORT     = 1883
MQTT_USERNAME   = "robot"
MQTT_PASSWORD   = "robot"

TOPIC_SCAN       = Topics.LIDAR_SCAN
TOPIC_ODOM       = Topics.ODOM_POSE
TOPIC_CORRECTION = Topics.CORRECTION
TOPIC_POSE       = Topics.POSE


# ---------------------------------------------------------------------------
# Map loading
# ---------------------------------------------------------------------------

def load_map_pointcloud(map_file, resolution, origin_x, origin_y,
                        occupied_threshold=50):
    """
    Load an occupancy grid PNG and return occupied cells as an (N,2) array
    of map-frame (x, y) coordinates.

    Pixels darker than occupied_threshold are treated as occupied walls.
    The image bottom row corresponds to the lowest map-frame y value.
    """
    try:
        from PIL import Image
    except ImportError:
        raise ImportError("Pillow required:  pip install Pillow")

    img = Image.open(map_file).convert("L")
    arr = np.flipud(np.array(img))          # flip so row 0 = south (low y)
    rows, cols = np.where(arr < occupied_threshold)
    x = origin_x + (cols + 0.5) * resolution
    y = origin_y + (rows + 0.5) * resolution
    pts = np.column_stack([x, y])
    print(f"Map loaded: {len(pts)} occupied cells from '{map_file}'")
    return pts


# ---------------------------------------------------------------------------
# Scan conversion
# ---------------------------------------------------------------------------

def scan_to_cartesian(scan, max_range=MAX_SCAN_RANGE):
    """
    Convert published scan to robot-frame Cartesian (x, y) array.

    scanner.py already negates the raw RPLidar angle, so "a" is CCW-positive.
    Robot frame:  x = forward (east), y = leftward (north).
    a=0 points east (+x); standard polar convention.
    """
    pts = []
    for entry in scan:
        a = entry["a"]              # already CCW — no negation needed
        d = entry["d"]
        if 0.05 < d < max_range:
            pts.append([d * math.cos(a), d * math.sin(a)])
    return np.array(pts) if pts else np.zeros((0, 2))


# ---------------------------------------------------------------------------
# 2D rigid-transform helpers
# ---------------------------------------------------------------------------

def rot2(theta):
    """2×2 rotation matrix, theta in radians (CCW positive)."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def apply_transform(pts, dx, dy, dtheta):
    """Apply rigid transform (dx, dy, dtheta radians) to (N,2) array."""
    return (rot2(dtheta) @ pts.T).T + np.array([dx, dy])


def scan_to_map_frame(scan_pts, map_pose):
    """
    Rotate and translate robot-frame scan points into the map frame.
    map_pose: {"x", "y", "h"}
    """
    return apply_transform(scan_pts, map_pose["x"], map_pose["y"], map_pose["h"])


# ---------------------------------------------------------------------------
# ICP
# ---------------------------------------------------------------------------

def icp(source, target_kdtree, target_pts,
        max_iterations=MAX_ITERATIONS,
        convergence_threshold=CONVERGENCE_THRESHOLD,
        max_correspondence_dist=MAX_CORRESPONDENCE_DIST):
    """
    Point-to-point ICP: find the rigid transform that aligns 'source' to
    'target_pts' using Horn's closed-form SVD solution at each step.

    Args:
        source:               (N,2) source cloud — scan in current map-frame estimate.
        target_kdtree:        KDTree built from map occupied cells.
        target_pts:           (M,2) numpy array of map occupied cells (same data
                              as kdtree, kept separately for fast indexing).

    Returns:
        (dx, dy, dtheta, mean_error, converged)
        dx, dy in metres; dtheta in radians.
    """
    pts = source.copy()
    cum_R = np.eye(2)
    cum_t = np.zeros(2)

    for _ in range(max_iterations):
        dists, idx = target_kdtree.query(pts, k=1)
        dists = dists.flatten()
        idx   = idx.flatten()

        mask = dists < max_correspondence_dist
        if mask.sum() < MIN_POINTS_FOR_ICP:
            return 0.0, 0.0, 0.0, float("inf"), False

        src = pts[mask]
        tgt = target_pts[idx[mask]]

        # Optimal rotation + translation via SVD
        sc = src.mean(axis=0)
        tc = tgt.mean(axis=0)
        H  = (src - sc).T @ (tgt - tc)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:           # correct reflection
            Vt[-1] *= -1
            R = Vt.T @ U.T
        t = tc - R @ sc

        pts_new   = (R @ pts.T).T + t
        cum_t     = R @ cum_t + t
        cum_R     = R @ cum_R

        if np.mean(np.linalg.norm(pts_new - pts, axis=1)) < convergence_threshold:
            pts = pts_new
            break
        pts = pts_new

    mean_err = float(np.mean(dists[mask])) if mask.sum() > 0 else float("inf")
    dx     = float(cum_t[0])
    dy     = float(cum_t[1])
    dtheta = float(math.atan2(cum_R[1, 0], cum_R[0, 0]))
    converged = mean_err < MAX_CORRESPONDENCE_DIST
    return dx, dy, dtheta, mean_err, converged


# ---------------------------------------------------------------------------
# Localizer
# ---------------------------------------------------------------------------

class ICPLocalizer:
    """
    Maintains the map->odom correction transform via periodic ICP.

    Thread-safe: update() and get_map_pose() may be called from different
    threads (MQTT callback vs. timer).
    """

    def __init__(self):
        map_pts = load_map_pointcloud(
            MAP_FILE, MAP_RESOLUTION, MAP_ORIGIN_X, MAP_ORIGIN_Y)
        self._map_pts = map_pts
        self._kdtree  = KDTree(map_pts)
        print(f"KD-tree built over {len(map_pts)} map points")

        # map->odom correction — starts at identity (robot at known home pose)
        self.correction = {"dx": 0.0, "dy": 0.0, "dtheta_deg": 0.0}
        self._last_icp_err = 0.0   # most recent ICP mean correspondence error
        self._lock = threading.Lock()

    def update(self, scan, odom_pose):
        """
        Run one ICP cycle and update the stored correction.

        Args:
            scan:       Decoded MQTT scan payload (list of dicts).
            odom_pose:  Decoded MQTT odometry payload {"x","y","h",...}.

        Returns:
            Updated correction dict, or None if ICP failed.
        """
        scan_pts = scan_to_cartesian(scan)
        if len(scan_pts) < MIN_POINTS_FOR_ICP:
            print(f"ICP skipped — only {len(scan_pts)} usable scan points")
            return None

        # Project scan into current map-frame estimate
        map_pose  = self.get_map_pose(odom_pose)
        scan_map  = scan_to_map_frame(scan_pts, map_pose)

        t0 = time.monotonic()
        dx, dy, dtheta, mean_err, converged = icp(
            scan_map, self._kdtree, self._map_pts)
        elapsed_ms = (time.monotonic() - t0) * 1000

        print(f"ICP {'OK' if converged else 'no-converge'} "
              f"{elapsed_ms:.0f}ms | "
              f"Δ dx={dx:+.4f}m  dy={dy:+.4f}m  "
              f"dθ={math.degrees(dtheta):+.3f}°  "
              f"err={mean_err:.4f}m")

        # Reject implausibly large corrections
        if (abs(dx) > MAX_CORRECTION_JUMP or
                abs(dy) > MAX_CORRECTION_JUMP or
                abs(math.degrees(dtheta)) > MAX_CORRECTION_ROT_DEG):
            print("ICP result rejected — correction too large")
            return None

        self._last_icp_err = mean_err if converged else float("inf")

        # Blend smoothly into current correction
        alpha = CORRECTION_ALPHA
        with self._lock:
            self.correction = {
                "dx":         alpha * dx + (1 - alpha) * self.correction["dx"],
                "dy":         alpha * dy + (1 - alpha) * self.correction["dy"],
                "dtheta_deg": (alpha * math.degrees(dtheta) +
                               (1 - alpha) * self.correction["dtheta_deg"]),
            }
            return dict(self.correction)

    def get_map_pose(self, odom_pose):
        """
        Apply the stored correction to an odometry pose.

        Args:
            odom_pose: dict with at least "x", "y", "h" keys (from odometer.py).

        Returns:
            Map-frame pose {"x", "y", "h"}.
        """
        with self._lock:
            corr = dict(self.correction)
        dtheta = math.radians(corr["dtheta_deg"])
        xy_map = rot2(dtheta) @ np.array([odom_pose["x"], odom_pose["y"]])
        xy_map += np.array([corr["dx"], corr["dy"]])
        return {
            "x": float(xy_map[0]),
            "y": float(xy_map[1]),
            "h": odom_pose["h"] + dtheta,
        }


# ---------------------------------------------------------------------------
# MQTT node
# ---------------------------------------------------------------------------

def run():
    """
    Run the ICP localizer as a standalone MQTT node (paho, blocking).

    Two-tier publish strategy:

    HIGH RATE (10 Hz) — odometry is subscribed continuously.
        Every incoming odom message is immediately transformed using the
        current correction and republished on Topics.POSE.  This gives the
        path follower a fresh corrected pose at the odometry rate.

    LOW RATE (every UPDATE_INTERVAL_SEC) — ICP runs in the main thread.
        One scan is grabbed (subscribe → receive → unsubscribe), ICP runs,
        and the correction is updated.  The scan topic is live for < 200ms
        per cycle so broker load on the Pi stays low.
    """
    GRAB_TIMEOUT_SEC = 2.0

    loc        = ICPLocalizer()
    t_start    = time.monotonic()
    odom_lock  = threading.Lock()
    latest_odom = [None]          # shared between odom callback and ICP thread

    # Open CSV log
    log_fh  = open(LOG_FILE, "w", newline="")
    log_csv = csv.writer(log_fh)
    log_csv.writerow(LOG_HEADER)
    print(f"Logging to {LOG_FILE}")

    # ------------------------------------------------------------------
    # High-rate odometry callback — republishes corrected pose at 10 Hz
    # ------------------------------------------------------------------
    def on_odom(client, userdata, msg):
        odom = json.loads(msg.payload.decode())
        with odom_lock:
            latest_odom[0] = odom
        map_pose = loc.get_map_pose(odom)
        client.publish(
            TOPIC_POSE,
            json.dumps({k: round(v, 6) for k, v in map_pose.items()})
        )

    # ------------------------------------------------------------------
    # One-shot scan grab
    # ------------------------------------------------------------------
    def grab_scan(client):
        received = threading.Event()
        hold     = [None]

        def on_msg(client, userdata, msg):
            hold[0] = json.loads(msg.payload.decode())
            received.set()

        client.message_callback_add(TOPIC_SCAN, on_msg)
        client.subscribe(TOPIC_SCAN)
        received.wait(timeout=GRAB_TIMEOUT_SEC)
        client.unsubscribe(TOPIC_SCAN)
        client.message_callback_remove(TOPIC_SCAN)
        return hold[0]

    # ------------------------------------------------------------------
    # Connect — subscribe to odom continuously, scan is one-shot
    # ------------------------------------------------------------------
    connected = threading.Event()

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print(f"Connected to broker at {BROKER_ADDRESS}")
            client.subscribe(TOPIC_ODOM)
            print(f"Subscribed continuously to {TOPIC_ODOM}  (high-rate pose republish)")
            print(f"Scan grabbed one-shot every {UPDATE_INTERVAL_SEC}s  (ICP update)")
            connected.set()
        else:
            print(f"Connection failed, rc={rc}")

    client = mqtt.Client()
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.on_connect = on_connect
    client.message_callback_add(TOPIC_ODOM, on_odom)
    client.connect(BROKER_ADDRESS, BROKER_PORT, 60)
    client.loop_start()

    print("ICP localizer starting — waiting for broker connection...")
    connected.wait()

    try:
        while True:
            # Sleep between ICP cycles — odom callback runs in background
            time.sleep(UPDATE_INTERVAL_SEC)

            cycle_start = time.monotonic()

            # Grab one lidar scan (subscribe → receive → unsubscribe)
            scan = grab_scan(client)
            if scan is None:
                print("Scan timeout — skipping ICP cycle")
                continue

            # Use the most recent odom snapshot for ICP
            with odom_lock:
                odom = latest_odom[0]
            if odom is None:
                print("No odom yet — skipping ICP cycle")
                continue

            elapsed_grab = (time.monotonic() - cycle_start) * 1000
            print(f"Grabbed scan in {elapsed_grab:.0f}ms", end="  |  ")

            # Run ICP
            correction = loc.update(scan, odom)

            # Publish correction transform
            if correction is not None:
                client.publish(
                    TOPIC_CORRECTION,
                    json.dumps({k: round(v, 6) for k, v in correction.items()})
                )

            # Log this ICP cycle
            cycle_ms = (time.monotonic() - cycle_start) * 1000
            corr     = loc.correction
            map_pose = loc.get_map_pose(odom)
            log_csv.writerow([
                round(time.monotonic() - t_start, 3),
                round(odom["x"], 4),     round(odom["y"], 4),
                round(math.degrees(odom["h"]), 3),
                round(map_pose["x"], 4), round(map_pose["y"], 4),
                round(math.degrees(map_pose["h"]), 3),
                round(corr["dx"], 4),    round(corr["dy"], 4),
                round(corr["dtheta_deg"], 3),
                round(loc._last_icp_err, 4),
                round(cycle_ms, 1),
            ])
            log_fh.flush()

    except KeyboardInterrupt:
        print("ICP localizer stopped.")
    finally:
        log_fh.close()
        print(f"Log saved to {LOG_FILE}")
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    run()
